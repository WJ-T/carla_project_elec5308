# hero_avoid_follow.py
# 需求：不固定地图；生成多辆NPC；相机跟随主角；主角具备简单规避（避让NPC）能力；不销毁

import math, random, time
import carla

HOST="127.0.0.1"; PORT=2000
TM_PORT=8002

# ====== 可调参数 ======
NUM_NPC = 20            # 生成 NPC 数量
TARGET_SPEED = 14.0     # 主角目标速度 m/s (~50km/h)
MAX_STEER = 0.6         # 主角最大转向幅度（绝对值）
SAFE_HEADWAY = 12.0     # 与前车的安全跟车距离（米）
BRAKE_DISTANCE = 8.0    # 强制刹车距离（米）
AVOID_RADIUS = 10.0     # 侧向避让的影响半径（米）
AVOID_GAIN = 0.03       # 侧向避让强度（越大越容易“别”开）
ALPHA = 0.2             # 跟随相机平滑系数（0.1更平顺，0.3更跟手）
CAM_DIST = 8.0          # 相机在车后距离（米）
CAM_HEIGHT = 3.0        # 相机高度（米）
CAM_PITCH = -12.0       # 相机俯角（度）
# =====================

def lerp(a,b,t): return a + (b-a)*t

def calc_speed(v):
    return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

def angle_diff(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return d

def follow_tf_from_vehicle(t):
    """根据车辆 transform 计算跟车相机 transform"""
    yaw = math.radians(t.rotation.yaw)
    loc = carla.Location(
        x=t.location.x - CAM_DIST*math.cos(yaw),
        y=t.location.y - CAM_DIST*math.sin(yaw),
        z=t.location.z + CAM_HEIGHT
    )
    rot = carla.Rotation(pitch=CAM_PITCH, yaw=t.rotation.yaw, roll=0.0)
    return carla.Transform(loc, rot)

def spawn_hero(world):
    bp_lib = world.get_blueprint_library()
    # 优先选 model3，找不到就任意 vehicle.*
    cand = bp_lib.filter("vehicle.tesla.model3")
    bp = cand[0] if cand else bp_lib.filter("vehicle.*")[0]
    spawns = world.get_map().get_spawn_points()
    random.shuffle(spawns)
    for sp in spawns:
        veh = world.try_spawn_actor(bp, sp)
        if veh:
            print("[HERO] spawned at:", veh.get_transform().location)
            return veh
    raise RuntimeError("没有可用出生点，清场或换点再试。")

def spawn_npcs(world, n):
    bp_lib = world.get_blueprint_library()
    spawns = world.get_map().get_spawn_points()
    random.shuffle(spawns)
    vehicles = []
    for sp in spawns:
        if len(vehicles) >= n: break
        bp = random.choice(bp_lib.filter("vehicle.*"))
        npc = world.try_spawn_actor(bp, sp)
        if npc:
            vehicles.append(npc)
    print(f"[NPC] spawned: {len(vehicles)} / {n}")
    return vehicles

def setup_tm(client, npcs):
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_global_distance_to_leading_vehicle(1.0)
    tm.global_percentage_speed_difference(10)  # 所有 NPC 稍微慢一点
    for v in npcs:
        v.set_autopilot(True, TM_PORT)
    return tm

def hero_avoid_control(world, hero):
    """
    简单双层策略：
    1) 车道跟随：沿着路网 waypoint 前瞻方向转向；
    2) 避让：
       - 前向最近车辆：距离 < SAFE_HEADWAY → 减小油门；< BRAKE_DISTANCE → 刹车；
       - 侧向车辆：距离 < AVOID_RADIUS → 根据横向相对位置施加转向修正（把方向打离它）。
    3) 油门 PID（简化）：按速度误差调整 throttle。
    """
    m = world.get_map()
    hero_tf = hero.get_transform()
    hero_wp = m.get_waypoint(hero_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)

    # 前瞻 waypoint -> 目标方向
    lookahead = 6.0  # 前瞻距离（米）
    wps = hero_wp.next(lookahead)
    if not wps:
        target_yaw = hero_tf.rotation.yaw
    else:
        target_yaw = wps[0].transform.rotation.yaw

    # 基础转向：朝向目标 yaw
    yaw_err = angle_diff(target_yaw, hero_tf.rotation.yaw) / 90.0  # 归一化到[-1,1]附近
    steer_cmd = max(-MAX_STEER, min(MAX_STEER, 0.6 * yaw_err))

    # 感知周边车辆
    actors = world.get_actors().filter('vehicle.*')
    hx, hy = hero_tf.location.x, hero_tf.location.y
    yaw_rad = math.radians(hero_tf.rotation.yaw)
    front_vec = (math.cos(yaw_rad), math.sin(yaw_rad))

    min_front_dist = float('inf')
    for v in actors:
        if v.id == hero.id: continue
        loc = v.get_transform().location
        dx, dy = loc.x - hx, loc.y - hy
        # 前后判断：与前向向量点积>0 表示在前方
        front_proj = dx*front_vec[0] + dy*front_vec[1]
        lateral = -dx*front_vec[1] + dy*front_vec[0]  # 右为正、左为负（车辆局部坐标）
        dist2 = dx*dx + dy*dy
        dist = math.sqrt(dist2)

        # 前车距离（用于跟车/刹车）
        if front_proj > 0 and abs(lateral) < 3.0:  # 同车道或相邻、在正前方
            if dist < min_front_dist:
                min_front_dist = dist

        # 侧向避让：在影响半径内，根据横向位置对 steer 做修正（把方向打离它）
        if dist < AVOID_RADIUS:
            steer_cmd += AVOID_GAIN * (-lateral)  # lateral>0在右侧 → 往左打（负号）；反之亦然

    # 油门/刹车
    v = hero.get_velocity()
    speed = calc_speed(v)
    throttle = 0.0
    brake = 0.0

    # 跟车策略
    if min_front_dist < BRAKE_DISTANCE:
        brake = 0.7
    elif min_front_dist < SAFE_HEADWAY:
        # 逼近前车则减速：把目标速度降到前车距离/SAFE_HEADWAY 的比例
        reduce = max(0.3, min(1.0, (min_front_dist - BRAKE_DISTANCE)/(SAFE_HEADWAY - BRAKE_DISTANCE)))
        target_v = TARGET_SPEED * reduce
        err = max(0.0, target_v - speed)
        throttle = max(0.0, min(0.6, 0.15*err))
    else:
        # 正常巡航 PID（简化）
        err = TARGET_SPEED - speed
        throttle = max(0.0, min(0.8, 0.12*err))

    steer_cmd = max(-MAX_STEER, min(MAX_STEER, steer_cmd))
    hero.apply_control(carla.VehicleControl(throttle=throttle, steer=steer_cmd, brake=brake))

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)
    world = client.get_world()  # 不切换地图，使用当前已加载地图
    print("[WORLD]", world.get_map().name)

    # 生成主角与 NPC
    hero = spawn_hero(world)
    npcs = spawn_npcs(world, NUM_NPC)
    tm = setup_tm(client, npcs)

    # 相机跟随（异步 + 平滑），避免“快进感”
    spectator = world.get_spectator()
    world.wait_for_tick()
    t = hero.get_transform()
    cam_tf = follow_tf_from_vehicle(t)
    spectator.set_transform(cam_tf)

    print("运行中：主角自主规避；相机跟随；Ctrl+C 结束（不销毁车辆）")
    try:
        while True:
            world.wait_for_tick()  # 与服务器帧同步（异步模式下按实时来）
            # 主角控制
            hero_avoid_control(world, hero)
            # 平滑相机
            t = hero.get_transform()
            tgt = follow_tf_from_vehicle(t)
            # 位置平滑
            cur_loc = cam_tf.location; tgt_loc = tgt.location
            sm_loc = carla.Location(
                x=lerp(cur_loc.x, tgt_loc.x, ALPHA),
                y=lerp(cur_loc.y, tgt_loc.y, ALPHA),
                z=lerp(cur_loc.z, tgt_loc.z, ALPHA)
            )
            # 朝向平滑
            cur_rot = cam_tf.rotation; tgt_rot = tgt.rotation
            sm_rot = carla.Rotation(
                pitch=lerp(cur_rot.pitch, tgt_rot.pitch, ALPHA),
                yaw=lerp(cur_rot.yaw, tgt_rot.yaw, ALPHA),
                roll=0.0
            )
            cam_tf = carla.Transform(sm_loc, sm_rot)
            spectator.set_transform(cam_tf)

    except KeyboardInterrupt:
        print("\n退出脚本：车辆与NPC都保留在场景中。")
    finally:
        # 不销毁任何车；Traffic Manager 维持默认异步即可
        pass

if __name__ == "__main__":
    main()
