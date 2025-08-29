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
    改进：Stanley 车道保持 + 路口/大曲率限速 + 边界排斥 + 温和避让
    """
    m = world.get_map()
    tf = hero.get_transform()
    v = hero.get_velocity()
    speed = calc_speed(v) + 1e-3

    # 1) 取得当前车道中心（用于横向误差）和前瞻路径（用于航向误差）
    wp = m.get_waypoint(tf.location, project_to_road=True,
                        lane_type=carla.LaneType.Driving)

    # 路口/分叉时，挑选“与当前车头夹角最小”的下一条路
    lookahead = 6.0
    next_wps = wp.next(lookahead)
    if not next_wps:
        target_wp = wp
    else:
        def yaw_diff(w):
            return abs(angle_diff(w.transform.rotation.yaw, tf.rotation.yaw))
        target_wp = min(next_wps, key=yaw_diff)

    target_yaw = target_wp.transform.rotation.yaw

    # 2) Stanley 控制器（heading + cross track）
    # 车辆坐标系前向（单位向量）
    yaw = math.radians(tf.rotation.yaw)
    front_vec = (math.cos(yaw), math.sin(yaw))

    # 车道切线朝向
    tyaw = math.radians(target_yaw)
    tangent = (math.cos(tyaw), math.sin(tyaw))

    # 横向误差：英雄车位置到道路中心的矢量，在车辆坐标系的“侧向”分量
    dx = target_wp.transform.location.x - tf.location.x
    dy = target_wp.transform.location.y - tf.location.y
    lateral_err = -dx*front_vec[1] + dy*front_vec[0]  # 右正左负

    # 航向误差
    heading_err = math.radians(angle_diff(target_yaw, tf.rotation.yaw))

    # Stanley：δ = ψ_err + atan(k * e / (v + ε))
    K_STANLEY = 1.0
    steer_cmd = heading_err + math.atan2(K_STANLEY * lateral_err, speed)

    # 3) 边界排斥（靠近车道边缘时，往中心推）
    lane_w = wp.lane_width if wp.lane_width > 0 else 3.5
    # 以中心为 0，绝对值越大越接近路沿
    center_offset = lateral_err
    ratio = abs(center_offset) / (0.5 * lane_w)  # >1 表示越界
    if ratio > 0.6:  # 开始感到“挤边”
        repel = 0.5 * (ratio - 0.6)  # 0~0.5 的附加项
        steer_cmd += -math.copysign(repel, center_offset)  # 往中心打

    # 4) 周围车辆避让（温和版）
    AVOID_R = 10.0
    AVOID_K = 0.015  # 比你原来小，避免被 NPC 直接“别到路沿”
    min_front = float('inf')
    actors = world.get_actors().filter('vehicle.*')
    hx, hy = tf.location.x, tf.location.y
    for a in actors:
        if a.id == hero.id: 
            continue
        loc = a.get_transform().location
        dx, dy = loc.x - hx, loc.y - hy
        front_proj = dx*front_vec[0] + dy*front_vec[1]
        lateral = -dx*front_vec[1] + dy*front_vec[0]
        dist = math.hypot(dx, dy)

        # 前向同车道“跟车距离”
        if front_proj > 0 and abs(lateral) < 3.0:
            min_front = min(min_front, dist)

        # 侧向避让（在影响半径内：往远离它的方向打一点点）
        if dist < AVOID_R:
            steer_cmd += AVOID_K * (-lateral)

    # 5) 弯道/路口限速：大转角或在 junction 内 → 降速
    TARGET_V = TARGET_SPEED
    corner = abs(angle_diff(target_yaw, tf.rotation.yaw))
    if target_wp.is_junction or corner > 25:  # 路口或大弯
        TARGET_V = min(TARGET_V, 8.0)         # ~30 km/h
    if corner > 45:
        TARGET_V = min(TARGET_V, 6.0)

    # 跟车再降速
    if min_front < BRAKE_DISTANCE:
        brake = 0.7; throttle = 0.0
    elif min_front < SAFE_HEADWAY:
        scale = max(0.3, min(1.0, (min_front - BRAKE_DISTANCE)/
                              (SAFE_HEADWAY - BRAKE_DISTANCE)))
        TARGET_V *= scale
        brake = 0.0
        throttle = max(0.0, min(0.6, 0.15*(TARGET_V - speed)))
    else:
        brake = 0.0
        throttle = max(0.0, min(0.8, 0.12*(TARGET_V - speed)))

    # 6) 限幅并下发控制
    steer_cmd = float(max(-MAX_STEER, min(MAX_STEER, steer_cmd)))
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
