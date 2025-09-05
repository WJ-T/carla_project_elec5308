# auto_pilot_with_npc.py —— 不改地图 + 同步模式 + 主角&NPC自动驾驶(TM)
# + 相机平滑 + 红绿灯缩短 + 主角“优先变道再跟随”
import time, math, random
import carla

HOST="127.0.0.1"; PORT=2000
TM_PORT=8002

# ====== 可调参数 ======
FIXED_DT = 0.01            # 仿真步长
CAM_ALPHA = 0.20           # 相机平滑系数
CAM_DIST  = 8.0            # 相机在车后距离
CAM_HEIGHT= 3.0
CAM_PITCH = -12.0

NUM_NPC   = 100             # NPC 数量
MIN_GAP_M = 3.0            # TM 全局跟车最小距离
HERO_SPEED_DELTA = -100     # 主角相对限速的百分比（负数=更快）
NPC_SPEED_MIN    = 0       # NPC 慢速随机区间[%]
NPC_SPEED_MAX    = 15
AUTO_LC_HERO     = True    # 允许主角自动变道（仍由我们优先触发）
AUTO_LC_NPC      = True
IGNORE_LIGHTS_P  = 0.0
IGNORE_SIGNS_P   = 0.0

# —— 主角优先变道策略参数 ——
DECIDE_PERIOD = 0.5        # 每多少秒评估一次
BLOCKED_DIST  = 12.0       # 认为被前车“堵住”的阈值（米）
SAFE_AHEAD    = 15.0       # 目标车道前方安全距离
SAFE_BEHIND   = 10.0       # 目标车道后方安全距离
CHECK_RADIUS  = 30.0       # 搜索附近车辆的半径（米）

def lerp(a,b,t): return a + (b-a)*t

def follow_transform(t):
    yaw_rad = math.radians(t.rotation.yaw)
    cam_loc = carla.Location(
        x=t.location.x - CAM_DIST*math.cos(yaw_rad),
        y=t.location.y - CAM_DIST*math.sin(yaw_rad),
        z=t.location.z + CAM_HEIGHT
    )
    cam_rot = carla.Rotation(pitch=CAM_PITCH, yaw=t.rotation.yaw, roll=0.0)
    return carla.Transform(cam_loc, cam_rot)

def spawn_one(world, bp_filter="vehicle.*"):
    bp_lib = world.get_blueprint_library()
    bps = bp_lib.filter(bp_filter) or bp_lib.filter("vehicle.*")
    bp = random.choice(bps)
    spawns = world.get_map().get_spawn_points()
    random.shuffle(spawns)
    for sp in spawns:
        v = world.try_spawn_actor(bp, sp)
        if v: return v
    return None

def spawn_npcs(world, count):
    out = []
    for _ in range(count):
        v = spawn_one(world, "vehicle.*")
        if v: out.append(v)
        if len(out) >= count: break
    return out

def shorten_all_lights(world, green=3.0, yellow=1.0, red=3.0):
    tls = world.get_actors().filter('traffic.traffic_light')
    for tl in tls:
        try:
            tl.set_green_time(green)
            tl.set_yellow_time(yellow)
            tl.set_red_time(red)
        except RuntimeError:
            pass
    print(f"[LIGHTS] cycles set to G={green}s Y={yellow}s R={red}s")

# ========= 车道/安全判断与决策 =========
def _lane_forward_vec(yaw_deg: float):
    r = math.radians(yaw_deg)
    return math.cos(r), math.sin(r)

def _nearest_leader_same_lane(world, hero):
    """返回：与主角同车道且在前方的最近车辆距离（米）；找不到则 inf"""
    m = world.get_map()
    htf = hero.get_transform()
    hwp = m.get_waypoint(htf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
    hx, hy = htf.location.x, htf.location.y
    fx, fy = _lane_forward_vec(htf.rotation.yaw)

    best = float('inf')
    for a in world.get_actors().filter('vehicle.*'):
        if a.id == hero.id: continue
        wp = m.get_waypoint(a.get_transform().location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if not wp or not hwp: continue
        # 同一条车道（用 road_id + lane_id 粗略判断）
        if wp.road_id != hwp.road_id or wp.lane_id != hwp.lane_id: continue
        loc = a.get_transform().location
        dx, dy = loc.x - hx, loc.y - hy
        proj = dx*fx + dy*fy
        lateral = -dx*fy + dy*fx
        if proj > 0 and abs(lateral) < 2.0:
            d = math.hypot(dx, dy)
            if d < best: best = d
    return best

def _adjacent_lane(hwp, side: str):
    """获取相邻车道 waypoint（左/右）；不可用时返回 None"""
    if side == 'left':
        adj = hwp.get_left_lane()
        allow = bool(hwp.lane_change & carla.LaneChange.Left)
    else:
        adj = hwp.get_right_lane()
        allow = bool(hwp.lane_change & carla.LaneChange.Right)
    if not adj or adj.lane_type != carla.LaneType.Driving: return None
    if not allow: return None
    return adj

def _lane_safe(world, adj_wp, side_forward_yaw, hero_loc):
    """判断目标相邻车道是否安全：前≥SAFE_AHEAD、后≥SAFE_BEHIND"""
    if adj_wp is None: return False
    fx, fy = _lane_forward_vec(side_forward_yaw)
    hx, hy = hero_loc.x, hero_loc.y
    safe_ahead = True
    safe_behind = True

    for a in world.get_actors().filter('vehicle.*'):
        loc = a.get_transform().location
        # 半径粗筛
        if loc.distance(hero_loc) > CHECK_RADIUS: continue

        wp = world.get_map().get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if not wp: continue
        # 必须在目标车道（同 road_id + lane_id）
        if wp.road_id != adj_wp.road_id or wp.lane_id != adj_wp.lane_id: continue

        dx, dy = loc.x - hx, loc.y - hy
        proj = dx*fx + dy*fy    # 沿目标车道前向的投影
        lat  = -dx*fy + dy*fx   # 目标车道横向偏移
        if abs(lat) > 2.5:      # 横向偏太多就不算冲突
            continue
        d = math.hypot(dx, dy)
        if proj >= 0:   # 在前方
            if d < SAFE_AHEAD: safe_ahead = False
        else:           # 在后方
            if d < SAFE_BEHIND: safe_behind = False

        if not safe_ahead or not safe_behind:
            return False

    return True

def _try_force_lane_change(tm, hero, prefer_left=True):
    """兼容不同版本的 TM force_lane_change 签名"""
    if prefer_left:
        # 试左
        try:
            tm.force_lane_change(hero, True)   # 某些版本支持方向参数
            return True
        except TypeError:
            tm.force_lane_change(hero)         # 某些版本无方向参数（随机/可行即换）
            return True
    else:
        # 试右
        try:
            tm.force_lane_change(hero, False)
            return True
        except TypeError:
            tm.force_lane_change(hero)
            return True

def decide_lane_change(world, tm, hero):
    """被前车顶到阈值内 → 先左后右地判定相邻车道是否“存在且安全”；可则强制变道"""
    m = world.get_map()
    htf = hero.get_transform()
    hwp = m.get_waypoint(htf.location, project_to_road=True, lane_type=carla.LaneType.Driving)

    # 1) 前方最近同道车距离
    dist_front = _nearest_leader_same_lane(world, hero)
    if dist_front >= BLOCKED_DIST:
        return  # 不拥堵，顺其自然

    # 2) 计算左右相邻车道 waypoint 及其车道前向
    left_wp  = _adjacent_lane(hwp, 'left')
    right_wp = _adjacent_lane(hwp, 'right')

    left_yaw  = (left_wp.transform.rotation.yaw  if left_wp  else htf.rotation.yaw)
    right_yaw = (right_wp.transform.rotation.yaw if right_wp else htf.rotation.yaw)

    # 3) 安全性判定：优先左，再右
    if _lane_safe(world, left_wp, left_yaw, htf.location):
        _try_force_lane_change(tm, hero, prefer_left=True)
        return
    if _lane_safe(world, right_wp, right_yaw, htf.location):
        _try_force_lane_change(tm, hero, prefer_left=False)
        return
    # 两侧都不安全：保持跟随（交给 TM 按 MIN_GAP_M 控制）

# ============= 主流程 =============
def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(30.0)

    # 随机地图
    maps = ["Town01","Town02","Town03","Town04","Town05","Town07","Town10HD_Opt"]
    chosen = random.choice(maps)
    world = client.load_world(chosen)
    print(f"[MAP] Loaded {chosen}")
    shorten_all_lights(world, green=3.0, yellow=1.0, red=1.0)

    # 同步模式
    original = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = FIXED_DT
    world.apply_settings(s)

    # TM
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)
    tm.set_global_distance_to_leading_vehicle(MIN_GAP_M)
    tm.global_percentage_speed_difference(0)

    # 主角 & NPC
    hero = spawn_one(world, "vehicle.tesla.model3")
    if not hero:
        world.apply_settings(original)
        raise RuntimeError("主角 spawn 失败")
    print("[HERO]", hero.id, hero.type_id)

    npcs = spawn_npcs(world, NUM_NPC)
    print(f"[NPC] spawned: {len(npcs)}")

    hero.set_autopilot(True, TM_PORT)
    tm.vehicle_percentage_speed_difference(hero, HERO_SPEED_DELTA)
    tm.auto_lane_change(hero, AUTO_LC_HERO)
    tm.ignore_lights_percentage(hero, IGNORE_LIGHTS_P)
    tm.ignore_signs_percentage(hero, IGNORE_SIGNS_P)

    for v in npcs:
        v.set_autopilot(True, TM_PORT)
        tm.auto_lane_change(v, AUTO_LC_NPC)
        tm.ignore_lights_percentage(v, IGNORE_LIGHTS_P)
        tm.ignore_signs_percentage(v, IGNORE_SIGNS_P)
        tm.vehicle_percentage_speed_difference(v, random.randint(NPC_SPEED_MIN, NPC_SPEED_MAX))

    # 相机
    spectator = world.get_spectator()
    world.tick()
    curr_tf = follow_transform(hero.get_transform())
    spectator.set_transform(curr_tf)

    print("运行中（主角优先变道策略已启用）…  Ctrl+C 退出（不销毁）")

    last_tick = time.perf_counter()
    last_decide = last_tick

    try:
        while True:
            world.tick()

            # 每 DECIDE_PERIOD 评估一次是否要变道
            now = time.perf_counter()
            if now - last_decide >= DECIDE_PERIOD:
                decide_lane_change(world, tm, hero)
                last_decide = now

            # 相机平滑
            tgt = follow_transform(hero.get_transform())
            sm_loc = carla.Location(
                x=lerp(curr_tf.location.x, tgt.location.x, CAM_ALPHA),
                y=lerp(curr_tf.location.y, tgt.location.y, CAM_ALPHA),
                z=lerp(curr_tf.location.z, tgt.location.z, CAM_ALPHA)
            )
            sm_rot = carla.Rotation(
                pitch=lerp(curr_tf.rotation.pitch, tgt.rotation.pitch, CAM_ALPHA),
                yaw=lerp(curr_tf.rotation.yaw,   tgt.rotation.yaw,   CAM_ALPHA),
                roll=0.0
            )
            curr_tf = carla.Transform(sm_loc, sm_rot)
            spectator.set_transform(curr_tf)

            # 自适应节拍（tick 已慢则不再 sleep）
            elapsed = now - last_tick
            if elapsed < FIXED_DT:
                time.sleep(FIXED_DT - elapsed)
            last_tick = time.perf_counter()

    except KeyboardInterrupt:
        print("\n退出脚本（车辆仍在场景中；设置已还原）。")
    finally:
        try: tm.set_synchronous_mode(False)
        except Exception: pass
        world.apply_settings(original)

if __name__ == "__main__":
    main()
