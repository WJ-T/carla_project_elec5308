# auto_pilot_with_npc.py —— 不改地图 + 同步模式 + 主角&NPC自动驾驶(TM) + 平滑跟随 + 自适应节拍
import time, math, random
import carla

HOST="127.0.0.1"; PORT=2000
TM_PORT=8002

# ====== 可调参数 ======
FIXED_DT = 0.01            # 仿真步长（0.01更丝滑，性能不足可改 0.02~0.05）
CAM_ALPHA = 0.20           # 相机平滑系数（0.1更稳，0.3更跟手）
CAM_DIST  = 8.0            # 相机在车后距离（米）
CAM_HEIGHT= 3.0
CAM_PITCH = -12.0

NUM_NPC   = 60             # 生成的 NPC 数量
MIN_GAP_M = 6.0            # 与前车的最低目标距离（米）
HERO_SPEED_DELTA = -90     # 主角相对限速的百分比（负数=更快，正数=更慢）
NPC_SPEED_MIN    = 0       # NPC个体速度差范围（百分比）
NPC_SPEED_MAX    = 15      # 例如 [0,15] 表示比限速慢 0~15%
AUTO_LC_HERO     = True    # 主角是否允许自动变道
AUTO_LC_NPC      = True    # NPC 是否允许自动变道
IGNORE_LIGHTS_P  = 0.0     # 忽略红绿灯概率[0,100]
IGNORE_SIGNS_P   = 0.0     # 忽略交通标志概率[0,100]

# ====== 工具函数 ======
def lerp(a,b,t): return a + (b-a)*t

def follow_transform(t):
    """根据车辆 transform 计算相机 transform"""
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
    bps = bp_lib.filter(bp_filter)
    if not bps:
        bps = bp_lib.filter("vehicle.*")
    bp = random.choice(bps)
    spawns = world.get_map().get_spawn_points()
    random.shuffle(spawns)
    for sp in spawns:
        v = world.try_spawn_actor(bp, sp)
        if v:
            return v
    return None

def spawn_npcs(world, count):
    out = []
    for _ in range(count):
        v = spawn_one(world, "vehicle.*")
        if v: out.append(v)
        if len(out) >= count: break
    return out

def shorten_all_lights(world, green=3.0, yellow=1.0, red=3.0):
    """缩短全局红绿灯周期"""
    tls = world.get_actors().filter('traffic.traffic_light')
    for tl in tls:
        try:
            tl.set_green_time(green)
            tl.set_yellow_time(yellow)
            tl.set_red_time(red)
        except RuntimeError:
            pass
    print(f"[LIGHTS] cycles set to G={green}s Y={yellow}s R={red}s")

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)

    # —— 随机地图 ——
    maps = ["Town01","Town02","Town03","Town04","Town05","Town07","Town10HD_Opt"]
    chosen = random.choice(maps)
    world = client.load_world(chosen)
    print(f"[MAP] Loaded {chosen}")
    shorten_all_lights(world, green=3.0, yellow=1.0, red=1.0)

    # —— 进入同步模式（保存&还原设置） ——
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DT
    world.apply_settings(settings)

    # —— Traffic Manager 设置 ——
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)
    tm.set_global_distance_to_leading_vehicle(MIN_GAP_M)
    tm.global_percentage_speed_difference(0)

    # —— 生成主角（优先 model3） ——
    hero = spawn_one(world, "vehicle.tesla.model3")
    if not hero:
        world.apply_settings(original_settings)
        raise RuntimeError("主角 spawn 失败（出生点被占/资源不足）。")
    print("[HERO]", hero.id, hero.type_id)

    # —— 生成 NPC 若干 ——
    npcs = spawn_npcs(world, NUM_NPC)
    print(f"[NPC] spawned: {len(npcs)}")

    # —— Autopilot 设置 ——
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
        slow_pct = random.randint(NPC_SPEED_MIN, NPC_SPEED_MAX)
        tm.vehicle_percentage_speed_difference(v, slow_pct)

    # —— 相机跟随 ——
    spectator = world.get_spectator()
    world.tick()
    curr_tf = follow_transform(hero.get_transform())
    spectator.set_transform(curr_tf)

    print("主角+NPC 自动驾驶（TM）运行中...  Ctrl+C 退出（不销毁）")

    last_tick = time.perf_counter()

    try:
        while True:
            world.tick()

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

            # —— 自适应节拍 ——（避免 tick 本身已慢还继续 sleep）
            now = time.perf_counter()
            elapsed = now - last_tick
            if elapsed < FIXED_DT:
                time.sleep(FIXED_DT - elapsed)
            last_tick = time.perf_counter()

    except KeyboardInterrupt:
        print("\n退出脚本（车辆仍在场景中；设置已还原）。")
    finally:
        try:
            tm.set_synchronous_mode(False)
        except Exception:
            pass
        world.apply_settings(original_settings)

if __name__ == "__main__":
    main()
