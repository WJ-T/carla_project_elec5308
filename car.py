# auto_pilot_with_npc.py —— 不改地图 + 同步模式 + 主角&NPC自动驾驶(TM) + 平滑跟随
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

NUM_NPC   = 18             # 生成的 NPC 数量（核显建议 8~15）
MIN_GAP_M = 8.0            # ★ 与前车的最低目标距离（米）（全局）
HERO_SPEED_DELTA = -40       # 主角相对限速的百分比（负数=更快，正数=更慢）
NPC_SPEED_MIN    = 0       # NPC个体速度差范围（百分比）
NPC_SPEED_MAX    = 15      # 例如 [0,15] 表示比限速慢 0~15%
AUTO_LC_HERO     = True    # 主角是否允许自动变道
AUTO_LC_NPC      = True    # NPC 是否允许自动变道
IGNORE_LIGHTS_P  = 0.0     # 忽略红绿灯概率[0,100]（保持 0 最守规矩）
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

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(20.0)

    # 只使用当前地图
    world = client.get_world()
    print("[MAP]", world.get_map().name)

    # —— 进入同步模式（保存&还原设置） ——
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DT
    world.apply_settings(settings)

    # —— Traffic Manager 设置 ——
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)

    # 全局“最低距离”与速度偏差（注意：TM 用“期望最小间距”来影响跟车）
    tm.set_global_distance_to_leading_vehicle(MIN_GAP_M)
    tm.global_percentage_speed_difference(0)  # 仅作用于未单独设置过的车辆

    # —— 生成主角（优先 model3） ——
    hero = spawn_one(world, "vehicle.tesla.model3")
    if not hero:
        world.apply_settings(original_settings)
        raise RuntimeError("主角 spawn 失败（出生点被占/资源不足）。")
    print("[HERO]", hero.id, hero.type_id)

    # —— 生成 NPC 若干 ——
    npcs = spawn_npcs(world, NUM_NPC)
    print(f"[NPC] spawned: {len(npcs)}")

    # —— 将全部车辆交给 TM 自动驾驶，并按角色设置参数 ——
    # 主角
    hero.set_autopilot(True, TM_PORT)
    tm.vehicle_percentage_speed_difference(hero, HERO_SPEED_DELTA)  # 相对限速的百分比
    tm.auto_lane_change(hero, AUTO_LC_HERO)
    tm.ignore_lights_percentage(hero, IGNORE_LIGHTS_P)
    tm.ignore_signs_percentage(hero, IGNORE_SIGNS_P)

    # NPC：给每台车一个轻微不同的速度偏差，允许变道
    for v in npcs:
        v.set_autopilot(True, TM_PORT)
        tm.auto_lane_change(v, AUTO_LC_NPC)
        tm.ignore_lights_percentage(v, IGNORE_LIGHTS_P)
        tm.ignore_signs_percentage(v, IGNORE_SIGNS_P)
        # 让 NPC 整体“比限速慢一点”，促使后车在安全时变道绕行
        slow_pct = random.randint(NPC_SPEED_MIN, NPC_SPEED_MAX)
        tm.vehicle_percentage_speed_difference(v, slow_pct)
        # 可选：给轻微的车道内侧向偏移，显得更自然
        # tm.vehicle_lane_offset(v, random.uniform(-0.2, 0.2))

    # —— 相机：平滑追尾主角 ——
    spectator = world.get_spectator()
    world.tick()  # 先推进一帧，确保有有效 transform
    curr_tf = follow_transform(hero.get_transform())
    spectator.set_transform(curr_tf)

    print("主角+NPC 自动驾驶（TM）运行中...  Ctrl+C 退出（不销毁）")
    try:
        while True:
            world.tick()  # 与模拟同步推进

            target_tf = follow_transform(hero.get_transform())
            # 位置平滑
            cur_loc = curr_tf.location
            tgt_loc = target_tf.location
            sm_loc = carla.Location(
                x=lerp(cur_loc.x, tgt_loc.x, CAM_ALPHA),
                y=lerp(cur_loc.y, tgt_loc.y, CAM_ALPHA),
                z=lerp(cur_loc.z, tgt_loc.z, CAM_ALPHA)
            )
            # 朝向平滑
            cur_rot = curr_tf.rotation
            tgt_rot = target_tf.rotation
            sm_rot = carla.Rotation(
                pitch=lerp(cur_rot.pitch, tgt_rot.pitch, CAM_ALPHA),
                yaw=lerp(cur_rot.yaw, tgt_rot.yaw, CAM_ALPHA),
                roll=0.0
            )

            curr_tf = carla.Transform(sm_loc, sm_rot)
            spectator.set_transform(curr_tf)

            # 避免“快进”（按 FIXED_DT 节拍跑）
            time.sleep(FIXED_DT)

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
