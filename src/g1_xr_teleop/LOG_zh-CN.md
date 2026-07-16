# g1_xr_teleop 修改日志

## 2026-07-06

- 在 `g1_control_ws/src` 下新增 `g1_xr_teleop` ROS 2 Python 包。
- 新增包元数据：`package.xml`、`setup.py`、`setup.cfg`、resource marker。
- 新增默认配置文件 `config/xr_teleop.yaml`，默认 `dry_run: true`，避免启动后直接控制机器人。
- 新增 launch 文件 `launch/g1_xr_teleop.launch.py`，支持覆盖 `dry_run`、`enable_xr`、`enable_ik`。
- 新增主节点 `xr_teleop_node.py`，实现 `/g1/state/arm`、`/g1/state/controller_status` 订阅，以及 `/g1/command/arm_joints`、`/g1/command/loco` 发布骨架。
- 新增 `xr_input_adapter.py`，用于包装 Unitree 官方 `TeleVuerWrapper`；默认不启用 XR，避免缺少官方子模块时影响 ROS 2 包构建。
- 新增 `arm_ik_adapter.py`，用于包装 Unitree 官方 G1 IK；初始化时处理官方 IK 的相对 URDF 路径。
- 新增 `arm_command_mapper.py`、`loco_command_mapper.py`、`safety_gate.py`、`calibration.py` 和 `teleop_types.py`，分别负责命令转换、安全门、标定占位和内部数据结构。
- 将官方 `xr_teleoperate/assets/g1` 复制到本包 `g1_xr_teleop/assets/g1`，包含 G1 URDF、MJCF 和 mesh 文件，供本包 IK 独立使用。
- 将官方 G1 IK 文件和 `weighted_moving_filter.py` 复制到 `g1_xr_teleop/third_party/unitree_xr`，并修改为使用本包内置 G1 assets。
- 将官方 `televuer` Python 源码复制到 `g1_xr_teleop/third_party/televuer`，并修改 XR adapter 为优先使用本包内置版本。
- 删除已不再使用的 `xr_teleoperate_path` 参数，避免后续误以为运行时仍依赖外部 `xr_teleoperate` 仓库。
- 新增英文 `README.md`，说明包的作用、边界、构建和启动方法。
- 新增中文 `LOG_zh-CN.md`，记录本次脚手架修改。
- 更新 `g1_control_ws/.gitignore`，忽略 Python `__pycache__/` 和 `*.pyc`。
- 已执行 `python3 -m compileall g1_control_ws/src/g1_xr_teleop/g1_xr_teleop`，Python 语法检查通过。
- 已执行 `colcon build --packages-up-to g1_xr_teleop`，构建通过；安装空间中已确认包含本包内置 G1 assets。
- 已执行一次 dry-run launch smoke test。默认 CycloneDDS 在当前沙箱无法枚举网卡；切换到 Fast DDS 后节点可启动，安全门因无 `/lowstate`、无 XR 数据、无 deadman 输入而保持关闭，符合预期。

## 2026-07-07

- 将 `g1_xr_teleop` 默认 `arm_model` 从 `G1ARM7` 改为 `G1ARM5`，匹配当前 23 自由度、无灵巧手、无腰部自由度的 G1 机器人。
- 在 `g1_xr_teleop.launch.py` 中新增 `arm_model` launch 参数，允许通过 `arm_model:=G1ARM5` 或 `arm_model:=G1ARM7` 覆盖配置。
- 更新英文 README 中的启动示例和 arm model 说明。

## 2026-07-09

- 为 vendored `TeleVuer` 增加 XR 事件诊断计数：`camera_events`、`controller_events`、`hand_events`。
- 将官方 `TeleVuer` 事件回调中静默吞掉的异常改为记录 `last_event_error` 并打印，便于排查 Quest 进入 VR 后仍然 `motion_data_ready=false` 的情况。
- 扩展 `g1_xr_teleop` 每秒 debug 日志，输出 raw trigger、摇杆值和 XR 事件计数；该修改只增加诊断信息，不改变控制命令发布逻辑。
- 增强 `TeleVuer.close()` 清理逻辑：正常 `terminate` 后等待，若 Vuer 子进程仍未退出则执行 `kill` 兜底，并注册 `atexit` 以减少 Ctrl+C 后 8012 端口残留。
- 增强 `XRInputAdapter.close()` 外层保护，清理失败时记录 warning 并释放 wrapper 引用，避免 ROS node 销毁流程被子进程清理异常阻塞。
- 修正 deadman trigger 判定逻辑，明确按 Unitree TeleVuer 的 `10.0=松开`、`0.0=完全按下` 约定归一化，避免右 trigger 完全按下时仍被判定为 `deadman not pressed`。
- 为 controller `thumbstickValue` 增加长度和类型容错，异常或追踪丢失时回退为 `[0.0, 0.0]`，避免 `ValueError: Can only assign sequence of same size` 阻断 controller event。
- 在 XR event 成功处理后清空上一条 `last_event_error`，避免旧错误在 debug 日志中持续残留。

## 2026-07-14

- 在 `XRFrame` 中增加 `head_pose`，并从 `TeleVuerWrapper.get_tele_data()` 传入 ROS teleop 调试流程。
- 扩展 `g1_xr_teleop` 低频 debug 日志：每秒输出 head、left wrist、right wrist 的位置和 RPY 摘要，并输出左右 trigger、squeeze、摇杆、摇杆按键、右 A 键和 XR event counters。
- 更新英文 README 的 Meta Quest 连接说明，补充 G1/PC/Quest 与路由器的有线和无线连接拓扑、PC 有线和 Wi-Fi IP 配置、Quest Browser 访问 URL、进入 `Virtual Reality` 的流程，以及 8012 端口残留检查方法。
- 新增 XR 数据卡住诊断状态 `xr_health`，根据 camera/controller/hand event counter 的增量和距上次增长的时间判断 `stuck`、`camera_stale`、`controller_stale` 等状态；该诊断只用于日志提示，不改变当前 arm/loco 数据处理逻辑。
- 新增参数 `xr_stuck_timeout_sec`，默认 `1.0` 秒；当 XR 数据疑似卡住或恢复时分别打印 warning/info，提示操作者调整或重新佩戴头显。
- 在 dry-run/debug 日志中新增 `loco_cmd=[vx=...,vy=...,omega=...]` 摘要，便于验证 Quest 摇杆到 locomotion 命令的方向和比例。
- 当前 locomotion 映射保持与 Unitree 官方 `xr_teleoperate` 一致：`vx=-left_stick_y`、`vy=-left_stick_x`、`omega=-right_stick_x`，再分别乘以配置中的最大速度限制。
