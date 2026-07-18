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

## 2026-07-19

- 新增 `native_optimization_ik.py`，使用普通 Pinocchio 数值运动学和 SciPy 有边界 trust-region least-squares 实现双臂优化 IK，不再要求 `pinocchio.casadi`。
- G1ARM5 保留 Unitree 官方 G1 23 自由度 IK 的模型缩减、左右末端 `0.20 m` 偏置、位置/旋转/正则/平滑目标权重 `50.0/0.5/0.02/0.1`、关节限位、warm start 和加权移动滤波。
- 同时支持 G1ARM7；使用官方对应的 29 自由度 URDF、锁定腿/腰/手指、左右末端 `0.05 m` 偏置和旋转权重 `1.0`。
- 新增目标位姿有限值与旋转矩阵检查、SO(3) 投影、单周期关节变化限制、求解质量阈值和失败回退；不合格解保持当前实测关节角。
- `ArmIKAdapter` 新增 `native_optimization` 和 `unitree_casadi` 后端选择；默认使用 `native_optimization`，官方 vendored CasADi 实现继续作为兼容参考保留。
- 新增 IK ROS 参数，包括最大函数评估次数、收敛容差、输出步长、误差阈值和滤波权重；debug 日志新增求解耗时、误差、接受状态、评估次数和实际 `arm_target`。
- 新增 `ik_offline_check` 离线检查入口，以及关节顺序、可达目标收敛、输出限速和非法目标回退自动测试。
- 修复 venv 中 NumPy `2.2.6` 与系统 SciPy `1.8.0` 不兼容的问题，在 venv 安装 SciPy `1.15.3`。
- 确认普通 `colcon build` 生成的 ROS console script 固定使用 `/usr/bin/python3`，无法读取仅安装在 venv 中的 Pinocchio；改用激活 venv 后执行 `python3 /usr/bin/colcon build`，安装入口的 shebang 正确指向 venv Python。
- G1ARM5 200 帧连续轨迹离线基准：`200/200` 接受，平均约 `1.49 ms`，P95 约 `1.56 ms`，最大约 `3.05 ms`，最大位置误差约 `1.9 mm`，低于 30 Hz 的 `33.3 ms` 周期。
- G1ARM5 200 次大幅随机/部分不可达目标压力测试：P95 约 `8.1 ms`，最大约 `13.8 ms`；超过误差阈值的解被拒绝并回退。
- G1ARM7 100 帧连续轨迹离线检查同样 `100/100` 接受，P95 约 `1.83 ms`，确认通用优化框架可正确构建 14 关节双臂模型。
- 新增 `pytest.ini`，将测试收集限制在 `test/`，避免 ROS `launch_testing` 插件在 IK 测试期间导入 vendored XR 后端；包级测试最终为 `4 tests, 0 errors, 0 failures`。
- launch 新增 `enable_arm` 和 `enable_loco` 参数，允许 IK 调试阶段明确关闭 locomotion，避免首次真实双臂测试与行走命令混合。
- 修复 venv 中 Vuer 0.0.60 与 params-proto 3.3.0 的运行时不兼容：固定 `params-proto==2.13.2` 后，`from vuer import Vuer` 验证通过；README 已记录该版本约束。
- 将原标定占位实现替换为双模式手臂追踪状态机。`relative_clutch` 在 deadman 每次按下时捕获 XR 双腕和机器人双腕锚点，按相对平移与旋转增量生成目标；松开、XR 未就绪或活动事件流超时后清除锚点，并在恢复后重新捕获。
- 新增 `unitree_absolute` 模式，直接将 TeleVuer 已转换到机器人腰部坐标约定的双腕 SE(3) 位姿输入 IK，保持 Unitree 官方 teleop 的绝对映射逻辑；本 ROS 接口额外保留 deadman 安全门。
- 新增 `arm_tracking_mode` launch/YAML 参数，默认设为更适合初期真机测试的 `relative_clutch`；新增 `arm_position_scale`、`arm_rotation_scale` 相对模式缩放参数。
- `ArmIKAdapter` 新增正运动学与求解状态重置接口，供相对离合捕获当前机器人末端锚点并清空旧滤波历史。
- 将 IK 单关节独立裁剪改为 Unitree 官方形式的整条关节增量向量等比例缩放，保留候选解的运动方向。
- 扩展 IK 诊断，分别记录优化器原始候选解误差、实际限速/滤波输出误差，以及限速前后的最大关节变化，便于区分求解质量和输出追踪过程。
- 对数值变化的 `arm target jump` 安全原因进行类别归一，避免同类跳变数值每帧变化时重复刷屏；首次出现和状态变化仍会记录。
- 新增标定状态机与统一向量限速测试，源码环境测试结果为 `10 passed`。
- 相对离合捕获锚点的首个控制周期直接保持当前实测关节角，不运行带正则项的 IK，从控制输出层保证捕获瞬间不产生姿态跳变。
