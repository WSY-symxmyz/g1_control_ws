# G1 Control Workspace 修改日志

本文件用于记录 `g1_control_ws` 中控制接口、消息、参数和安全逻辑的主要变更。之后每次修改接口或控制逻辑时，都应在这里追加记录，方便长期维护和实机测试追踪。

## 2026-07-17

### 修复 locomotion 高频 API 请求的 5 秒阻塞

- 根据实机 `/api/sport/request` 和 `/api/sport/response` 日志，确认旧实现
  每次调用 Unitree API 都临时创建 response subscription。高频发送 `7105`
  时偶发错过 response，随后同步等待固定 `5 s`，导致机器人只执行一小段
  `0.15 s` 速度后长时间停顿。
- 将 `BaseApiClient` 改为持久订阅 `/api/sport/response`：
  - request publisher 和 response subscriber 都在 client 构造时创建。
  - 使用 Unitree `identity.id` 在 pending request 表中匹配响应。
  - 增加异步调用和请求超时清理，不再以 20 Hz 创建/销毁 DDS endpoint。
  - 超时日志打印 `api_id` 和 `identity.id`，方便定位 `7001`、`7101`、
    `7105` 的具体故障。
- `7105 SetVelocity` 改为异步响应处理，不再阻塞 ROS command subscription
  或 keepalive timer；连续多次响应失败后关闭 locomotion gate 并发送零速度。

### 调整 FSM 500 初始化与停止语义

- 节点启动后异步执行一次 `7001 GetFsmId`：
  - 已处于 FSM 500 时直接进入 ready，不发送 `7101`。
  - 不为 500 时调用官方 `Start` 对应的 `7101 SetFsmId(500)`，然后再次
    查询并验证结果。
  - 初始化完成前拒绝速度命令，避免执行启动期间缓存的过期非零速度。
- 速度命令不再隐式设置 FSM；正常运行阶段只发送 `7105`。
- 分离“FSM 已确认”和“当前运动命令 active”状态。`StopMove` 只发送零速度、
  清除当前运动命令，不再把 FSM 500 标记为失效，因此下一次移动不会重复
  发送 `7101`。
- 参考 Unitree 官方 `Move(..., continuous=false)`，新增默认 `1.0 s` 的速度
  有效期，同时保留 `0.30 s` ROS 输入 watchdog 和显式零速度停止。
- 新增参数：
  - `loco_fsm_startup_delay_sec`
  - `loco_api_response_timeout_sec`
  - `loco_velocity_duration_sec`
  - `loco_api_failure_limit`
- 更新 `README.md` 中 FSM 初始化、状态检查、StopMove 和实机调试说明。

## 2026-06-04

### 新增键盘 delta 遥操作 demo

- 新增 `g1_arm_keyboard_delta_teleop` 节点，源码位于
  `src/g1_control_interface/src/apps/g1_arm_keyboard_delta_teleop.cpp`。
- 该 demo 不直接发布 `/arm_sdk`，而是向 `/g1/command/arm_joints` 发布
  `g1_control_msgs/msg/ArmJointCommand`，并设置 `is_delta=true`，复用现有
  arm interface 中用于 RL/遥操作的 delta 累加、关节限位和速度限幅逻辑。
- 默认按键映射：
  - `w` / `s` 控制 `left_shoulder_pitch` 增减。
  - `a` / `d` 控制 `left_shoulder_roll` 增减。
  - 方向键上/下控制 `right_shoulder_pitch` 增减。
  - 方向键左/右控制 `right_shoulder_roll` 增减。
  - `q` 退出 demo。
- 默认每次 key event 发送 `0.01 rad` 的 delta，可通过 ROS 参数
  `delta_step_rad` 调整；按住按键时依赖终端 key-repeat 持续发送小步
  delta 命令。
- 更新 `README.md`，补充键盘 demo 的运行命令、按键映射和
  `delta_step_rad` 参数示例。

### 解耦 arm 实时控制与 loco/FSM 查询

- 发现 `/arm_sdk` 在 unified interface 中只有约 1~3 Hz，且最大间隔约
  `5 s`；官方 `g1_arm_sdk_dds_example` 在相同环境下 `/arm_sdk` 稳定约
  49 Hz。对照后判断问题来自 interface 内部阻塞，而不是 `/arm_sdk`
  topic 或官方 QoS 本身。
- 定位到旧状态发布路径会周期性查询 loco FSM：
  `PublishStatus()` -> `RobotModeManager::snapshot()` ->
  `LocoController::get_fsm()` -> `BaseApiClient::Call()`，其中 API 等待
  response 的超时时间为 `5 s`。当 FSM 查询未及时返回时，会阻塞 ROS
  callback，并影响 arm 的 ROS timer 发布。
- 将 arm `/arm_sdk` 发布从 ROS timer 改为独立控制线程，保持与官方示例
  更接近的结构：
  - 独立线程按 `arm_control_period_sec` 调用控制发布逻辑。
  - ROS executor 中的 service、status、loco API 调用不再阻塞 arm 的
    50 Hz 位置控制循环。
  - `/arm_sdk` publish 放在 mutex 外执行，避免 DDS publish 偶发阻塞时
    长时间占用 arm 状态锁。
- `ControllerStatus` 不再自动查询 FSM，只发布缓存值：
  - 启动时 `fsm_id=-1`、`fsm_mode=-1`，表示尚未查询。
  - `/g1/loco/set_mode` 显式设置 locomotion mode 后更新缓存。
  - `/g1/loco/stop` 和 disable 只更新缓存 message，不主动查询 FSM。
- 不新增手动 FSM 查询 service；如需实时确认 FSM，使用 Unitree 官方查询
  API/工具手动查询。原则是：与实时位置控制无关的状态查询不得阻塞 arm
  或其他位置控制路径。
- 更新 `README.md`，说明 arm 独立 50 Hz 控制线程、FSM 缓存语义，以及
  启动时不阻塞查询 FSM 的设计。

### 调整 set_pose 队列完成判断

- 侧平举测试中，`commanded_positions` 已经等于 `target_positions`，但
  `left_shoulder_roll` 和 `right_shoulder_roll` 的实测误差分别约为
  `0.058 rad` 和 `0.086 rad`，超过原先 `0.04 rad` measured 到达阈值。
  因此 controller 一直认为当前 service target 未完成，后续回原位
  set_pose 命令进入 FIFO 队列后无法开始执行。
- 对照官方 `g1_arm_sdk_dds_example.cpp`：官方示例不使用 `/lowstate`
  判断动作是否实际到达目标，而是按内部命令位置和固定时长推进动作。
- 将 `/g1/arm/set_pose` 队列切换条件改为：内部 `commanded_positions`
  已按限速走到 `target_positions` 时，即认为当前 service target 的命令
  轨迹完成，可以执行下一个队列目标。
- 删除不再使用的 `arm_service_reached_tolerance_rad` 参数。实际关节角误差
  仍通过 `/g1/debug/arm_command` 暴露给用户观察，但不再阻塞 service 队列。

### 从零重建上肢 arm interface

- 基于官方示例 `example/src/src/g1/high_level/g1_arm_sdk_dds_example.cpp`
  的控制思路重写 `ArmSdkController`：
  - 启动后先等待 `/lowstate`。
  - 第一次接管时从实测关节角初始化 `target` 和 `commanded`。
  - 接管后按 `arm_control_period_sec` 固定周期发布 `/arm_sdk`，默认
    `0.02 s`，即 50 Hz。
  - 每次发布完整 arm/waist joints 的 `unitree_hg/msg/LowCmd`。
  - `q` 使用限速后的绝对关节角，`dq=0`，`tau=0`，`kp/kd` 使用默认值
    或命令中给出的稀疏覆盖值。
  - `NOT_USED_JOINT.q=1.0` 表示 arm_sdk 接管；调用 `/g1/arm/stop`
    时按 `arm_release_duration_sec` 逐步降到 `0.0` 并停止发布。
- 重新定义 `/g1/command/arm_joints` 为 RL/遥操作实时 servo 入口：
  - subscription 使用 keep-last-1 + best-effort QoS，避免高频命令排队。
  - `is_delta=true` 时，单条 delta 由
    `arm_servo_max_delta_per_command_rad` 限幅后累加到目标。
  - `is_delta=false` 时，绝对目标不拒绝远目标，而是相对最新实测姿态
    或最近发布姿态按 `arm_servo_max_abs_step_per_command_rad` 裁剪成安全小步。
  - 新命令到达后直接更新当前 servo target，不进入队列。
- 重新定义 `/g1/arm/set_pose` 为单次位置控制入口：
  - service request 被转换成完整 absolute target。
  - 未指定 joint 使用接收请求时的 `/lowstate` 实测角作为保持目标。
  - 如果已有 service target 正在执行，新 request 进入 FIFO 队列。
  - 当前目标的内部 `commanded_positions` 按限速走到 `target_positions`
    后，执行下一个队列目标。
- 清理旧 trajectory/home-return 相关实现和配置：
  - 删除 `arm_trajectory_generator.hpp/.cpp`。
  - 从 `CMakeLists.txt` 移除旧 trajectory generator 编译项。
  - 删除 `arm_home_positions` 配置。
  - 废弃旧参数：`arm_command_timeout_sec`、
    `arm_servo_command_timeout_sec`、`arm_stop_return_home_duration_sec`、
    `arm_stop_return_home_timeout_sec`、`arm_home_reached_tolerance_rad`、
    `arm_home_release_settle_sec`、`arm_hold_on_timeout`。
- 新增/保留的上肢主要参数：
  - `arm_control_period_sec`
  - `arm_servo_max_delta_per_command_rad`
  - `arm_servo_max_abs_step_per_command_rad`
  - `arm_max_position_step_rad`
  - `arm_lowstate_fresh_timeout_sec`
  - `arm_release_duration_sec`
  - `arm_default_kp`
  - `arm_default_kd`
  - `arm_waist_kp_scale`
  - `arm_waist_kd_scale`
- 更新 `README.md` 中 arm 控制、参数和 stop 行为说明，使其与当前实现一致。
- 调整 debug 发布顺序：在每个控制周期内先发布
  `/g1/debug/arm_command`，再发布 `/arm_sdk`。这样当 `/arm_sdk` DDS
  发布链路出现阻塞或 backpressure 时，debug topic 更适合判断 controller
  timer 本身是否稳定。

### 增加上肢命令调试 topic 与实时 QoS

- 新增 `g1_control_msgs/msg/ArmCommandDebug.msg`。
- 新增 `/g1/debug/arm_command` topic，用于发布：
  - `measured_positions`：来自 `/lowstate` 的实测关节角。
  - `target_positions`：interface 内部最新目标角。
  - `commanded_positions`：经过限速后实际发送到 `/arm_sdk` 的绝对位置命令。
  - `control_active`、`trajectory_mode`、`release_after_motion`：控制器状态标志。
- 将 `/g1/command/arm_joints` 订阅 QoS 改为 keep-last-1 + best-effort，避免遥操作/RL 高频命令排队。
- `/arm_sdk` publisher 保持官方示例风格的 reliable depth-10 QoS。此前尝试 keep-last-1 + reliable 可能在部分 DDS 网络中引入 backpressure，导致 controller timer/debug topic 出现低频或长间隔。
- 将 `/g1/debug/arm_command` 改为 best-effort QoS，并把 debug 发布移动到 `/arm_sdk` 发布之前。这样 debug topic 更适合判断 interface 控制循环本身是否稳定，避免被 `/arm_sdk` DDS 发布阻塞误导。
- 将 `/g1/command/arm_joints` 订阅 QoS 从 keep-last-1 + best-effort 回退为默认 depth-10，便于单独排查 QoS 改动是否导致低频或间歇响应。debug topic 保留。
- README 增加 debugging 方法，用于区分 interface 输出不连续和机器人实际响应不连续。

### 上肢控制接口重构：区分 streaming/servo 与 trajectory/debug

- 将 `/g1/command/arm_joints` 明确定位为高频 streaming/servo 控制入口，用于键盘、手柄、遥操作、RL policy 等实时小步命令。
- 将 `/g1/arm/set_pose` 保留为 service-based trajectory/debug 入口，用于单次或多次较大幅度绝对位置目标。
- 新增 `ArmJointCommand` 字段：
  - `is_delta`：为 `true` 时表示使用相对增量命令。
  - `deltas`：相对目标增量，最终会被 interface 累加为 `/arm_sdk` 所需的绝对位置命令。
- `/g1/command/arm_joints` 采用 latest-command-wins 策略：如果尚未到达旧目标就收到新命令，不排队、不等待，直接更新内部目标。
- 控制器内部继续以固定周期向 `/arm_sdk` 发布 `unitree_hg/msg/LowCmd`，并对实际发布的绝对位置命令做每周期限速，保证命令连续。
- streaming 模式下收到新命令时，不再重置已经发布的 commanded position，避免高频控制中的二次动作感和延迟。
- service/trajectory 模式下，大目标会从当前实测姿态开始，按统一速度限制安全移动。

### 新增和调整的安全参数

- 新增 `arm_servo_command_timeout_sec`：streaming 命令超时时间。超过该时间未收到新命令时，控制器保持当前实测姿态，避免继续执行过期目标。
- 新增 `arm_servo_max_delta_per_command_rad`：单条相对增量命令的最大允许幅度，防止键盘、手柄或 RL 客户端误发过大的相对步长。
- `arm_max_position_step_rad` 继续作为每个控制周期的全局最大发布步长，默认 `0.01 rad`，对应官方 `g1_arm_sdk_dds_example` 中 `0.5 rad/s * 0.02 s`。
- 每关节 `joint_limit_max_velocities` 也参与限速，实际每周期步长取 `arm_max_position_step_rad` 和 `joint_limit_max_velocity * arm_control_period_sec` 中较小者。

### 文档更新

- 更新 `README.md`，说明：
  - streaming topic 和 service trajectory 的用途差异。
  - 绝对位置 streaming 命令示例。
  - 相对增量 streaming 命令示例。
  - 新增参数的含义。

### 后续注意事项

- RL/遥操作应优先使用 `/g1/command/arm_joints`，不要用 service 高频控制。
- `/arm_sdk` 仍然只接收绝对位置命令，所有相对增量命令都必须先在 interface 内部转换为绝对目标。
- 后续如果增加 emergency stop、command arbitration、Python client 或仿真适配，也应在本文件中记录。
