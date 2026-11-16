# roboapi-fasim

把 FaSim / Isaac Sim 上的 ROS 2 运控接到 [RoboAPI](https://developers.pokebot.com/zh/docs/roboapi/latest/add-hardware) 的机械臂、平行夹爪和腕部相机接口。

本包通过 [ros2_robot_interface](https://github.com/fiveages-sim/ros2_robot_interface) 与 `ocs2_arm_controller` / `ocs2_wbc_controller` / `gripper_controller` 对话。RoboAPI 负责标定、任务坐标、30 Hz 目标调度和本地控制台；本包只做 FaSim 侧驱动。

控制点是 **tool0**（法兰中心、Z 向外），位姿单位为米，四元数为 `wxyz`。请确认运控发布的位姿是 tool0，而不是 ROS-Industrial `flange`。

## 安装

需要同一 Python 环境中有：

- RoboAPI `0.4.5`
- 系统 ROS 2 Jazzy（`rclpy` / `cv_bridge` / 各 `*-msgs`，**不要**用 pip 装）
- `ros2_robot_interface`

### 作为 lerobot_ros2 子模块

```bash
cd /path/to/lerobot_ros2
./init.sh submodules submodules/roboapi-fasim   # 或菜单勾选后初始化
# 若环境已建好：
./init.sh install-checked                       # 会 editable 安装已检出的 roboapi-fasim
```

也可手动：

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/lerobot_ros2/.venv/bin/activate
uv pip install -e submodules/ros2_robot_interface --no-deps
uv pip install -e submodules/roboapi-fasim
```

### 单独克隆

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

git clone https://github.com/fiveages-sim/ros2_robot_interface.git
git clone https://github.com/fiveages-sim/roboapi-fasim.git
uv pip install -e ./ros2_robot_interface --no-deps
uv pip install -e ./roboapi-fasim
```

`--no-deps` 是因为 ROS 消息包不在 PyPI。安装后需重新启动 RoboAPI，向导才能发现 `fasim` 组件。

## 配置（roboapi init）

配置建议放在 FaSim 资产目录旁，不要用 `~/.config/roboapi/`。以 UR5e 为例：

```bash
export ROS_DOMAIN_ID=1   # 与 Isaac / 运控一致
source /path/to/lerobot_ros2/.venv/bin/activate
roboapi init -c ~/FaSim-Isaac/robots/manipulators/UniversalRobots/UR5e/roboapi/config.yaml
roboapi http -c ~/FaSim-Isaac/robots/manipulators/UniversalRobots/UR5e/roboapi/config.yaml
```

向导里为臂 / 夹爪 / 相机都选 **FaSim ROS 2** 组件。

### 话题探测

若仿真或实机驱动已在跑且 `ROS_DOMAIN_ID` 一致，`configure` 会用 `ros2_robot_interface` 扫描 ROS 图，预填：

| 项 | 探测内容 |
| --- | --- |
| 机械臂 | `/left_*` 或 `/right_*` 位姿与目标、`/joint_states`、夹爪命令与控制器 |
| 关节名 | 从 `joint_states` 抽样；双臂时按 `arm_side` 过滤 |
| 基坐标系 | 从位姿消息 `header.frame_id` 读取（若有） |
| 相机 | 所有 `sensor_msgs/Image` 话题，优先匹配 `wrist` / `acepro` / `action6` / `rgb` |

探测失败时回退到内置默认（见下表）。向导仍可手改每一项。

### 单臂 / 双臂

RoboAPI 一次只控制 **一条臂**。配置字段 `arm.arm_side` 为 `left` 或 `right`。

- 图上同时有左右 Cartesian 话题时，init 会先问控制哪条臂。
- 选中侧的位姿 / 目标话题写入配置，并映射到接口的单臂主通道。
- 夹爪走真实的 left/right handler（不会因为自动探测把右夹爪命令打到左边）。

| 组件 | 左臂默认 | 右臂默认 |
| --- | --- | --- |
| `arm_side` | `left` | `right` |
| 位姿 / 目标 | `/left_current_pose`、`/left_target`、`/left_current_target` | `/right_*` |
| 关节状态 | `/joint_states` | 同左（共享） |
| 夹爪关节 / 命令 | `gripper_joint` 或 `left_gripper_joint`，`/…/position_command` | `right_gripper_joint`，`/right_gripper_joint/position_command` |
| 夹爪控制器 | `gripper_controller` / `left_gripper_controller` | `right_gripper_controller` |
| 相机 | `/wrist_camera/rgb`；UR5e 场景常见 `/acepro2/rgb`、`/action6/rgb` | 按实际 Image 话题选择 |

UR5e + 2F-85 单臂场景通常选 `left`，夹爪开合：RoboAPI 开度 0=闭合、1=张开；硬件 `gripper_joint` 常为 0 张开、0.8 闭合，驱动会换算。

已有 `config.yaml` 时可直接改 `arm.arm_side` 与对应话题，不必重跑 init；改侧后需重新标定连接身份。

## 运行行为

### 进入控制

- 机械臂、夹爪共享同一个 `ROS2RobotInterface` 会话。
- 只读反馈只订阅状态，不切 FSM。
- 首次下发运动时会 `prepare_control`：切到 OCS2（`arm_pose`）。

### WBC 双臂

若检测到 WBC（图上有 `ocs2_wbc_controller`），进入控制时会额外发 mode：

1. `ARMS_INDEPENDENT`
2. 启用当前侧：`LEFT_ARM_ENABLE` / `RIGHT_ARM_ENABLE`
3. 关闭另一侧：`RIGHT_ARM_DISABLE` / `LEFT_ARM_DISABLE`

避免另一侧仍被 WBC 跟踪、与单臂 Cartesian 目标打架。非 WBC 或没有 `/mode_command` 时跳过。

### 推理观测

送给策略的是末端位姿、夹爪开度、相机图像。  
`/joint_states` 用于读臂关节日志状态和夹爪开度反馈，**不作为策略观测输入**。

### 其它

- `servo_pose` 的可选 `twist` / `position_only` 暂未使用；目标由运控跟踪并在两次推理之间保持。
- 相机需持续发布约 1280×720 RGB；保持原始朝向与畸变，不要裁剪、补边或镜像。

## 故障排查

| 现象 | 排查 |
| --- | --- |
| init 探测不到话题 | 确认已 `source` ROS、仿真/驱动在跑、`ROS_DOMAIN_ID` 与 RoboAPI 一致 |
| 配了右夹爪却动左边 | 升级到含侧别修复的本包并重启 RoboAPI；确认 `gripper.command_topic` 含 `right`，且 `arm_side=right` |
| WBC 下单臂发力异常 | 确认进入控制后另一侧已被 `*_ARM_DISABLE`；看 `/ocs2_wbc_controller/current_state` |
| 找不到 fasim 选项 | `uv pip install -e .` 后重启 `roboapi` 进程 |
| 相机超时 | 核对 Image 话题名（如 `/acepro2/rgb`）与帧率 |

## 相关路径

- 本仓库也可单独使用，或作为 `lerobot_ros2` 的 submodule：`submodules/roboapi-fasim`
- UR5e 本地配置说明：`FaSim-Isaac/robots/manipulators/UniversalRobots/UR5e/roboapi/README.md`
