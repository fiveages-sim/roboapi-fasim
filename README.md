# roboapi-fasim

把 FaSim / Isaac Sim 上的 ROS 2 运控接到 [RoboAPI](https://developers.pokebot.com/zh/docs/roboapi/latest/add-hardware) 的机械臂、平行夹爪和腕部相机接口。生成方式与官方脚手架相同：

```bash
uv pip install 'roboapi==0.4.5'
roboapi new-embodiment fasim
```

本包在生成结果上实现了驱动，通过 `ros2_robot_interface` 与 `ocs2_arm_controller` / `gripper_controller` 对话。RoboAPI 仍负责标定、任务坐标转换、30 Hz 目标调度和本地控制台。

控制点是 **tool0**（法兰中心、Z 向外），位姿单位为米，四元数为 `wxyz`。请确认运控发布的 `/left_current_pose` 是 tool0，而不是 ROS-Industrial `flange`。

## 安装

先在同一 Python 环境中安装 RoboAPI 0.4.5、系统里的 ROS 2 Jazzy（`rclpy` / `cv_bridge` 来自 `/opt/ros/jazzy`，不要用 pip 装），以及 [ros2_robot_interface](https://github.com/fiveages-sim/ros2_robot_interface)：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
# 若使用 lerobot_ros2 的 uv 环境：
source /path/to/lerobot_ros2/.venv/bin/activate

git clone https://github.com/fiveages-sim/ros2_robot_interface.git
git clone https://github.com/fiveages-sim/roboapi-fasim.git
uv pip install -e ./ros2_robot_interface --no-deps
uv pip install -e ./roboapi-fasim
```

如果已经作为 `lerobot_ros2` 的子模块检出，把上面的路径换成 `lerobot_ros2/submodules/` 下的对应目录即可。

`--no-deps` 是因为 `rclpy` 和各 `*-msgs` 由系统 ROS 提供，不在 PyPI 上。安装后重新启动 RoboAPI，向导才能发现 `fasim` 组件。

UR5e 仿真的配置和标定放在 FaSim 资产目录，不要用 `~/.config/roboapi/`：

```bash
export ROS_DOMAIN_ID=1
source /path/to/lerobot_ros2/.venv/bin/activate
roboapi init -c ~/FaSim-Isaac/robots/manipulators/UniversalRobots/UR5e/roboapi/config.yaml
roboapi http -c ~/FaSim-Isaac/robots/manipulators/UniversalRobots/UR5e/roboapi/config.yaml
```

该目录已有面向 `env/fiveages_acepro2.usda` 的配置（相机话题 `/acepro2/rgb`）。向导里选择三套 `fasim` 组件。若仿真已在跑且 `ROS_DOMAIN_ID` 一致，`roboapi init` 会用 `ros2_robot_interface` 扫描话题，把位姿 / 关节 / 夹爪 / Image 相机话题填成默认值（仍可手改）。探测失败时回退到下面的内置默认（UR5e + 2F-85 + `ocs2_arm_controller`）：

| 组件 | 默认 |
| --- | --- |
| 机械臂位姿 / 目标 | `/left_current_pose`、`/left_target`、`/left_current_target` |
| 关节状态 | `/joint_states`，关节名为 UR5e 六轴 |
| 夹爪 | `gripper_joint`，命令话题 `/gripper_joint/position_command`，控制器 `gripper_controller` |
| 相机 | `/wrist_camera/rgb`，1280×720 RGB |

RoboAPI 开度 0 为闭合、1 为张开。官方 2F-85 的 `gripper_joint` 是 0 张开、0.8 闭合，驱动会换算。完整开合时间默认 1 秒。

腕部相机需要有节点向配置的话题发布图像。UR5e 的 `env/fiveages_acepro2.usda` 发布 `/acepro2/rgb`，`env/fiveages_action6.usda` 发布 `/action6/rgb`，请在配置里填写对应话题。图像保持原始朝向和鱼眼畸变，不做裁剪、补边或镜像。

## 说明

- 机械臂、夹爪共享同一个 `ROS2RobotInterface` 会话；只读反馈只订阅状态，不切换 FSM。进入控制时会切换到 OCS2。
- `servo_pose` 的可选 `twist` 和 `position_only` 暂未使用：目标由运控跟踪并在两次推理之间保持。
- 本仓库可单独使用，也可以作为 `lerobot_ros2` 的 git submodule 放在 `submodules/roboapi-fasim`。
