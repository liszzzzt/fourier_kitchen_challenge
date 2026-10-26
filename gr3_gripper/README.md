# Robotiq 2F-85 夹爪 ROS2 模块

将现有 `gripper_controller.py` 的 MuJoCo 夹爪控制封装为 ROS2 接口。
范围是夹爪开合、限速和状态反馈；调用方可以是任何组员的 ROS2 模块。
保留原来的 `closure=0` 张开、`closure=1` 闭合、控制值 `255*closure`、
默认每仿真秒变化 `0.5` 的逻辑。**本模块是仿真控制，不是实物驱动。**

## 架构

Ubuntu 推理/任务进程通过 ROS2 domain 6 发请求；Docker 中拥有当前
`MjModel/MjData` 的仿真进程接收请求，并在原物理循环中更新夹爪 actuator。
夹爪不经过 Aurora SDK。原 Aurora domain 123 和 31 个 Core 控制通道保持独立。

`GripperROSBridge.step(data)` 必须在实际运行的仿真进程内调用。
另开一个普通 ROS2 节点无法获得已有进程的 `MjData`。本提交不修改主仓库的
站立、行走、手臂模块，也没有接入未知的 viewer 入口文件。

## 模型契约

现有集成模型需要包含：

| 项目 | 名称 / 值 |
|---|---|
| 夹爪 actuator | `rq_right_fingers_actuator` |
| 控制范围 | `[0, 255]`，`ctrllimited` 必须开启 |
| 力限制 | `forcelimited` 必须开启；沿用模型的 `forcerange` |
| 反馈关节 | `rq_right_left_driver_joint`、`rq_right_right_driver_joint` |
| 31 通道完整机器人 | 夹爪 actuator 必须位于前 31 个 actuator 之后 |

代码不修改模型的惯量、摩擦、约束、增益或限力。原控制器里的 `CoreLayout`、
`core_layout()` 和通道顺序检查保留，供原 viewer 沿用。

模型来源：[Kinova 收录的 Robotiq 2F-85 描述](https://github.com/Kinovarobotics/ros_kortex/tree/kinetic-devel/kortex_description/grippers/robotiq_2f_85)。
该目录的 [README](https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/robotiq_2f_85/README.md)
说明它来自 Robotiq 上游提交 `4e3196c1c6a83f54acfa3dd8cf02b575ebba3e53`；
原资源的 [许可证](https://github.com/Kinovarobotics/ros_kortex/blob/kinetic-devel/kortex_description/grippers/robotiq_2f_85/LICENSE) 随原资源适用。

原 Xacro 的主动关节是 `${prefix}finger_joint`，范围 0–0.8 rad，其他关节有
mimic 关系；它与这里已转换的 MuJoCo actuator 命名、控制单位不同。
不能把原 Xacro 的角度直接作为 `data.ctrl`，也不能把 `closure` 当作米或牛顿。
本提交不重新分发第三方网格/模型。使用已经集成 Robotiq 的现有 MuJoCo 模型；
仓库里的旧 dummy-hand 模型没有上述 actuator，检查会拒绝执行。

`models/smoke_fixture.xml` 是自建的两个铰链测试模型，用于验证通信和控制过程。
它不代表 Robotiq 几何、抓碗能力或完整机器人动力学。

## 环境

| 位置 | 基线 | 本模块所需 |
|---|---|---|
| 机器人 Docker | `fourier_aurora_sdk_gr3:v1.3.0`；Python 3.10.12；NumPy 1.21.5；MuJoCo 2.3.6 | ROS2 Humble、`rclpy`、`std_msgs`；现有 NumPy/MuJoCo |
| Ubuntu 22.04.5 推理端 | Python 3.10.22；NumPy 1.26.4 | 客户端只需 ROS2 Humble、`rclpy`、`std_msgs`、Python 标准库 |

推理端现有 SciPy 1.11.4、OpenCV 4.10.0、PyTorch 2.5.1+cpu、
Torchvision 0.20.1+cpu、PyYAML 6.0.3、Pillow 10.4.0 无需为夹爪改动。
安装包不会自动安装/升级 NumPy、MuJoCo 或 Aurora。

两端设置 `ROS_DOMAIN_ID=6`，source `/opt/ros/humble/setup.bash`。
宿主与 Docker 的 DDS 发现和传输必须可达；同机 Linux 可使用已配置的 host
network，Fast DDS 使用共享内存时还须正确配置 IPC/共享内存或改用 UDP。
不同机器须允许 DDS 流量并校准系统时钟。

Conda 的 Python 同为 3.10 不保证 `rclpy` 二进制兼容。
先在实际解释器中跑下方 preflight；若导入失败，可用宿主系统 ROS Python 进程
运行客户端，由推理进程调用。不要用 `pip install rclpy` 替代 Humble 安装。
参见 [ROS2 官方 Python 环境说明](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Using-Python-Packages.rst)。

## 安装与自检

以下从仓库根目录运行。可先直接用源码运行，不依赖修改现有项目的打包配置：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=6
export PYTHONPATH="$PWD/gr3_gripper:${PYTHONPATH:-}"
# Docker 中：
python3 -m gr3_gripper.preflight --role robot --model /实际路径/main.xml
# 宿主机中，使用运行推理/客户端的解释器：
python -m gr3_gripper.preflight --role compute
```

preflight 是只读检查，不连接 Aurora，不执行物理步，不发送夹爪命令。
它检查相关 Python/库版本和 ROS 消息 type support；不会宣称跨容器通信通过。
宿主系统 Python 作为隔离客户端时，Python 版本与 3.10.22 的差异会如实报告。

需要 `ros2 run` 时，可把 `gr3_gripper` 放入 ROS2 workspace 的 `src` 目录，
在该 workspace 执行：

```bash
colcon build --packages-select gr3_gripper --symlink-install
source install/setup.bash
ros2 run gr3_gripper gripper_client --help
```

## 接到现有仿真循环

在原先创建 `GripperController` 的位置替换 import，并创建 bridge。
使用原来的 `model`、`data`，不要重新加载一份模型或创建第二套物理世界。

```python
import rclpy
from gr3_gripper.controller import GripperController, core_layout
from gr3_gripper.ros_bridge import GripperROSBridge

# model、data 已由现有 viewer 创建；若已有 rclpy context，不重复 init。
rclpy.init()
gripper = GripperController(model, command_file=None, closure_rate=0.5)
bridge = GripperROSBridge(gripper, namespace="/yolanda/right_gripper")
try:
    while viewer.is_running():
        # 原来的 Core 输出等逻辑继续执行，只写自己的通道。
        # 用这一行替换原 gripper.update(data)，放在 mj_step 之前：
        bridge.step(data)
        mujoco.mj_step(model, data)
        viewer.sync()
finally:
    bridge.destroy_node()
    rclpy.shutdown()
```

上面只示意插入位置；保留原有 Core 更新、viewer 锁、实时节拍、渲染和退出逻辑。
不要把这个节点再交给其他线程的 executor。`step()` 负责非阻塞处理 ROS 回调、
更新 actuator 并采集状态，都在拥有 MuJoCo 数据的线程里完成。
若暂停时需要继续处理请求超时，暂停分支也须调用 `step(data)`，但不执行物理步。
整个进程停住时无法回传状态，客户端会超时。

ROS2 接管时禁用旧 `command_file`，避免两个来源竞争同一夹爪。
仿真时间回退会保留原控制器的回到张开指令行为，并换新的 `server_id`；
重新加载模型时应销毁旧 bridge 并创建新的 controller/bridge。

## 组员调用

每台机器人/夹爪用独立 namespace。例：`/yolanda/right_gripper`、
`/member_b/right_gripper`。同一 namespace 只运行一个 bridge。

```bash
# 宿主机；先设置上面的 ROS 环境与 PYTHONPATH
python -m gr3_gripper.client --namespace /yolanda/right_gripper --closure 0
python -m gr3_gripper.client --namespace /yolanda/right_gripper --closure 0.5
python -m gr3_gripper.client --namespace /yolanda/right_gripper --closure 1
```

客户端先发现 bridge、取得当前会话，再只发送一次请求。默认等 10 秒。
退出码 0 表示**指令渐变完成**，1 表示服务端拒绝/错误/超时，2 表示通信结果不明。
通信超时不会自动重发。

| ROS2 topic | 类型 | 含义 |
|---|---|---|
| `<namespace>/command` | `std_msgs/msg/String` | JSON 开合请求 |
| `<namespace>/status` | `std_msgs/msg/String` | JSON 状态；运行中最高约 20 Hz |

请求字段全部必填，禁止额外字段：

```json
{"version":1,"id":"unique-command-id","server_id":"从 READY 状态读取",
 "backend":"simulation_mujoco","closure":0.5,"expires_at":1234567890.0}
```

`expires_at` 使用当前 Unix 时间加最多 30 秒，同时限定执行期限；示例时间需替换。
请求最多 4096 字节，`closure` 必须为有限 JSON 数字，且在 0–1 内。
同会话同 ID 同内容返回已记录状态；同 ID 不同内容被拒绝；执行中拒绝新命令。
已过期 ID 不会被再次执行。多个组员共享同一夹爪时，由任务层协调顺序。

状态依次为 `ACCEPTED` → `RAMPING` → `RAMP_COMPLETE`；另有 `READY`、
`REJECTED`、`TIMEOUT`、`RESET`、`ERROR`。`RAMP_COMPLETE` 仅表示
`data.ctrl` 的渐变指令到达目标，不表示手指到位、接触或夹持成功。
状态中的 `robot_task_success` 始终为 `null`。

观测包含目标/当前指令开合量、actuator 控制值、actuator 广义力、两个 driver
关节位置（rad）、仿真时间和旧文件接口错误。actuator 广义力不是指尖法向力。
超时保持当前指令开合量；异常会锁定 bridge，须检查仿真后重建它。

## 测试

```bash
PYTHONPATH=gr3_gripper python3 -m unittest discover -s gr3_gripper/test -v
# 有已有集成模型时，同时检查它的 actuator 隔离：
GR3_TEST_MODEL=/实际路径/main.xml PYTHONPATH=gr3_gripper \
  python3 -m unittest discover -s gr3_gripper/test -v
```

不接真实机器人也能测试 ROS2 + MuJoCo：Docker 运行合成测试模型，宿主运行客户端。
两个终端均先配置 ROS 环境和源码路径：

```bash
# Docker 终端
python3 -m gr3_gripper.demo --namespace /transport_test/right_gripper
# 宿主终端
python -m gr3_gripper.client --namespace /transport_test/right_gripper --closure 0.8
python -m gr3_gripper.client --namespace /transport_test/right_gripper --closure 0
```

自动 ROS2 测试使用两个真实进程；没有 `rclpy` 时明确 skip，不伪造中间件。
实际已运行的检查和限制见 [VALIDATION.md](VALIDATION.md)。
