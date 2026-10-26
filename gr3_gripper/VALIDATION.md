# 验证记录

本记录区分控制逻辑、ROS2 通信和实际机器人验证，不把指令完成视为抓取成功。

## 本地已执行

2026-10-26，在 Ubuntu 24.04.3 的独立环境中安装并运行：

| 项目 | 实际值 / 结果 |
|---|---|
| Python | 3.10.12 |
| NumPy | 1.21.5 |
| MuJoCo | 2.3.6 |
| unittest | 19 项：18 通过，1 跳过（缺少真实 rclpy） |
| 用户现有 `fixed_arm.mjb` | MuJoCo 2.3.6 加载成功；夹爪控制和其他 actuator 写入隔离通过 |
| 模型信息 | `nu=8`，夹爪 actuator ID 7，ctrlrange `[0,255]`，forcerange `[-5,5]` |

测试覆盖开合渐变、真实合成模型物理步、仿真重置、非有限输入、模型契约、
夹爪与 Core actuator 写入隔离、旧文件请求处理、重复请求、会话/期限检查、
并发请求拒绝、超时保持和反馈错误。现有 `fixed_arm.mjb` 只做加载及控制写入
检查；没有据此声称完整机器人动力学或抓取能力已验证。

运行方式（路径替换为本机实际值）：

```bash
GR3_TEST_MODEL=/path/to/fixed_arm.mjb PYTHONPATH=gr3_gripper \
  /path/to/python3.10 -m unittest discover -s gr3_gripper/test -v
```

只读 preflight 对上述 Python、NumPy、MuJoCo 和模型检查通过；ROS2 环境及
`rclpy` 检查失败，所以整体结果如实为 `ok=false`，没有发送任何命令。

输入溯源（输入文件本身不随本包重新分发）：

- 用户 `gripper_controller.py` SHA256：`ef33ba3a812a5b82ca6321ba4e9f72b140dbf41d88bad41484a0294f13f8e364`
- `fixed_arm.mjb` SHA256：`be5d1aa8e6d1ee129b8b7ad143e9958ab76f89a5abeceb58c00050c854b7411e`

## ROS2 自动检查

仓库工作流 `Gripper ROS2 Humble` 使用官方 `ros:humble-ros-base-jammy` 容器，
构建独立包，并运行真实 rclpy 的双进程测试：一个进程执行合成 MuJoCo 模型，
另一个进程通过 ROS2 请求闭合 0.8、再张开 0，检查两次 `RAMP_COMPLETE`。
运行前强制导入 rclpy，避免把缺依赖的 skip 当成通信通过。

本次 GitHub 写入返回 403，工作流尚未上传或运行，ROS2 通信验证仍待执行。
上传后工作流结果以对应提交的 GitHub Actions 记录为准。该容器的系统 Python
补丁版本由镜像决定，不等同于用户的 Python 3.10.12 镜像验收。

## 尚需目标环境验证

- Ubuntu 22.04.5 与 `fourier_aurora_sdk_gr3:v1.3.0` 的实际部署。
- 宿主 Conda Python 3.10.22 的 rclpy 导入与 DDS 跨容器发现、传输。
- 将 `bridge.step(data)` 接入用户正在运行的 viewer 的原物理循环。
- 完整机器人模型、接触/夹持质量，以及任何真实硬件驱动。

本模块不调用 Aurora SDK，也不更换推理依赖。ROS2 测试模型是自建夹具，
没有把 Kinova 的 ROS1 描述包冒充 ROS2 驱动或已转换好的完整机器人模型。
