# 来源与适配说明

模型以用户指定的 [Kinova ros_kortex kinetic-devel](https://github.com/Kinovarobotics/ros_kortex/tree/kinetic-devel/kortex_description/grippers/robotiq_2f_85) 为准，固定使用提交 `d2bf9e89eb7c6c7b5dedc4f1b96c169f7f7b0013` 下该目录的 Xacro、网格、README 和许可证。原 README 记录其更早来源为 ROS-Industrial robotiq 提交 `4e3196c1c6a83f54acfa3dd8cf02b575ebba3e53`。许可证原文见 LICENSE（BSD 2-Clause），来源文件保留在 upstream/。

[MuJoCo Menagerie robotiq_2f85](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotiq_2f85) 作为 MJCF 联动/碰撞处理的参考，未直接采用其 2f85.xml、网格或惯量。两者不是可无条件互换的模型：Kinova 基座质量 0.22652 kg，Menagerie 为 0.777441 kg；关节结构与联动实现也不同。最终本包按 Kinova 的六个转动关节和 mimic 关系转换。

适配内容：

- Kinova 的原始 base STL 直接保留；8 个 DAE 按 Xacro 的 0.001 比例转换为 STL，保留视觉与碰撞网格区分。build_model.py 可用 Python3.10+ 与 NumPy 重建生成资源。
- 质量、质心、惯性张量、关节安装变换和限位来自 Xacro。固定 finger/pad 链不额外增加关节。
- URDF mimic 转成 MuJoCo joint equality（软约束，非机械硬锁）；一个 position actuator 控制主关节 0–0.8 rad，其余五个联动。
- 无额外基座旋转关节；夹爪基座固定在 GR3 right_end_effector_link 下，随原手臂及手腕运动。GR3 原腕 yaw/pitch/roll 均保留，夹爪未固定在世界坐标。
- 适配坐标使用 RotX(pi)，同时保留 Kinova 基座 RotZ(1.57)。前者是本次适配选择，仍需整机观察朝向和碰撞。
- damping=0.05、armature=0.0001、kp=20、执行器力限 ±2、pad 摩擦等属于仿真初始调参，不能解释为厂家力控性能。碰撞排除用于相邻/重叠联动部件，需在整机接触测试中复核。
- 独立测试验证空夹开合；未验证抓取稳定性、额定夹持力、自适应接触或完整机械四连杆接触行为。

本目录的 standalone.xml 仅供独立模型验证。viewer 中实际使用 definitions.xml 和 body.xml；HandController 始终操作 viewer 的现有模型与数据。
