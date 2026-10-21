# GR3 Kitchen MuJoCo Scene
场景文件是gr3_kitchen.xml 这个文件把场景和gr3的框架全部写进了一个xml所以加载的时候直接加载该文件就可以，像是这样“python3 -m mujoco.viewer \  --mjcf=/workspace/gr3_kitchen_portable/scenes/gr3_kitchen.xml”
所有引用的资源包括贴图和物体模型都在assets文件夹中，下载的时候请一起下载

包含 Fourier、厨房家具和桌面抓取测试立方体的 MuJoCo 场景。资源采用相对路径引用，移动项目时请一起保留 `scenes` 和 `assets` 目录。

## 项目结构

```text
gr3_kitchen_portable/
├── README.md
├── scenes/
│   └── gr3_kitchen.xml
└── assets/
    └── 网格及贴图资源
```

## 场景内容

- GR3 V233 机器人模型。
- 餐桌、柜子、洗碗机、垃圾桶及房间墙壁。
- 一个可自由运动、参与碰撞的红色立方体，用于后续抓取实验。

立方体参数：

| 参数 | 数值 |
| --- | --- |
| Body 名称 | `grasp_cube` |
| 自由关节名称 | `grasp_cube_free` |
| Geom 名称 | `grasp_cube_geom` |
| 边长 | 0.05 m |
| 质量 | 0.1 kg |
| 初始中心位置 | `(0.2, -0.85, 0.786)` m |
| 桌面顶部高度 | 0.76 m |


## 在现有 GR3 容器中运行

以下命令在 Ubuntu 虚拟机终端执行，提示符类似 `pi@TurtleBot3`，不是容器内部的 `root@...`。

假设已有容器名为 `gr3`。将项目放在虚拟机的 `/home/pi/gr3_kitchen` 后执行：

```bash
sudo docker start gr3-gui
sudo docker exec gr3-gui mkdir -p /workspace/gr3_kitchen
sudo docker cp /home/pi/gr3_kitchen/. gr3:/workspace/gr3/
```

在 Ubuntu 图形桌面会话中授权显示并打开场景：

```bash
xhost +SI:localuser:root
sudo docker exec -it \
  -e DISPLAY="$DISPLAY" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  gr3 python3 -m mujoco.viewer \
  --mjcf=/workspace/gr3_kitchen_portable/scenes/gr3_kitchen.xml
```
