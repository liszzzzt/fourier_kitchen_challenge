# GR3 厨房场景更新说明

本更新包包含厨房、小物体、辅助桌和木地板所需的场景配置与资源。它是现有 GR3 环境的更新包，不是完整 SDK 或 Docker 镜像。

## 文件结构

```text
kitchen_export/
├── README.md
├── main.xml
├── kitchen.xml
├── assets/
│   ├── meshes/
│   └── textures/
└── components/
```

- `main.xml`：GR3 场景主入口，包含原地面及木纹材质配置。
- `kitchen.xml`：厨房资产声明、家具布局和小物体引用。
- `assets/`：家具和小物体的网格及贴图，包含木地板图片。
- `components/`：家具、小物体的 body 定义及自由关节配置。

必须使用修改木纹相对路径后重新导出的最新版 `main.xml`。本包不需要包含其他地形 XML、地形图片、`.bak` 或缓存文件，除非当前厨房确实引用它们。

以下命令在 **Ubuntu 虚拟机终端 `pi@TurtleBot3`** 执行，不是在容器 `root@...` 终端执行。

```bash
sudo docker start gr3
```
## 替换主入口与厨房入口
```bash
sudo docker cp /home/pi/kitchen_export/main.xml gr3:/usr/local/resources/GR3_V233/mjcf/main.xml
```
作用：替换 GR3 主入口及地板外观配置。同名文件会覆盖，不替换 `gr3v2_3_3.xml`。
```bash
sudo docker cp /home/pi/kitchen_export/kitchen.xml gr3:/usr/local/terrains/kitchen.xml
```
作用：替换厨房场景入口。
## 更新网格、贴图和组件
```bash
sudo docker exec gr3 mkdir -p /usr/local/terrains/assets /usr/local/terrains/components
```
作用：保证目标目录存在，不删除已有文件。
```bash
sudo docker cp /home/pi/kitchen_export/assets/. gr3:/usr/local/terrains/assets/
sudo docker cp /home/pi/kitchen_export/components/. gr3:/usr/local/terrains/components/
```
作用：复制两个目录的全部内部内容。`/.` 避免产生 `assets/assets` 或 `components/components` 的额外嵌套。
同名文件覆盖，容器中独有的旧文件不会删除。不要把整个项目直接复制到 GR3 的 `mjcf` 目录，也不要把本包 `main.xml` 当成地形文件放入 `terrains`。


## 使用原有入口运行

先在 **Ubuntu 虚拟机图形桌面终端** 执行：
```bash
xhost +SI:localuser:root
sudo docker exec -it gr3 bash
```
作用：授权本机 root 访问桌面，然后进入容器。沿用现有容器的 X11 挂载、DISPLAY 和渲染配置。
进入 **容器终端 `root@...`** 后，若使用 ROS2 RGB-D viewer，执行：
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=6
```
以下工作目录按本项目现有布局填写；若接收方 SDK 位于 `/workspace`，使用其原有工作目录即可，不需要移动 SDK。

```bash
cd /workspace/fourier_aurora_sdk
python3 sim/start_simulate_root.py
```
按菜单名称选择 **GR3_V233 → KITCHEN**，序号以实际菜单为准。


