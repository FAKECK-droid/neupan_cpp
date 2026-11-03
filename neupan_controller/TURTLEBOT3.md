# TurtleBot3 Waffle：官方仿真 + NeuPAN

适用环境：ROS 2 Humble、Gazebo Classic、官方 TurtleBot3 Waffle。新增入口为
`turtlebot3_neupan.launch.py`，包含官方 `nav2_bringup/tb3_simulation_launch.py`，
保留原版 `nav2_controller/controller_server`，默认使用官方独立进程模式
（`use_composition:=False`），也可显式启用 composition。

只替换 `controller_server.FollowPath` 的 DWB 配置，并将两个 costmap 的轮廓设为
训练使用的 Waffle 轮廓。AMCL、map_server、planner_server、BT、goal/progress checker、
behavior_server、velocity_smoother 和地图/场景均沿用官方配置。官方 local_costmap 图层
继续运行，NeuPAN 本身不读取其中的栅格；这里不使用本包的空图层 server 入口。

## 模型尺寸与参数来源

对照本机官方文件：

- `nav2_bringup/worlds/waffle.model` 和
  `turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf`。
- `nav2_bringup/urdf/turtlebot3_waffle.urdf` 和
  `turtlebot3_description/urdf/turtlebot3_waffle.urdf`。
- `nav2_bringup/params/nav2_params.yaml`。

URDF 底盘碰撞箱为 0.266 × 0.266 m，中心相对 `base_link` 向后偏移 0.064 m；
Gazebo SDF 的箱体是 0.265 × 0.265 m。车轮中心为 y=±0.144 m，轮胎宽 0.018 m，
外缘到 y=±0.153 m；Nav2 仿真模型的相机碰撞体前缘到 x=0.073 m。
向外取整后的包络矩形为 **0.275 × 0.310 m**，不是以驱动轴为中心的矩形：

```yaml
vertices:
  - [-0.200, -0.155]
  - [0.075, -0.155]
  - [0.075, 0.155]
  - [-0.200, 0.155]
```

唯一轮廓来源为 `config/turtlebot3_waffle_train.yaml` 的 `robot.vertices`。
训练、导出和仿真启动均读取该顺序；启动时将其写入 local/global costmap 的
`footprint`，覆盖官方 `robot_radius` 的圆形近似，保留原有 footprint padding。
插件读取未加 padding 的轮廓。修改顶点后必须重新训练，并在启动时使用对应训练配置。

速度限制沿用官方 Nav2 演示与 velocity_smoother：0.26 m/s、1.0 rad/s；
加速度 2.5 m/s²、3.2 rad/s²。参考速度为 0.20 m/s，20 Hz、20 步预测时域共 1 s。
激光最大使用距离 3.5 m；训练点范围为 [-5, 5] m 的正方形，覆盖当前观测和预测位移。
这些是仿真初始配置，避障阈值、权重和性能需由使用者在仿真中评估。

## 编译（不运行测试）

在本仓库根目录执行：

```bash
cd /root/scout_mini_ws/src/neupan_cpp
source /opt/ros/humble/setup.bash
colcon build --base-paths libneupan neupan_controller \
  --packages-up-to neupan_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DNEUPAN_BUILD_TESTS=OFF -DBUILD_TESTING=OFF
source install/setup.bash
```

仿真依赖 `nav2_bringup`、`turtlebot3_gazebo`、Gazebo Classic；
点云转换使用 `laser_geometry`。训练另外需要上游 NeuPAN 及其 Python 依赖、
支持 CUDA 的 PyTorch 和可用的 NVIDIA GPU。训练不依赖 ROS 节点运行。

## GPU 训练并导出

按上游 `NeuPAN/example/dune_train`、`neupan/blocks/dune_train.py` 和说明实现：
调用原有 `robot`、`ObsPointNet`、`DUNETrain.start()`，不复制训练损失或标签求解逻辑。
这里训练的是机器人几何对应的 **DUNE**，不包含 LON/MPC 权重训练。

先 source ROS 与本包安装环境，再激活已安装上游依赖和 CUDA PyTorch 的环境，例如现有 venv：

```bash
cd /root/scout_mini_ws/src/neupan_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash
source /root/scout_mini_ws/src/NeuPAN/.venv/bin/activate

ros2 run neupan_controller train_turtlebot3.py \
  --neupan-source /root/scout_mini_ws/src/NeuPAN
```

默认配置 `device: cuda:0`；可通过 `--device cuda:1` 选择另一张 GPU。
没有可用 CUDA 时明确报错，不自动回退 CPU。上游 CVXPY/ECOS 的随机点标签生成仍在
CPU 上进行；网络、训练张量和优化器使用 GPU。训练阶段尚未开始前的 CPU 标签生成
不代表 CUDA 配置无效。训练后的 C++ 控制器仍使用 libneupan 的 CPU 推理与求解。

默认 100000 个点、batch 2048、5000 epoch，沿用上游训练及其内部 validation/save 周期。
调用已有的 `libneupan/tools/export_dune_weights.py` 导出 NPTF；CMake 将该工具及
`nptf.py` 安装到训练脚本旁，训练结束自动导出最终 epoch，无需手工转换。

默认输出目录为 `~/.ros/neupan/turtlebot3_waffle/`：

- `model_final.pth`：最终网络权重；上游还会保存周期 checkpoint 与训练记录。
- `training_config.yaml`：本次使用的配置副本。
- `turtlebot3_waffle.bin`：控制器读取的 NPTF，内含训练轮廓的 G/h。

`--config /绝对路径/训练配置.yaml` 和 `--output-dir /绝对路径/输出目录` 可覆盖默认值。
同一输出目录再次训练会覆盖同名产物；保留旧模型时使用不同输出目录。
使用自定义训练配置启动仿真时，同时传入 `training_config` 与 `dune_checkpoint`。
默认配置使用包安装目录中的 YAML；编辑源码 YAML 后需重新编译安装，或显式传入源码路径。

## 启动仿真（由使用者执行）

训练完成后，在普通 ROS 终端执行：

```bash
cd /root/scout_mini_ws/src/neupan_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch neupan_controller turtlebot3_neupan.launch.py
```

启动文件同时保留 Gazebo 11 内置模型目录 `/usr/share/gazebo-11/models` 和
TurtleBot3 模型目录，避免覆盖环境变量后找不到 `sun/ground_plane`。
该官方场景的资源均已在本机安装，模型数据库使用包内的空本地索引
`config/gazebo_offline/database.config`，不等待旧的在线模型库扫描。
子进程 PATH 优先使用 `/usr/bin`，使官方 `spawn_entity.py` 使用系统 Python，
避免训练 venv 缺少 Ubuntu 的 ROS/lxml 依赖。此设置仅作用于当前 launch 的子进程。
默认权重路径与训练输出一致，不会使用为其他轮廓训练的 `diff_default.bin`，
也不在启动时自动训练。

自定义模型和配置示例：

```bash
ros2 launch neupan_controller turtlebot3_neupan.launch.py \
  dune_checkpoint:=/绝对路径/turtlebot3_waffle.bin \
  training_config:=/绝对路径/training_config.yaml \
  headless:=False use_composition:=False
```

其他启动参数包括 `controller_params_file`、`base_params_file`、`map`、`world`、
`use_rviz`、`use_sim_time`、`autostart`、`x_pose`、`y_pose`、`yaw`。
默认模型固定为官方 Waffle；切换 Burger/Waffle Pi 需要相应模型、轮廓及重新训练。
`base_params_file` 是未套 namespace 的完整 Humble Nav2 配置；
`controller_params_file` 是本包的 FollowPath 覆盖文件。

默认关闭 composition，是为了绕过本机日志中出现的
`/nav2_container/_container/load_node` 响应超时：组件加载器会逐个等待响应，
首个加载响应丢失会使后续 planner_server、bt_navigator 和导航 lifecycle manager
没有机会启动，RViz 随后提示 `navigate_to_pose action server is not available`。
独立进程模式沿用官方启动分支及同样的控制器/规划器配置。
该提示是通用信息，不能单凭其中的 initial pose 字样判断原因；
本机这次日志已记录 AMCL 收到初始位姿，实际卡点在导航组件加载阶段。

RViz 使用 `2D Pose Estimate` 设置初始位姿，默认出生点 (-2.0, -0.5)，朝向 +X；
再用 `Navigation2 Goal` 设置导航目标。全局路径仍由 planner_server 规划，经
BT 的 ComputePathToPose/FollowPath action 传给插件。

## 点云与命令链

```text
Gazebo /scan (LaserScan) ──→ AMCL / 官方 costmap
                       └─→ scan_to_cloud ─→ /scan/points (PointCloud2) ─→ NeuPAN
planner_server ─→ BT {path} ─→ 原版 controller_server / FollowPath ─→ NeuPAN
原版 controller_server ─→ /cmd_vel_nav ─→ velocity_smoother ─→ /cmd_vel ─→ Gazebo
```

转换节点只调用 `laser_geometry::LaserProjection::projectLaser`，保持点云的扫描坐标系
`base_scan` 和时间戳；不另建障碍地图。插件按点云时间戳做 TF、底盘系高度筛选和 XY 投影。
虽然输出是 PointCloud2，数据仍来自二维激光，不是新增三维传感器。
高度筛选 0.02–0.35 m 是相对 `base_link` 原点的范围，包含官方激光安装高度 0.122 m。
保留官方深度相机；该默认入口使用激光投影点云，不依赖相机点云的话题或渲染行为。

## 点云时间差与断续控制

本机运行中被动采样观察到 /clock 约 10 Hz、激光及点云约 5 Hz，
点云采集时间稳定比最近的 /clock 快约 0.042 s。这是独立话题到达顺序和时钟发布周期
造成的有限提前，不能用 age < 0 一律判错。
插件允许的时间范围为：
`-local_costmap.transform_tolerance <= now - cloud_stamp <= FollowPath.pointcloud_timeout`。
仍按真实 cloud_stamp 查 TF，超过范围时拒绝输入并报告具体时间差，不重写时间戳，
不增加旧点云超时值。

旧判断会反复触发原版 controller_server 的零速度输出；日志还记录了
Failed to make progress 以及 spin/wait/backup 恢复动作，表现为断续前进、后退或原地调整。
修复时间判断后需要重启正在运行的 controller_server 才能加载新插件；
若仍触发 NeuPAN collision stop，异常会给出距离与阈值，用于区分真实碰撞急停。

本次修改只执行编写与编译，训练、Gazebo 和导航运行验证由使用者完成。
