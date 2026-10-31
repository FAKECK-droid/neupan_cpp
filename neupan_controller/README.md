# neupan_controller

ROS 2 Humble / Nav2 差速 Controller 插件，调用 `libneupan`，输出底盘坐标系的
`linear.x` 和 `angular.z`。不包含全局规划器，不读取 costmap 栅格，不建立局部障碍地图。

## 数据流与 Nav2 接口

`planner_server → ComputePathToPose → BT 的 {path} → FollowPath →
controller_server → setPlan(path) → libneupan`。

路径由 Nav2 action/BT 传递，不订阅 `/plan`、`/initial_path` 或目标话题。
`setPlan` 保留完整全局路径；第一次计算时调用库的 `replaceInitialPath`，
由库重采样并投影当前进度，后续求解保留 warm start。
Nav2 返回单个目标 pose 时，仅补入当前 pose 以适配库的外部路径接口。路径坐标使用
`Path.header.frame_id`；所有 poses 应属于该坐标系。

- 位姿：controller_server 从其 Costmap2DROS 的 TF 获取，并传入
  `computeVelocityCommands(pose, velocity, goal_checker)`。插件把位姿变换到全局路径坐标系。
- 速度：controller_server 从 `odom_topic` 获取；插件用传入的实际
  `linear.x/angular.z` 和一个控制周期构造库的 `Command`，约束下一步加速度。
- 轮廓：configure 时调用 `getUnpaddedRobotFootprint()`，转换为库的凸多边形。
  不额外声明 length/width/vertices。轮廓固定于 configure 时；更换轮廓和模型后需重新 configure。
- 输出：返回 `TwistStamped`，由 Humble controller_server 发布 `cmd_vel`
  （`geometry_msgs/msg/Twist`）；插件不自行发布底盘命令。
- 到达：Nav2 goal checker 负责。库的位置到达锁存关闭，接近终点后按 goal checker 的 XY/yaw
  容差做原地转向；仍每周期调用库检查当前障碍距离并要求求解成功。转向角速度受
  `max_speed[1]` 和 `max_acce[1]` 约束。这里复用库的当前距离急停，不另加旋转扫掠碰撞检测。

## 编译和接入

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to neupan_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select neupan_controller
colcon test-result --verbose
```

测试包含 pluginlib 加载和真实 ControllerServer 的生命周期 / FollowPath 闭环，
使用独立 ROS 域及同一进程的合成 TF、里程计、点云，不连接硬件。

把 [config/neupan_controller.yaml](config/neupan_controller.yaml) 的
`controller_server` 和 `local_costmap` 两段合并到已有 Nav2 参数文件，
沿用已有的 planner_server、global_costmap、BT、定位和 lifecycle manager。
在已有 launch 中，将 controller_server 的启动 package/executable 改为
`neupan_controller` / `neupan_controller_server`，节点名仍是 `controller_server`。
此入口使用独立进程；已有 bringup 使用 composition 时，需把 controller_server 单独改为
普通 Node 启动项，不能只修改 Controller 插件类型。也可以手动启动（随后仍由 lifecycle manager 管理）：

```bash
ros2 run neupan_controller neupan_controller_server --ros-args --params-file /absolute/path/to/nav2_params.yaml
```
插件类型为 `neupan_controller::NeupanController`，Controller ID 为 `FollowPath`。
示例文件是控制器配置片段，不是完整 Nav2 bringup 配置。

Humble 的 controller_server 在内部创建、configure/activate local_costmap，不能只靠
Controller 插件把该节点完全移除。示例保留一个很小的空容器，`publish_frequency: 0.0`；
随包的 `neupan_controller_server` 只继承原生 ControllerServer，在构造后把
`plugins` / `filters` 设置为带类型的空 string array，不覆盖任何控制、action 或生命周期方法。
这是因为本机 Humble 无法把 YAML 的 `[]` 解析为有类型的空数组，会在创建节点时报
`No parameter value set`；不要在参数文件里写这两个空数组。
无 obstacle/static/inflation layer，
无栅格参与 NeuPAN。保留 `update_frequency: 1.0`，让 Costmap2DROS 的启动初始化正常完成；
不要设为 0，否则其 start() 等待 initialized_ 的流程无法完成。
若要连此容器也不创建，必须修改 controller_server，超出普通插件的职责。
插件也可以由标准 `nav2_controller/controller_server` 加载，但其 costmap 图层需要由调用方配置；
本包的小入口用于保证完全不加载局部栅格图层。

示例轮廓 **1.6 × 2.0 m** 严格匹配自带 `diff_default.bin`，用于验证接入，
不是 Scout Mini 尺寸。实际使用时同时改 `local_costmap.footprint` 和
`FollowPath.dune_checkpoint`，使用按真实底盘训练、导出的 NPTF `.bin`。
库会检查模型和轮廓的矩阵一致性；请保留训练时的顶点顺序，不能只保证多边形看起来相同。
`robot_radius` 生成的圆近似轮廓也必须有对应训练模型。默认读取未加 padding 的轮廓，
安全距离使用 `collision_threshold` 和 `adjust.d_min/d_max`。

## 话题与 TF

| 来源/方向 | 配置或话题 | 类型/用途 |
|---|---|---|
| planner_server → BT → controller_server | `FollowPath.path` | `nav_msgs/msg/Path`，全局路径 |
| 输入插件 | `FollowPath.points_topic: points` | `sensor_msgs/msg/PointCloud2`，雷达或深度相机障碍点 |
| 输入 controller_server | `odom_topic: odom` | `nav_msgs/msg/Odometry`，底盘实际速度 |
| 输入 | `/tf`、`/tf_static` | 路径坐标系、odom、base_link、传感器之间的变换 |
| 输入 controller_server | `speed_limit` | `nav2_msgs/msg/SpeedLimit`，Nav2 外部线速度上限 |
| 输出 controller_server | `cmd_vel` | `geometry_msgs/msg/Twist` |

话题默认使用相对名称，跟随 controller_server 的 namespace。
雷达例：`points_topic: /lidar/points`；相机例：
`points_topic: /camera/depth/color/points`。一次订阅一路；多传感器需要上游融合。
插件 QoS 为 SensorDataQoS（best effort、volatile、depth 1），兼容 best effort/reliable 发布者。

点云要求 little-endian FLOAT32 的 `x/y/z` 字段，允许额外字段、organized 点云及行 padding。
先按点云时间戳变换到 `robot_base_frame`，按车体系高度和 XY 距离筛选，再转到全局路径坐标系，
取 XY 交给库。相机 optical frame 可直接输入，只需正确的 TF。
NaN/Inf 丢弃；地面、顶棚和车体自反射应通过高度参数及上游感知处理。
不添加 voxel map、ray tracing、障碍保留或额外下采样，点数选择交给库的 PAN/DUNE。

典型 TF：`map → odom → base_link → lidar/camera_optical_frame`。
点云时间戳必须与 ROS 时钟一致；`pointcloud_timeout` 按消息采集时间判断，
不按接收时间延长旧观测。需要保留点云采集时刻的 TF。
新鲜空点云（或筛选后无点）表示当前没有障碍；缺少消息、过期消息、未来时间戳、TF 缺失、
字段格式错误则抛出 Nav2 异常。插件不再使用上一帧障碍继续行驶。
仿真时统一给 controller_server、local_costmap、planner_server 和传感器设置 `use_sim_time`。

## 参数

插件参数均在 `controller_server.ros__parameters.FollowPath` 下；完整值见示例 YAML。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `points_topic` | `points` | 输入点云 |
| `pointcloud_timeout` | 0.5 s | 消息最大年龄 |
| `min_obstacle_height / max_obstacle_height` | 0.05 / 1.5 m | 车体系 Z 范围 |
| `min_obstacle_range / max_obstacle_range` | 0 / 10 m | 车体系 XY 范围 |
| `dune_checkpoint` | 包内 `models/diff_default.bin` | NPTF 权重绝对路径 |
| `receding` | 10 | MPC 步数 |
| `step_time` | 1 / controller_frequency | MPC 步长 |
| `ref_speed` | 0.5 m/s | 参考速度 |
| `max_speed` | [0.8, 1.0] | [线速度, 角速度] 上限 |
| `max_acce` | [1.0, 2.0] | [线加速度, 角加速度] 上限 |
| `collision_threshold` | 0.1 m | 库的当前障碍距离急停阈值 |
| `rotate_to_goal_gain` | 2.0 | 终点转向比例增益 |
| `ipath.interval / close_threshold / ind_range` | 0.05 / 0.1 / 10 | 库的路径采样与进度搜索 |
| `pan.*`、`adjust.*` | 见 YAML | 直接映射库的 PAN/NRMP 参数 |

`transform_tolerance`、`robot_base_frame` 复用 local_costmap 配置，
`controller_frequency`、`failure_tolerance`、`odom_topic` 为 controller_server 参数，
XY/yaw 容差由 goal_checker 配置，要求 goal checker 支持 `getTolerances()`（例如 SimpleGoalChecker）。

Nav2 SpeedLimit 的绝对值单位为 m/s，百分比相对于配置的最大线速度，
0 恢复配置上限。限制会降低参考速度，并在必要时同比缩放 v/w 保持曲率。

库抛出的配置/轮廓/路径错误不重复检查。运行时库的 `Info.stop` 或
`!Info.solved` 转为 `nav2_core::PlannerException`，不发送未避障的 nominal。
Humble controller_server 按 `failure_tolerance` 发送零速度并等待恢复或终止 FollowPath。
插件不增加失败计数、重试求解、自己的定时控制循环或全局规划回退。

## 动态调参

在 configure 后注册参数回调，cleanup 时移除；active 和 inactive 状态都支持下表中的动态参数。
回调与控制计算共用互斥锁，先检查整批插件参数，再替换运行配置。输入格式错误、非有限值、
无效范围或库拒绝的配置会返回失败，保留原来的插件配置；库已有的距离关系校验不在插件中重复实现。

| 参数 | 更新方式 |
|---|---|
| `pointcloud_timeout`、`min_obstacle_height`、`max_obstacle_height`、`min_obstacle_range`、`max_obstacle_range` | 直接更新，下一控制周期采用新筛选条件 |
| `rotate_to_goal_gain` | 直接更新，下一控制周期采用新增益 |
| `ref_speed` | 调用库的 setReferenceSpeed，保留当前规划状态 |
| `max_speed`、`max_acce`、`collision_threshold` | 按新配置重建核心规划器 |
| `ipath.close_threshold`、`ipath.ind_range` | 按新配置重建核心规划器 |
| `pan.iter_num`、`pan.dune_max_num`、`pan.iter_threshold` | 按新配置重建核心规划器 |
| `adjust.q_s`、`adjust.p_u`、`adjust.eta`、`adjust.d_max`、`adjust.d_min`、`adjust.ro_obs`、`adjust.bk` | 调用库的 NRMP::updateParams，下一次求解重建 QP，保留路径和标称控制序列 |

参考速度和 adjust 参数复用库已有的更新、校验接口，不重新加载模型。
其余表中标记为重建的参数，使用库构造接口重建规划器，重新加载模型并清空求解 warm start，
但保留收到的 Nav2 全局路径，
下一周期由库投影当前位置恢复路径进度；第一步仍使用实际里程计速度约束加速度。
最近收到的 Nav2 SpeedLimit 会重新应用，百分比限速以更新后的最大线速度为基准。

例如：

```bash
ros2 param set /controller_server FollowPath.ref_speed 0.6
ros2 param set /controller_server FollowPath.max_speed "[0.8, 1.2]"
ros2 param set /controller_server FollowPath.adjust.q_s "[2.0, 2.0, 1.0]"
ros2 param set /controller_server FollowPath.max_obstacle_height 1.2
```

涉及互相约束的参数（如 `adjust.d_min/d_max`），可以通过 ROS 2
`/controller_server/set_parameters_atomically` 服务在同一请求中修改，避免中间组合无效。

`points_topic`、`dune_checkpoint`、`receding`、`step_time`、`ipath.interval`、
`pan.nrmp_max_num` 以及插件类型属于初始化参数；插件处于已配置状态时会明确拒绝修改。
需要先 `deactivate → cleanup`，再设置参数，随后 `configure → activate`，或修改 YAML 后重启。
轮廓、base frame、TF 容差和控制周期仍在 configure 时从 Nav2 读取，
不由插件参数回调更新。Nav2 自身参数的动态行为由其对应组件负责。

## 接口依据

已对照 Humble 源码：

- [nav2_core/controller.hpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_core/include/nav2_core/controller.hpp)
- [controller_server.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp)
- [costmap_2d_ros.hpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/include/nav2_costmap_2d/costmap_2d_ros.hpp)
- [costmap_2d_ros.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_ros.cpp)
- [layered_costmap.cpp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/layered_costmap.cpp)
