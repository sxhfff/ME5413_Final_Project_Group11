# FAST-LIO 接入说明

## 当前项目正在用什么建图算法

当前总入口是 `me5413_world/launch/mapping.launch`。

- 默认算法是 `gmapping`
- 可选算法有 `gmapping`、`rtabmap`、`cartographer`
- 现在已经额外接入了 `fast_lio_slam` 选项

默认话题:

- 2D 激光: `/front/scan`
- 3D 点云: `/mid/points`
- IMU: `/imu/data`
- 轮速融合里程计: `/odometry/filtered`

## 这次接口改了什么

`mapping.launch` 现在支持:

```bash
roslaunch me5413_world mapping.launch slam_method:=fast_lio_slam
```

新增文件:

- `launch/include/fast_lio_slam.launch`
- `config/fast_lio_velodyne.yaml`

这套配置假设你安装的是 ROS1 官方 `FAST_LIO` 仓库，对应节点:

- 包名: `fast_lio`
- 节点名: `fastlio_mapping`

并且按 Velodyne 模式使用:

- 点云话题: `/mid/points`
- IMU 话题: `/imu/data`

## 推荐安装方式

项目当前仿真是 Velodyne 风格点云，不是 Livox 自定义消息，所以优先按 `FAST_LIO` 的 Velodyne 配置接入。

官方仓库:

- https://github.com/hku-mars/FAST_LIO

官方 README 明确说明:

- ROS >= Melodic
- Ubuntu >= 16.04
- Velodyne / Ouster 需要编辑 `config/velodyne.yaml`
- 关键参数包括 `lid_topic`、`imu_topic`、`timestamp_unit`、`scan_line`、`extrinsic_T`、`extrinsic_R`

## 安装步骤

### 1. 安装依赖

```bash
sudo apt update
sudo apt install -y libpcl-dev libeigen3-dev ros-noetic-tf2-eigen ros-noetic-geographic-msgs
```

如果你的系统没有 catkin 工具，也一起装:

```bash
sudo apt install -y python3-catkin-tools
```

### 2. 拉取 FAST_LIO

建议直接放到当前工作区 `src` 下:

```bash
cd /home/yushi/ros_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init --recursive
```

### 3. 编译

```bash
cd /home/yushi/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

### 4. 启动仿真世界

```bash
source /opt/ros/noetic/setup.bash
source /home/yushi/ros_ws/devel/setup.bash
roslaunch me5413_world world.launch
```

### 5. 启动 FAST-LIO 建图

另一个终端:

```bash
source /opt/ros/noetic/setup.bash
source /home/yushi/ros_ws/devel/setup.bash
roslaunch me5413_world mapping.launch slam_method:=fast_lio_slam
```

## 上线前必须核对

### 1. 点云字段时间戳

官方 README 提到，如果 PointCloud2 里缺少每点时间字段，会看到类似:

`Failed to find match for field 'time'.`

这意味着当前点云消息格式不符合 FAST-LIO 的预期。你需要核对 `/mid/points` 的字段是否包含时间，并确认 `timestamp_unit` 是否正确。

### 2. LiDAR 线数

当前配置先按 `VLP-16` 写成 `scan_line: 16`。如果你仿真里不是 16 线，要改这个值。

### 3. LiDAR 到 IMU 外参

目前 `config/fast_lio_velodyne.yaml` 里先设成:

- `extrinsic_est_en: true`
- `extrinsic_T = [0, 0, 0]`
- `extrinsic_R = I`

这只是为了先把接口接通，不是最终标定值。稳定使用前，应该把 LiDAR 在 IMU 坐标系下的真实外参填进去，并把 `extrinsic_est_en` 改成 `false`。

### 4. 可能不需要 `/odometry/filtered`

FAST-LIO 主要吃点云和 IMU，不依赖当前 2D SLAM 使用的 `/odometry/filtered`。如果后面你要让导航栈继续接它的输出，需要再检查:

- FAST-LIO 发布的 odom / path / tf
- 导航节点期望的 `map -> odom -> base_link` 关系

## 快速检查命令

```bash
rostopic list | egrep '/mid/points|/imu/data|/front/scan|/odometry/filtered'
```

```bash
rostopic echo -n 1 /imu/data
```

```bash
rostopic echo -n 1 /mid/points
```

```bash
rosparam get /fastlio_mapping/common/lid_topic
rosparam get /fastlio_mapping/common/imu_topic
```

## 如果启动失败，优先检查

- `rospack find fast_lio` 能否找到包
- `/mid/points` 是否真的是 `sensor_msgs/PointCloud2`
- 点云里是否带每点时间字段
- `scan_line` 和 `timestamp_unit` 是否匹配
- LiDAR / IMU 外参是否正确
