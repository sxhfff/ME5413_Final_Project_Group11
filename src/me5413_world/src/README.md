# me5413_world/scripts README

这个目录存放了一楼任务流程中用到的主要 Python 脚本，功能大致可以分成 4 类：

1. 导航巡航
2. 方块检测
3. 数字识别
4. 主流程编排与辅助工具

## 目录概览

| 文件 | 作用 | 类型 |
| --- | --- | --- |
| `start_robot.py` | 主流程脚本，串联检测、巡航、识别 | 主入口 |
| `findcube.py` | 基于 `/front/scan` 做聚类，检测地图中的方块位置 | 感知 |
| `snake_path.py` | 在指定多边形区域内生成蛇形巡航路径并发送导航目标 | 导航 |
| `subscribe_box_pos.py` | 读取方块坐标，规划访问顺序，到每个方块前识别数字并统计结果 | 任务执行 |
| `digit_recognition_service_node.py` | 提供“开始识别 / 停止识别”ROS服务，并在 RViz 中显示识别结果 | 服务节点 |
| `digit_recognizer.py` | 数字模板匹配识别核心逻辑 | 感知模块 |
| `publish_initial_pose.py` | 向 `/initialpose` 发布一次 AMCL 初始位姿 | 辅助工具 |
| `waypoint_patrol.py` | 简单的固定航点巡航测试脚本 | 调试脚本 |
| `templates/` | 数字识别模板图像目录 | 资源 |

## 推荐运行关系

完整流程的设计思路如下：

- `start_robot.py`
  - 持续启动 `digit_recognition_service_node.py`
  - 持续启动 `findcube.py`
  - 单次执行 `snake_path.py`
  - 单次执行 `subscribe_box_pos.py`

也就是说，如果希望按项目当前逻辑跑完整任务，优先使用：

```bash
cd ~/5413/ME5413_Final_Project_Group11/src/me5413_world/scripts
python3 start_robot.py
```

前提是 ROS 核心环境、定位、建图/导航、`move_base`、TF 等已经正常启动。

## 各脚本说明

### 1. `start_robot.py`

这是总控脚本，内部通过 `subprocess` 和线程管理其他脚本的生命周期。

执行顺序：

1. 持续运行 `digit_recognition_service_node.py`
2. 持续运行 `findcube.py`
3. 执行一次 `snake_path.py`
4. 执行一次 `subscribe_box_pos.py`
5. 结束后停止前面两个持续运行的脚本

特点：

- `findcube.py` 和数字识别服务如果异常退出，会被自动拉起
- 给 `findcube.py` 传入了一楼区域限制参数：
  - `_region_min_x:=-2.0`
  - `_region_max_x:=23.0`
  - `_region_min_y:=-2.0`
  - `_region_max_y:=22.5`

适用场景：

- 作为比赛/演示时的一键启动入口

### 2. `findcube.py`

作用：

- 订阅激光雷达 `/front/scan`
- 使用 DBSCAN 对点云聚类
- 通过 TF 将激光坐标转换到 `map`
- 根据包围框大小筛选可能的方块
- 对多次检测结果做融合
- 发布可视化 marker 和方块信息

主要输入：

- `/front/scan` (`sensor_msgs/LaserScan`)
- TF：雷达坐标系到 `map`

主要输出：

- `/cluster_markers_map`：聚类与方块的 RViz Marker
- `/found_blocks_info`：字符串格式的方块信息，例如 `(id,x,y,label)`

重要参数/常量：

- `CLUSTER_EPS = 0.2`
- `CLUSTER_MIN_SAMPLES = 10`
- 有效方块宽高范围：`0.6m ~ 0.9m`
- 输出坐标系：`map`
- 默认地图 YAML：`../maps/my_map.yaml`

补充说明：

- 脚本会从地图 YAML 和地图图片自动计算有效地图边界
- 启动时也可以通过私有参数限制检测区域，例如：

```bash
python3 findcube.py _region_min_x:=-2.0 _region_max_x:=23.0 _region_min_y:=-2.0 _region_max_y:=22.5
```

### 3. `snake_path.py`

作用：

- 在预设的一楼多边形区域内自动生成蛇形覆盖路径
- 根据当前位置选择更合理的起始方向
- 对长路径段进行插值细分
- 逐个向 `move_base` 发送导航目标

主要输入：

- `amcl_pose`
- `move_base`

主要逻辑：

- 如果机器人还在起始房间附近，会先去门口过渡点
- 使用四边形边界生成蛇形路径
- 对边界做内缩，避免贴墙太近
- 每段路径根据下一个点自动计算朝向

关键参数：

- `START_ROOM_POSE = (0.0895, 0.00393)`
- `DOOR_TRANSITION_WAYPOINT = (2.51, 0.588)`
- `MAX_SEGMENT_LENGTH = 1.5`
- `WALL_MARGIN = 1.5`
- 当前蛇形列数：`num_columns = 4`

适用场景：

- 对一楼区域进行覆盖式巡航搜索

### 4. `subscribe_box_pos.py`

作用：

- 等待稳定的 `/found_blocks_info`
- 解析方块坐标
- 用动态规划近似求一条较短访问顺序
- 导航到每个方块前方固定偏移位置
- 调用数字识别服务识别方块上的数字
- 统计识别结果中出现频率最低的数字

主要输入：

- `/found_blocks_info`
- `amcl_pose`
- `move_base`
- `start_recognition` / `stop_recognition` 服务

主要输出：

- `/recognized_digits` (`Int32MultiArray`)
- `/least_frequent_digits` (`String`)
- 当前工作目录下的 `recognized_digits_summary.txt`

关键策略：

- 靠近方块时采用 `APPROACH_OFFSET_X = 1.3`
- 识别时长 `RECOGNITION_DURATION_SEC = 1.5`
- 导航失败的点会记录为 `-1`
- 最后会统计有效数字 `0~9` 中出现频率最低的数字

### 5. `digit_recognition_service_node.py`

作用：

- 封装数字识别为两个 ROS 服务：
  - `start_recognition`
  - `stop_recognition`
- 停止识别后输出“最佳识别结果”
- 在 RViz 中通过 `digit_recognition_marker` 显示当前数字

主要输出：

- 服务：`start_recognition`
- 服务：`stop_recognition`
- 话题：`digit_recognition_marker` (`visualization_msgs/Marker`)

行为说明：

- 启动时默认显示 `0`
- `start_recognition` 会重置显示并启动识别线程
- `stop_recognition` 会停止识别并返回最佳识别数字

### 6. `digit_recognizer.py`

作用：

- 真正执行数字模板匹配识别
- 从相机图像中找出与模板最相似的数字

主要输入：

- `/front/image_raw` (`sensor_msgs/Image`)

实现方法：

- 使用 `cv_bridge` 将 ROS 图像转为 OpenCV 图像
- 灰度化 + 直方图均衡化
- 对模板做多尺度缩放
- 用 `cv2.matchTemplate(..., cv2.TM_CCOEFF_NORMED)` 匹配
- 取得分最高且超过阈值 `0.7` 的数字

返回结果：

- 识别到数字时返回 `(digit, score, pixel_offset)`
- 未识别成功时返回 `None`

模板目录：

- 默认从当前目录下的 `templates/` 读取 `0.png ~ 9.png`

### 7. `publish_initial_pose.py`

作用：

- 向 `/initialpose` 发布一次 AMCL 初始位姿

可传入参数：

- `~x`
- `~y`
- `~yaw`

示例：

```bash
python3 publish_initial_pose.py _x:=0.0 _y:=0.0 _yaw:=0.0
```

适用场景：

- 启动导航前手动给机器人设置初始位姿

### 8. `waypoint_patrol.py`

作用：

- 按固定航点列表测试 `move_base`

特点：

- 航点写死在脚本内部
- 更适合作为早期调试脚本
- 当前文件中的注释也提示“后面替换成自己测出来的点”

适用场景：

- 单独验证导航是否正常
- 在不使用蛇形路径时做简单巡航测试

## 话题 / 服务速查

### 订阅的话题

| 脚本 | 话题 |
| --- | --- |
| `findcube.py` | `/front/scan` |
| `digit_recognizer.py` | `/front/image_raw` |
| `snake_path.py` | `amcl_pose` |
| `subscribe_box_pos.py` | `amcl_pose` |

### 发布的话题

| 脚本 | 话题 |
| --- | --- |
| `findcube.py` | `/cluster_markers_map`, `/found_blocks_info` |
| `digit_recognition_service_node.py` | `digit_recognition_marker` |
| `subscribe_box_pos.py` | `/recognized_digits`, `/least_frequent_digits` |
| `publish_initial_pose.py` | `/initialpose` |

### 使用的服务

| 脚本 | 服务 |
| --- | --- |
| `digit_recognition_service_node.py` | `start_recognition`, `stop_recognition` |
| `subscribe_box_pos.py` | 调用 `start_recognition`, `stop_recognition` |

## 依赖说明

这些脚本依赖以下 ROS / Python 组件：

- ROS `rospy`
- `move_base`
- `actionlib`
- `tf` / `tf2_ros`
- `cv2`
- `cv_bridge`
- `numpy`
- `scikit-learn` 中的 `DBSCAN`

如果某些脚本无法运行，通常优先检查：

- 是否已经 `source devel/setup.bash`
- `move_base` 是否启动
- TF 是否连通
- `/front/scan` 和 `/front/image_raw` 是否有数据
- AMCL 是否已经给出 `amcl_pose`

## 当前代码里的注意事项

1. `digit_recognizer.py` 会尝试加载 `templates/0.png ~ templates/9.png`，但当前目录里我只看到 `1.png ~ 9.png`。这意味着运行时大概率会警告 `0.png` 缺失。
2. `subscribe_box_pos.py` 会把结果文件写到“当前工作目录”下，所以从不同目录启动时，`recognized_digits_summary.txt` 的落点可能不同。
3. `waypoint_patrol.py` 里的航点仍然是测试值，不建议直接作为正式任务路径。
4. `start_robot.py` 用的是相对脚本名启动其他文件，所以最好在本目录下执行它。

## 建议使用方式

如果是正式流程：

```bash
cd ~/5413/ME5413_Final_Project_Group11/src/me5413_world/scripts
python3 start_robot.py
```

如果是分模块调试：

```bash
python3 findcube.py
python3 digit_recognition_service_node.py
python3 snake_path.py
python3 subscribe_box_pos.py
```


