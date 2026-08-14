# Scout Helios-16 实物播包 LIO 交接（审核用）

日期：2026-08-15  
对象：未参加本轮调试、需要独立复核的同事  
仓库：`XGC-Team/xgc2-faster-lio` 分支 `scout-helios16`（当前 `0667ccf`）  
配套：`XGC-Team/xgc2-lidar-imu-calib`（当前 `7ebba1e`）

一句话：实物 60 s 包上 Faster-LIO **能稳定出 10 Hz 里程计**，但 RViz 里积累的走廊会 **绕 Z 开花**。换过 24° 辨识偏航和单位阵后问题仍在。仿真 URDF 里有雷达–IMU 真值，可以当先验再拟合，但 **不能把仿真外参直接当成实车外参**。

---

## 1. 现象

在隔离 ROS master `http://127.0.0.1:11312` 上播

`/media/lxk/U128/xgc2-rosbags/agilex-all-20260815-000809.ring.bag`

并开 Faster-LIO + RViz（`/cloud_registered`，Decay 90 s）。

- 节点不崩，`/Odometry`、`/cloud_registered`、`/path` 约 10 Hz。
- 单帧走廊轮廓能看出来。
- **点云一积累，过道绕竖直轴转开 / 花开**。现场判断：像在 XY 里绕 Z 狂转，不是单纯可视化透明度问题。

请审核时先区分两种“花”：

1. **算法花**：单圈 60 s、不 loop，Decay 调成 0 再慢慢加大，第一圈就开始转。
2. **叠包花**：`rosbag play --loop` 且 Decay≥60 s。LIO **不会在 bag 循环时清图**，第二圈会把同一条走廊画到当前位姿上，看起来也像绕 Z 开花。

当前调试长期用了 `--loop` + Decay 90，两种可能叠在一起。复核请 **先只播一遍、不循环**。

---

## 2. 这个包是怎么回事

| 项 | 事实 |
| --- | --- |
| 原始包 | `/media/lxk/U128/xgc2-rosbags/agilex-all-20260815-000809.bag`（U 盘另有同名副本；默认 master 还在 loop 播 `/home/lxk/data/agilex-rosbags/` 那份） |
| 时长 | 59.8 s |
| 雷达 | `/rslidar_points`，597 帧，约 10 Hz，`16 × 1800` organized，`frame_id=rslidar` |
| 原始字段 | 只有 `x y z intensity`（`POINT_TYPE=XYZI` 录的） |
| ring 改写包 | 同目录 `*.ring.bag`：按官方行主序补了 `ring=行号`，**没有**编造逐点硬件时间 |
| 无效回波 | 一帧约 28800 点里约 1803 个 NaN |
| IMU | `/imu/data_raw`，11945 帧，约 200 Hz，`frame_id=imu_link` |
| 其它 | RealSense 彩色、少量深度、`/scout_status`、`/tf` |
| **没有** | `/odom`、逐点 `timestamp` / `time`、官方 `XYZIRT` |

约束（已约定、不要破）：

- 禁止自己编造逐点时间戳。
- 无硬件时间时，Faster-LIO 走公开的 yaw 去畸变：`omega_l = 3.61 deg/ms`（约 10 Hz 机械转速），见 `HesaiHandler`（`lidar_type: 5` 的 RoboSense 也走这条）。
- 日志里每帧 `Failed to find match for field 'timestamp'` 是预期的：PCL 把缺失的 `timestamp` 读成 0，于是走 yaw 模型。

机载驱动后续应使用 `rslidar_sdk` `POINT_TYPE=XYZIRT`（已改在 `XGC-Team/xgc2-robot-agilex`）。**这份包没有该字段。**

运动性质：平面、时间短。几乎只有 `ω_z`。对这种运动：

- 平移杠杆臂不可观。
- **绕 Z 的安装偏航也不可观**：`R_z` 把 `[0,0,ω]` 映成自己。Wahba 若仍输出十几度 yaw，那是 `ω_x/ω_y` 噪声，不是标定。

---

## 3. 工作区现状

### 3.1 仓库

| 用途 | 位置 | 远程 / 分支 / 提交 |
| --- | --- | --- |
| Faster-LIO 二次开发 | `/tmp/faster-lio`（`/tmp/faster-lio-ws/src/faster_lio` 软链到此） | `XGC-Team/xgc2-faster-lio` `scout-helios16` `0667ccf` |
| 离线外参辨识 | `/tmp/xgc2-lidar-imu-calib` | `XGC-Team/xgc2-lidar-imu-calib` `main` `7ebba1e` |
| Scout Gazebo 源码 | devops 目录 `products/ros1/simulator/gazebo-sim/agilex` | `lxk36/xgc2-gazebo-sim-agilex` `xgc2-noetic` `b36a853`（**尚未迁到 XGC-Team**） |
| 已安装的仿真包 | `/opt/ros/noetic/share/gazebo_sim_scout` | **没有** Helios / `enable_rslidar`，必须 overlay 源码树 |
| 目录指针 | `lxk36/xgc2-devops` `products/ros1/perception/slam` 与 `lidar-imu-calib` | 目录里的 Faster-LIO 副本偏旧（仍可能是 `extrinsic_est_en: true`），**不要当运行树** |

### 3.2 实物配置（当前磁盘）

`/tmp/faster-lio/config/scout_helios16.yaml`：

- `extrinsic_est_en: false`
- `extrinsic_T: [0, 0, 0.307]`（把 IMU 放在 base、雷达高 0.307 m 的几何先验）
- `extrinsic_R: I`
- `time_offset_lidar_to_imu: 0.0`（Faster-LIO **根本不读** 这个键）
- `lidar_type: 5`，`scan_line: 16`，`fov_degree: 180`（Helios 是 360°，180 是否裁扫描值得查）
- `dense_publish_en: true`，`point_filter_num: 2`

RViz：`config/rviz/scout_helios16.rviz`，Fixed Frame `camera_init`，`/cloud_registered` Decay 90。

### 3.3 仿真模型里的外参真值（URDF，不是实车）

`mini.xacro`：

- `imu_joint`：`base_link → imu_link`，`xyz="0 0 0.12"`，`rpy="0 0 0"`
- `rslidar_joint`：`base_link → rslidar`，`xyz="0 0 0.307"`，`rpy="0 0 0"`
- `gpu_ray` 传感器 pose 相对 `rslidar` 为 0

Faster-LIO 定义 `p_imu = R * p_lidar + t`：

```
R = I
t = (0, 0, 0.307 − 0.12) = (0, 0, 0.187)
```

已写在 `config/scout_helios16_sim.yaml`。IMU 仿真话题是 `/ugv1/imu/data_raw`，不是实车的 `/imu/data_raw`。

**不能**把 `t_z=0.187` 未经核对就当成实车 AHRS 的安装。实车 yaml 一直按“IMU 在 base”用 `0.307`。两者差 12 cm 竖直杠杆，平面运动上几乎看不出，也解释不了绕 Z 开花。

### 3.4 本机进程（写文档时）

| Master | 进程 | 说明 |
| --- | --- | --- |
| `:11311` 默认 | `roscore` + `rosbag play -l --clock` **原始** XYZI 包 | 别人可能在看，**不要杀** |
| `:11312` 隔离 | `roscore -p 11312` + Faster-LIO + ring 包 `--loop` | 实物 LIO 实验 |
| DISPLAY | `:1` | RViz / Gazebo 用这个 |

隔离 master 上曾残留多个已播完的 `/play_*` 注册，RViz 子进程有时会掉。

---

## 4. 已经试过、可以排除或降权的

1. **在线估外参**（`extrinsic_est_en: true` + 单位阵初值）  
   `offset_R_L_I` 会转，IMU 箭头和点云对不齐。已关掉。

2. **Wahba 拟合出的约 24° yaw**（`/tmp/scout_extrinsic.json`）  
   RMSE 0.166 rad/s，`observable_rotation` 曾被误标为 true。平面 `ω_z` 上看不出 `R_z`。写进 LIO 后走廊绕 Z 开花。已回滚单位阵；辨识脚本在 `ω_xy/ω_z` 太小时会丢掉 yaw。

3. **NaN 使 `atan2` 中毒**  
   曾直接把节点打崩（`Time has to be finite`）。已在预处理里丢掉非有限点。这不是现在开花的原因。

4. **RViz 全黑 / 点太稀**  
   网格关、Path Alpha 0、odom Keep 1、Decay 0。显示配置已改。**开花不是显示开关问题。**

5. **`time_offset_lidar_to_imu: -0.04`**  
   互相关估过，但 Faster-LIO 源码不读该参数，改它无效。

---

## 5. 仍应审核的假说（按建议顺序）

1. **loop + 积累点云**造成的假开花。先 `rosbag play --clock` **不** `--loop`，看单圈。
2. **无逐点时间 + yaw 去畸变扫向/起点约定**和 Helios 实际转向不一致，帧内拧、转弯时往地图里抹。
3. **实车 IMU 轴向 / 陀螺符号**和雷达 ROS 系（X 前 Y 左 Z 上）不一致。AHRS 仍发 orientation；Faster-LIO 只用 acc+gyro。抽样重力大致在 +Z，但不排除绕 Z 差 90°/180°。
4. **恒定偏航外参真的不是 0**，但这份包标定不了。24° 和 0° 都花，更像 1–3，而不是“再猜一个 yaw”。
5. `fov_degree: 180` 裁掉后半球。
6. RoboSense 走了 `HesaiHandler` + `hesai_ros::Point`（先 `timestamp` 后 `ring`），与 `robosense_ros::Point` 字段顺序不同；缺字段时一般是 0，但仍应核对。

### 5.1 已做的仿真对照（2026-08-15 第二次尝试）

URDF 真值：`R = I`，`t = (0, 0, 0.187)`。

在隔离 master `:11313` overlay 源码树启动 `helios16.launch`。本机 `gpu_ray` 会把 `gzserver` 打成 segfault，已临时改成 CPU `ray`（16×720，10 Hz）。录了 61 s 包 `/tmp/scout_sim_calib.bag`（616 帧雷达，12303 IMU）。Gazebo `/odom` 显示车走了约 15.5 m，最大 `ω_z ≈ 0.44 rad/s`。

同一套 `identify_lidar_imu_extrinsic.py`，`--prior-t 0,0,0.187`：

- `observable_rotation: false`（`n_excited = 0`：雷达 ICP 角速度没过阈值）
- `observable_translation: false`
- 输出保持先验：`R = I`，`T = [0, 0, 0.187]`

已把这组参数写进实物 `scout_helios16.yaml`，在 `:11312` **单圈、不 loop** 重播 ring 包。这是对照，不是实车标定。竖直杠杆从 0.307 改到 0.187 **解释不了**绕 Z 开花。

---

## 6. 复核怎么复现

不要动默认 master `:11311`。

```bash
# 隔离 master 应已存在；没有就另开
export ROS_MASTER_URI=http://127.0.0.1:11312
export DISPLAY=:1
source /opt/ros/noetic/setup.bash
source /tmp/faster-lio-ws/devel/setup.bash
rosparam set /use_sim_time true

roslaunch faster_lio mapping_scout_helios16.launch rviz:=true

# 先单圈，不要 --loop
rosbag play --clock --rate 1.0 \
  /media/lxk/U128/xgc2-rosbags/agilex-all-20260815-000809.ring.bag
```

看 `/cloud_registered`：第一圈走廊是否已经绕 Z 转。再决定要不要查去畸变或 IMU 轴。

辨识（不要把 `observable_rotation: false` 的 R 抄进 yaml）：

```bash
rosrun lidar_imu_calib identify_lidar_imu_extrinsic.py \
  /media/lxk/U128/xgc2-rosbags/agilex-all-20260815-000809.ring.bag \
  --prior-t 0,0,0.307 \
  --output /tmp/scout_extrinsic.yaml \
  --report /tmp/scout_extrinsic.json
```

明天实车：`POINT_TYPE=XYZIRT`，静置 8–10 s，原地左右转 + 8 字 + 加减速，尽量给一点俯仰/侧倾，录 2–3 分钟。脚本：`record_calib_bag.sh`。

---

## 7. 不要做什么

- 不要给 XYZI 包编造逐点时间。
- 不要在里程计里打开 `extrinsic_est_en`。
- 不要把平面包 Wahba 的 yaw 当标定。
- 不要杀默认 master 上的 loop 播包。
- 不要用 `pkill -f`（会误杀本会话）。
- 不要把仿真 `t_z=0.187` 写成“已标定的实车外参”。

---

## 8. 本轮对审核者的请求

1. 确认单圈（无 loop）是否仍绕 Z 开花。
2. 若单圈就花：优先查 yaw 去畸变扫向和 IMU 轴，而不是再调一个偏航角。
3. 若只有 loop 才花：是实验方法问题，不是外参。
4. 仿真拟合只作对照，实车外参仍要靠明天的激励包或钢尺/工装测量。
