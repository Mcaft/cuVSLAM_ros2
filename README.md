# cuvslam_bridge

ROS 2 (Humble) package: **cuVSLAM** visual(-inertial) odometry / SLAM on an Intel **RealSense D435i**.
标定、IMU 噪声、里程计模式、SLAM 参数**全部由 YAML 驱动**（ROS 2 参数），代码里没有硬编码的标定值。

## 依赖

| 组件 | 说明 |
|---|---|
| ROS 2 Humble | 系统已装 |
| `pyrealsense2` | **必须是 2.58.3 的 Python 绑定**（2.59.x 看不到 Motion Module）。本机：从 `~/drivers/nav_drivers/librealsense` 源码 `-DBUILD_PYTHON_BINDINGS=ON` 编译，`build/Release/pyrealsense2*.so` 放进 `~/msnet2d-venv/.../site-packages` |
| `cuvslam` | `pip install cuvslam-17.0.1+cu12-cp310-cp310-manylinux_2_35_aarch64.whl` |
| numpy / opencv | venv 内已装 |

节点直接读 RealSense，**不依赖 `realsense2_camera`**（那个包链接 librerealsense 2.59.2，没有 IMU）。

## 编译

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
ln -sfn ~/ros2_pkg/cuvslam_bridge src/cuvslam_bridge
colcon build --packages-select cuvslam_bridge --symlink-install
```

## 运行

```bash
source ~/ros2_pkg/cuvslam_bridge/env.sh     # ROS + venv + 包路径

ros2 launch cuvslam_bridge cuvslam.launch.py                       # 用 config/cuvslam.yaml
ros2 launch cuvslam_bridge cuvslam.launch.py config_file:=/path/to/rig.yaml
ros2 launch cuvslam_bridge cuvslam.launch.py use_slam:=true        # 开 SLAM（回环 + 地图点）
ros2 launch cuvslam_bridge cuvslam.launch.py odometry_mode:=stereo # 覆盖 yaml
ros2 launch cuvslam_bridge cuvslam.launch.py run_seconds:=30.0 snapshot_path:=$HOME/traj.png
ros2 launch cuvslam_bridge cuvslam.launch.py rviz:=true            # 需要桌面会话
```

实测（15W，640×480）：

| 配置 | 结果 |
|---|---|
| yaml 默认（inertial, auto_calibration） | `30.2 FPS · track 4.5 ms · IMU 30.0 Hz · slam=False` |
| `use_slam:=true` | `30.3 FPS · track 4.4 ms · slam=True` |
| `odometry_mode:=stereo` | `30.2 FPS · track 3.9 ms · IMU 0 Hz` |

## 参数（`config/cuvslam.yaml`，50 个）

### 图像流
`width` / `height` / `fps`（默认 640×480×30）

### 标定来源
- `auto_calibration: true` → 用 RealSense 出厂标定（内参 + 双目外参，推荐）
- `auto_calibration: false` → 用下面的 `cam0_*` / `cam1_*`（自己 Kalibr 标定后填入）

### 相机（每个 6 项 + 畸变）
`cam{0,1}_fx / fy / cx / cy / distortion_model / distortion`
畸变模型：`pinhole`(0 参数) | `brown`(4-5) | `fisheye`(4) | `polynomial`
`cam1_rig_from_camera`: `[qx,qy,qz,qw,tx,ty,tz]` —— **相机1 在 rig（=相机0）中的位姿**，
对应 RealSense 的 `ir2.get_extrinsics_to(ir1)`（这里最容易搞反，写错会直接发散）。

### IMU
| 参数 | 默认 | 含义 |
|---|---|---|
| `imu_rig_from_imu` | identity | IMU 在 rig 中的位姿（Kalibr 标定后填） |
| `imu_gyro_noise_density` | 2.4e-4 | rad/s/√Hz |
| `imu_gyro_random_walk` | 3.0e-6 | rad/s²/√Hz |
| `imu_accel_noise_density` | 8.0e-4 | m/s²/√Hz |
| `imu_accel_random_walk` | 1.5e-5 | m/s³/√Hz |
| `imu_frequency` | 400.0 | Hz |

### 里程计
`odometry_mode`（`inertial` / `stereo` / `mono` / `rgbd` / `multisensor`）、
`imu_decimate`、`rectified_stereo_camera`、`async_sba`、`use_denoising`、
`use_motion_model`、`use_gpu`、`max_frame_delta_s`

### SLAM
`use_slam`、`slam_sync_mode`、`slam_use_gpu`、`slam_map_cache_path`（非空则退出时 `save_map`）、
`slam_map_cell_size`、`slam_max_landmarks_distance`、`slam_max_map_size`、
`slam_planar_constraints`、`slam_retention_time_ms`、`slam_throttling_time_ms`

### 运行
`warmup_frames`、`rebase_origin`、`jump_warn`、`run_seconds`、`snapshot_path`、
`publish_image`、`publish_camera_info`、`frame_id`、`body_frame`

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/tf` | TF | `map → base_link`（ROS 约定；默认以预热后首帧为原点） |
| `/tf_static` | TF | `base_link → camera_infra1/2` |
| `/cuvslam/path` | nav_msgs/Path | 轨迹（≤5000 点） |
| `/cuvslam/pose` | PoseStamped | 单帧位姿 |
| `/cuvslam/landmarks` | PointCloud2 | 地图点（`use_slam:=true` 时才有） |
| `/camera/infra1/image_raw` | Image | `publish_image:=true` |
| `/camera/infra1/camera_info` | CameraInfo | pinhole、无畸变（IR 已硬件校正） |

## 已知限制

1. **IMU 约 30 Hz**：四流 pipeline 每个 frameset 只带 1 条 gyro。手动 sensor API 可拿
   stereo 29.7 FPS + accel 202 Hz + gyro 398 Hz（已验证），但喂满 400 Hz 时 cuVSLAM 的
   `track` 会从 5 ms 涨到 27 ms，故暂用 30 Hz。
2. **`imu_rig_from_imu` 默认是单位阵**：D435i 出厂没有 IMU-相机外参，必须 Kalibr 标定。
3. `env.sh` 里写死了 `~/ros2_ws/build/cuvslam_bridge`（`--symlink-install` 路径），换
   workspace 要同步改。
