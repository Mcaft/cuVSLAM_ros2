"""ROS 2 node: cuVSLAM visual(-inertial) odometry / SLAM on an Intel RealSense D435i.

Every calibration / SLAM knob is a ROS 2 parameter, so a YAML file fully describes
a rig. See config/cuvslam.yaml for the annotated defaults.

  ros2 launch cuvslam_bridge cuvslam.launch.py config_file:=/path/to/rig.yaml

Publishes:
  /tf  /tf_static  /cuvslam/{pose,path,landmarks}  /camera/infra1/{image_raw,camera_info}
"""
import statistics
import threading
import time

import numpy as np
import pyrealsense2 as rs
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

import cuvslam

R_CV2ROS = np.array([[0.0, 0.0, 1.0],
                     [-1.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0]])

DISTORTION_MODELS = {
    "pinhole": cuvslam.Distortion.Model.Pinhole,
    "brown": cuvslam.Distortion.Model.Brown,
    "fisheye": cuvslam.Distortion.Model.Fisheye,
    "polynomial": cuvslam.Distortion.Model.Polynomial,
}


def quat_from_R(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return [float((R[2, 1] - R[1, 2]) / s), float((R[0, 2] - R[2, 0]) / s),
                float((R[1, 0] - R[0, 1]) / s), float(0.25 * s)]
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [float(0.25 * s), float((R[0, 1] + R[1, 0]) / s),
                float((R[0, 2] + R[2, 0]) / s), float((R[2, 1] - R[1, 2]) / s)]
    if R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [float((R[0, 1] + R[1, 0]) / s), float(0.25 * s),
                float((R[1, 2] + R[2, 1]) / s), float((R[0, 2] - R[2, 0]) / s)]
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [float((R[0, 2] + R[2, 0]) / s), float((R[1, 2] + R[2, 1]) / s),
            float(0.25 * s), float((R[1, 0] - R[0, 1]) / s)]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class CuvslamBridge(Node):
    # ---------------------------------------------------------------- parameters
    P = {
        # stream
        "width": 640, "height": 480, "fps": 30,
        # calibration source
        "auto_calibration": True,
        # camera 0 (left IR)
        "cam0_fx": 386.4, "cam0_fy": 386.4, "cam0_cx": 319.5, "cam0_cy": 239.5,
        "cam0_distortion_model": "pinhole", "cam0_distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
        # camera 1 (right IR): 7 values = qx,qy,qz,qw,tx,ty,tz (rig_from_camera)
        "cam1_fx": 386.4, "cam1_fy": 386.4, "cam1_cx": 319.5, "cam1_cy": 239.5,
        "cam1_distortion_model": "pinhole", "cam1_distortion": [0.0, 0.0, 0.0, 0.0, 0.0],
        "cam1_rig_from_camera": [0.0, 0.0, 0.0, 1.0, 0.04993, 0.0, 0.0],
        # IMU
        "imu_rig_from_imu": [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        "imu_gyro_noise_density": 2.4e-4,
        "imu_gyro_random_walk": 3.0e-6,
        "imu_accel_noise_density": 8.0e-4,
        "imu_accel_random_walk": 1.5e-5,
        "imu_frequency": 400.0,
        # odometry
        "odometry_mode": "inertial",       # inertial | stereo | mono | rgbd | multisensor
        "imu_decimate": 1,
        "rectified_stereo_camera": True,
        "async_sba": False,
        "use_denoising": False,
        "use_motion_model": True,
        "use_gpu": True,
        "max_frame_delta_s": 1.0,
        # SLAM
        "use_slam": False,
        "slam_sync_mode": True,
        "slam_use_gpu": True,
        "slam_map_cache_path": "",
        "slam_map_cell_size": 0.5,
        "slam_max_landmarks_distance": 50.0,
        "slam_max_map_size": 300,
        "slam_planar_constraints": False,
        "slam_retention_time_ms": 0,
        "slam_throttling_time_ms": 0,
        # runtime
        "warmup_frames": 30,
        "rebase_origin": True,
        "jump_warn": 0.3,
        "run_seconds": 0.0,
        "snapshot_path": "",
        "publish_image": False,
        "publish_camera_info": True,
        "frame_id": "map",
        "body_frame": "base_link",
    }

    def __init__(self):
        super().__init__("cuvslam_bridge")
        cfg = {}
        for k, default in self.P.items():
            self.declare_parameter(k, default)
            cfg[k] = self.get_parameter(k).value
        self.cfg = cfg

        self.width = int(cfg["width"])
        self.height = int(cfg["height"])
        self.fps = int(cfg["fps"])
        self.mode = str(cfg["odometry_mode"]).lower()
        self.use_slam = bool(cfg["use_slam"])
        self.frame_id = cfg["frame_id"]
        self.body = cfg["body_frame"]

        self.tf = tf2_ros.TransformBroadcaster(self)
        self.static_tf = tf2_ros.StaticTransformBroadcaster(self)
        self.path_pub = self.create_publisher(Path, "/cuvslam/path", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/cuvslam/pose", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/cuvslam/landmarks", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/infra1/camera_info", 10)
        self.img_pub = (self.create_publisher(Image, "/camera/infra1/image_raw", 10)
                        if cfg["publish_image"] else None)

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.traj = []
        self.cloud_pts = None
        self.first_R = self.first_t = None
        self.last_t = None
        self.n_jumps = 0
        self.intr1 = None
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.imu_q = __import__("collections").deque(maxlen=5000)

    # ---------------------------------------------------------------- rig
    def _cam(self, cam_cfg, idx):
        pinhole = DISTORTION_MODELS.get(str(cam_cfg[f"cam{idx}_distortion_model"]).lower(),
                                        cuvslam.Distortion.Model.Pinhole)
        params = list(cam_cfg[f"cam{idx}_distortion"])
        if pinhole is cuvslam.Distortion.Model.Pinhole:
            params = []
        c = cuvslam.Camera()
        c.size = (int(cam_cfg[f"cam{idx}_width"]), int(cam_cfg[f"cam{idx}_height"]))
        c.focal = (float(cam_cfg[f"cam{idx}_fx"]), float(cam_cfg[f"cam{idx}_fy"]))
        c.principal = (float(cam_cfg[f"cam{idx}_cx"]), float(cam_cfg[f"cam{idx}_cy"]))
        c.distortion = cuvslam.Distortion(pinhole, params)
        pose = list(cam_cfg.get(f"cam{idx}_rig_from_camera",
                                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
        c.rig_from_camera = cuvslam.Pose(rotation=pose[:4], translation=pose[4:])
        return c

    def _build_rig(self, cfg, prof):
        ir1 = prof.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        ir2 = prof.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        i1, i2 = ir1.get_intrinsics(), ir2.get_intrinsics()
        self.intr1 = i1

        if cfg["auto_calibration"]:
            cam0 = dict(cam0_fx=i1.fx, cam0_fy=i1.fy, cam0_cx=i1.ppx, cam0_cy=i1.ppy,
                        cam0_width=i1.width, cam0_height=i1.height,
                        cam0_distortion_model="pinhole", cam0_distortion=[])
            cam1 = dict(cam1_fx=i2.fx, cam1_fy=i2.fy, cam1_cx=i2.ppx, cam1_cy=i2.ppy,
                        cam1_width=i2.width, cam1_height=i2.height,
                        cam1_distortion_model="pinhole", cam1_distortion=[])
            # pose of cam2 in the rig (=cam1) frame: extrinsics *from* cam2 *to* cam1
            ex = ir2.get_extrinsics_to(ir1)
            cam1["cam1_rig_from_camera"] = (quat_from_R(np.array(ex.rotation).reshape(3, 3))
                                            + list(ex.translation))
            self.get_logger().info(
                f"auto calibration: fx={i1.fx:.2f} baseline={abs(ex.translation[0])*1000:.2f} mm")
        else:
            cam0 = {k: cfg[k] for k in cfg if k.startswith("cam0_")}
            cam0["cam0_width"], cam0["cam0_height"] = self.width, self.height
            cam1 = {k: cfg[k] for k in cfg if k.startswith("cam1_")}
            cam1["cam1_width"], cam1["cam1_height"] = self.width, self.height
            self.get_logger().info(
                f"yaml calibration: fx={cam0['cam0_fx']:.2f} "
                f"baseline={cam1['cam1_rig_from_camera'][4]*1000:.2f} mm")

        rig = cuvslam.Rig()
        rig.cameras = [self._cam(cam0, 0),
                       self._cam(cam1, 1)]
        if self.mode == "inertial":
            cal = cuvslam.ImuCalibration()
            cal.gyroscope_noise_density = float(cfg["imu_gyro_noise_density"])
            cal.gyroscope_random_walk = float(cfg["imu_gyro_random_walk"])
            cal.accelerometer_noise_density = float(cfg["imu_accel_noise_density"])
            cal.accelerometer_random_walk = float(cfg["imu_accel_random_walk"])
            cal.frequency = float(cfg["imu_frequency"])
            pose = list(cfg["imu_rig_from_imu"])
            cal.rig_from_imu = cuvslam.Pose(rotation=pose[:4], translation=pose[4:])
            rig.imus = [cal]
        return rig

    def _odometry_config(self, cfg):
        modes = {"inertial": cuvslam.Tracker.OdometryMode.Inertial,
                 "stereo": cuvslam.Tracker.OdometryMode.Multicamera,
                 "mono": cuvslam.Tracker.OdometryMode.Mono,
                 "rgbd": cuvslam.Tracker.OdometryMode.RGBD,
                 "multisensor": cuvslam.Tracker.OdometryMode.Multisensor}
        if self.mode not in modes:
            self.get_logger().warn(f"unknown odometry_mode '{self.mode}', falling back to inertial")
            self.mode = "inertial"
        return cuvslam.Tracker.OdometryConfig(
            odometry_mode=modes[self.mode],
            rectified_stereo_camera=bool(cfg["rectified_stereo_camera"]),
            async_sba=bool(cfg["async_sba"]),
            use_denoising=bool(cfg["use_denoising"]),
            use_motion_model=bool(cfg["use_motion_model"]),
            use_gpu=bool(cfg["use_gpu"]),
            max_frame_delta_s=float(cfg["max_frame_delta_s"]),
        )

    def _slam_config(self, cfg):
        s = cuvslam.Tracker.SlamConfig()
        s.sync_mode = bool(cfg["slam_sync_mode"])
        s.use_gpu = bool(cfg["slam_use_gpu"])
        s.map_cell_size = float(cfg["slam_map_cell_size"])
        s.max_landmarks_distance = float(cfg["slam_max_landmarks_distance"])
        s.max_map_size = int(cfg["slam_max_map_size"])
        s.planar_constraints = bool(cfg["slam_planar_constraints"])
        s.retention_time_ms = int(cfg["slam_retention_time_ms"])
        s.throttling_time_ms = int(cfg["slam_throttling_time_ms"])
        if cfg["slam_map_cache_path"]:
            s.map_cache_path = cfg["slam_map_cache_path"]
        return s

    # ---------------------------------------------------------------- publishing
    def _camera_info(self, stamp):
        i = self.intr1
        ci = CameraInfo()
        ci.header = Header(stamp=stamp, frame_id="camera_infra1")
        ci.width, ci.height = self.width, self.height
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        ci.k = [i.fx, 0.0, i.ppx, 0.0, i.fy, i.ppy, 0.0, 0.0, 1.0]
        ci.p = [i.fx, 0.0, i.ppx, 0.0, 0.0, i.fy, i.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _publish(self, stamp, Rc, tc, lms, left):
        Rr = R_CV2ROS @ Rc @ R_CV2ROS.T
        qr = quat_from_R(Rr)
        t = R_CV2ROS @ tc

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.body
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = t
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qr
        self.tf.sendTransform(tf)

        ps = PoseStamped()
        ps.header = Header(stamp=stamp, frame_id=self.frame_id)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = t
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qr
        self.pose_pub.publish(ps)

        self.path.header.stamp = stamp
        self.path.poses.append(ps)
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]
        self.path_pub.publish(self.path)

        if lms:
            pts = (R_CV2ROS @ np.asarray(lms, dtype=np.float32).reshape(-1, 3).T).T
            msg = PointCloud2()
            msg.header = Header(stamp=stamp, frame_id=self.frame_id)
            msg.height, msg.width = 1, pts.shape[0]
            msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                          for i, n in enumerate(("x", "y", "z"))]
            msg.is_bigendian = False
            msg.point_step, msg.row_step, msg.is_dense = 12, 12 * pts.shape[0], True
            msg.data = pts.astype(np.float32).tobytes()
            self.cloud_pub.publish(msg)

        if self.img_pub is not None and left is not None:
            im = Image()
            im.header = Header(stamp=stamp, frame_id="camera_infra1")
            im.height, im.width = left.shape
            im.encoding, im.step = "mono8", left.shape[1]
            im.data = left.tobytes()
            self.img_pub.publish(im)

    # ---------------------------------------------------------------- loop
    def start(self):
        self.worker.start()

    def _run(self):
        cfg = self.cfg
        cfg_rs = rs.config()
        cfg_rs.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
        cfg_rs.enable_stream(rs.stream.infrared, 2, self.width, self.height, rs.format.y8, self.fps)
        if self.mode == "inertial":
            cfg_rs.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
            cfg_rs.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
        self.pipe = rs.pipeline()
        prof = self.pipe.start(cfg_rs)

        slam_cfg = self._slam_config(cfg) if self.use_slam else None
        self.tracker = cuvslam.Tracker(self._build_rig(cfg, prof),
                                       self._odometry_config(cfg), slam_cfg)

        st = []
        for name in ("camera_infra1", "camera_infra2"):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.body
            t.child_frame_id = name
            t.transform.rotation.w = 1.0
            st.append(t)
        self.static_tf.sendTransform(st)
        self.get_logger().info(
            f"cuVSLAM {cuvslam.get_version()} | mode={self.mode} | slam={self.use_slam} | "
            f"decimate={cfg['imu_decimate']} | warmup={cfg['warmup_frames']} | "
            f"rebase={cfg['rebase_origin']}")

        lat = []
        n_imu = n_frames = 0
        last_accel = [0.0, 0.0, 0.0]
        last_imu_ts = 0
        t0 = time.perf_counter()
        last_report = t0
        pos = [0.0, 0.0, 0.0]
        decimate = max(1, int(cfg["imu_decimate"]))
        warmup = int(cfg["warmup_frames"])
        rebase = bool(cfg["rebase_origin"])
        jump_warn = float(cfg["jump_warn"])
        run_seconds = float(cfg["run_seconds"])
        k_imu = 0

        while not self.stop_event.is_set():
            if run_seconds > 0 and time.perf_counter() - t0 > run_seconds:
                break
            try:
                fs = self.pipe.wait_for_frames(2000)
            except Exception:
                continue
            motions = [f for f in fs if f.is_motion_frame()]
            motions.sort(key=lambda f: f.get_timestamp())
            L = R = None
            img_ts = None
            if fs.get_infrared_frame(1) is not None:
                L = np.asanyarray(fs.get_infrared_frame(1).get_data())
                R = np.asanyarray(fs.get_infrared_frame(2).get_data())
                img_ts = int(fs.get_timestamp() * 1e6)

            for m in motions:
                md = m.as_motion_frame().get_motion_data()
                ts = int(m.get_timestamp() * 1e6)
                if m.get_profile().stream_type() == rs.stream.accel:
                    last_accel = [md.x, md.y, md.z]
                else:
                    self.imu_q.append((ts, [md.x, md.y, md.z], list(last_accel)))
            while self.imu_q and (img_ts is None or self.imu_q[0][0] < img_ts):
                ts, gyro, acc = self.imu_q.popleft()
                k_imu += 1
                if ts <= last_imu_ts or (k_imu % decimate):
                    continue
                meas = cuvslam.ImuMeasurement()
                meas.timestamp_ns = ts
                meas.angular_velocities = gyro
                meas.linear_accelerations = acc
                self.tracker.register_imu_measurement(0, meas)
                last_imu_ts = ts
                n_imu += 1

            if L is None or img_ts is None:
                continue
            t1 = time.perf_counter()
            est, slam_pose = self.tracker.track(img_ts, [L, R])
            lat.append((time.perf_counter() - t1) * 1000)
            n_frames += 1
            stamp = self.get_clock().now().to_msg()

            pose_src = slam_pose if (self.use_slam and slam_pose is not None) else (
                est.world_from_rig.pose if est.world_from_rig else None)
            if pose_src is not None:
                Rc = quat_to_R(pose_src.rotation)
                tc = np.asarray(pose_src.translation, dtype=float)
                if rebase and self.first_R is None and n_frames >= warmup:
                    self.first_R, self.first_t = Rc, tc
                if rebase and self.first_R is None:
                    continue
                if self.first_R is not None:
                    Rc = self.first_R.T @ Rc
                    tc = self.first_R.T @ (tc - self.first_t)
                if self.last_t is not None:
                    d = float(np.linalg.norm(tc - self.last_t))
                    if d > jump_warn:
                        self.n_jumps += 1
                        self.get_logger().warn(
                            f"pose jump {d:.2f} m (frame {n_frames}, total {self.n_jumps})")
                self.last_t = tc
                pos = list(tc)
                lms = None
                try:
                    lms = [list(l.position) for l in self.tracker.get_last_landmarks()]
                except Exception:
                    lms = None
                self._publish(stamp, Rc, tc, lms, L if cfg["publish_image"] else None)
                self.traj.append(tuple(tc))
                if lms:
                    self.cloud_pts = np.asarray(lms, dtype=np.float64)

            if cfg["publish_camera_info"]:
                self.info_pub.publish(self._camera_info(stamp))

            now = time.perf_counter()
            if now - last_report >= 1.0:
                self.get_logger().info(
                    f"{n_frames/(now-t0):5.1f} FPS | track {statistics.mean(lat[-30:]):5.2f} ms | "
                    f"IMU {n_imu/(now-t0):5.1f} Hz | pos ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
                    f"| poses {len(self.traj)} jumps {self.n_jumps}")
                last_report = now

        try:
            if self.use_slam and cfg["slam_map_cache_path"]:
                self.tracker.save_map(cfg["slam_map_cache_path"])
                self.get_logger().info(f"map saved: {cfg['slam_map_cache_path']}")
        except Exception as e:
            self.get_logger().error(f"save_map failed: {e}")
        try:
            self.pipe.stop()
        except Exception:
            pass
        if cfg["snapshot_path"] and self.traj:
            try:
                self._save_plot(cfg["snapshot_path"])
            except Exception as e:
                self.get_logger().error(f"snapshot failed: {e}")
        self.get_logger().info("cuvslam_bridge stopped")

    def _save_plot(self, path):
        import cv2
        tr = np.asarray(self.traj, dtype=np.float32)
        cp = None
        if self.cloud_pts is not None and len(self.cloud_pts):
            cp = (R_CV2ROS @ np.asarray(self.cloud_pts, dtype=np.float32).reshape(-1, 3).T).T
        pts = tr[:, :2] if cp is None else np.vstack([tr[:, :2], cp[:, :2]])
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        span = max(float((hi - lo).max()), 0.2)
        S, pad = 820, 40
        scale = S / span

        def px(x, y):
            return (int(pad + (x - lo[0]) * scale), int(pad + S - (y - lo[1]) * scale))

        img = np.full((S + 2 * pad, S + 2 * pad, 3), 24, np.uint8)
        for g in range(0, S + 1, 100):
            cv2.line(img, (pad + g, pad), (pad + g, pad + S), (44, 44, 48), 1)
            cv2.line(img, (pad, pad + g), (pad + S, pad + g), (44, 44, 48), 1)
        if cp is not None:
            for p in cp:
                cv2.circle(img, px(p[0], p[1]), 2, (90, 170, 255), -1)
        for a, b in zip(tr[:-1], tr[1:]):
            cv2.line(img, px(a[0], a[1]), px(b[0], b[1]), (150, 220, 40), 2)
        cv2.circle(img, px(tr[0, 0], tr[0, 1]), 7, (255, 255, 255), -1)
        cv2.circle(img, px(tr[-1, 0], tr[-1, 1]), 7, (60, 60, 255), -1)
        cv2.putText(img, f"cuVSLAM {self.mode} slam={self.use_slam}: {len(tr)} poses", (pad, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1)
        cv2.putText(img, f"grid = {100.0/scale:.3f} m", (pad, img.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        cv2.imwrite(path, img)


def main(args=None):
    rclpy.init(args=args)
    node = CuvslamBridge()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
