#!/usr/bin/env python3
"""Convert a Kalibr camchain-imucam.yaml into cuvslam_bridge parameters.

Kalibr gives you (per camera) intrinsics + distortion, T_cam_imu (cam0) and
T_cn_cnm1 (cam1 relative cam0). cuvslam_bridge wants the same numbers as flat
ROS 2 parameters, so this script rewrites them into a YAML the node can load.

  python3 kalibr2cuvslam.py --in camchain-imucam.yaml --out rig.yaml
  python3 kalibr2cuvslam.py --in camchain-imucam.yaml --base config/cuvslam.yaml --out rig.yaml
  python3 kalibr2cuvslam.py --in camchain-imucam.yaml --print      # dump only

Conventions
-----------
* Kalibr `T_cam_imu` maps points from IMU frame to cam0 frame:  p_cam0 = T * p_imu
  which is exactly cuVSLAM's `imu_rig_from_imu` (rig = cam0).
* Kalibr `T_cn_cnm1` maps points from camN to cam(N-1):  p_cam0 = T * p_cam1
  which is exactly `cam1_rig_from_camera`.
* Quaternions are written as [qx, qy, qz, qw] to match the node's parameter layout.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

# ---------------------------------------------------------------- yaml with !!opencv-matrix
class _Loader(yaml.SafeLoader):
    pass


def _opencv_matrix(loader, node):
    m = loader.construct_mapping(node, deep=True)
    return np.array(m["data"], dtype=float).reshape(int(m["rows"]), int(m["cols"]))


_Loader.add_constructor("tag:yaml.org,2002:opencv-matrix", _opencv_matrix)


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.load(f, Loader=_Loader)


# ---------------------------------------------------------------- math
def quat_from_R(R):
    """3x3 rotation -> quaternion (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return [float((R[2, 1] - R[1, 2]) / s), float((R[0, 2] - R[2, 0]) / s),
                float((R[1, 0] - R[0, 1]) / s), float(0.25 * s)]
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [float(0.25 * s), float((R[0, 1] + R[1, 0]) / s),
                float((R[0, 2] + R[2, 0]) / s), float((R[2, 1] - R[1, 2]) / s)]
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [float((R[0, 1] + R[1, 0]) / s), float(0.25 * s),
                float((R[1, 2] + R[2, 1]) / s), float((R[0, 2] - R[2, 0]) / s)]
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [float((R[0, 2] + R[2, 0]) / s), float((R[1, 2] + R[2, 1]) / s),
            float(0.25 * s), float((R[1, 0] - R[0, 1]) / s)]


def to_matrix(name, value):
    """Accept a 4x4 opencv-matrix node, a plain {rows,cols,data} dict, a nested list,
    or a flat 16-element list."""
    if isinstance(value, np.ndarray):
        M = value.astype(float)
    elif isinstance(value, dict):
        M = np.array(value["data"], dtype=float).reshape(int(value["rows"]), int(value["cols"]))
    elif isinstance(value, list):
        arr = np.array(value, dtype=float)
        if arr.shape == (16,):
            arr = arr.reshape(4, 4)
        elif arr.shape == (4, 4):
            pass
        else:
            raise ValueError(f"{name}: unexpected shape {arr.shape}")
        M = arr
    else:
        raise ValueError(f"{name}: unsupported type {type(value)}")
    if M.shape != (4, 4):
        raise ValueError(f"{name}: expected 4x4, got {M.shape}")
    return M


def matrix_to_qt(M):
    q = quat_from_R(M[:3, :3])
    t = [float(v) for v in M[:3, 3]]
    return q + t


# ---------------------------------------------------------------- distortion mapping
def map_distortion(kalibr_model, coeffs, cam_name, warn):
    model = str(kalibr_model or "none").lower()
    coeffs = [float(c) for c in (coeffs or [])]
    if model in ("none", "pinhole", ""):
        return "pinhole", []
    if model in ("radtan", "radial-tangential", "brown", "plumb_bob"):
        if len(coeffs) == 4:
            warn(f"{cam_name}: radtan with 4 coefficients (k1,k2,p1,p2); cuVSLAM 'brown' "
                 f"expects 4 or 5, passing them through unchanged.")
            return "brown", coeffs
        if len(coeffs) == 5:
            return "brown", coeffs
        warn(f"{cam_name}: radtan has {len(coeffs)} coefficients, expected 4 or 5; "
             f"check them manually before flying.")
        return "brown", coeffs
    if model in ("equidistant", "fisheye", "kb4"):
        return "fisheye", coeffs
    if model in ("rational", "polynomial"):
        return "polynomial", coeffs
    warn(f"{cam_name}: unknown distortion model '{model}', falling back to pinhole (no distortion).")
    return "pinhole", []


# ---------------------------------------------------------------- conversion
def convert(chain, base_params, warn, keep_distortion=False):
    params = dict(base_params)
    cams = sorted(k for k in chain if str(k).startswith("cam") and isinstance(chain[k], dict))
    if len(cams) < 2:
        raise SystemExit(f"need at least cam0 and cam1 in the camchain, found: {cams}")

    cam0, cam1 = cams[0], cams[1]
    for idx, name in enumerate((cam0, cam1)):
        c = chain[name]
        intr = c.get("intrinsics")
        if not intr or len(intr) != 4:
            raise SystemExit(f"{name}: 'intrinsics' must be [fx, fy, cx, cy]")
        fx, fy, cx, cy = [float(v) for v in intr]
        params[f"cam{idx}_fx"] = fx
        params[f"cam{idx}_fy"] = fy
        params[f"cam{idx}_cx"] = cx
        params[f"cam{idx}_cy"] = cy
        model, coeffs = map_distortion(c.get("distortion_model"), c.get("distortion_coeffs"), name, warn)
        params[f"cam{idx}_distortion_model"] = model
        params[f"cam{idx}_distortion"] = coeffs
        res = c.get("resolution")
        if res:
            params["width"], params["height"] = int(res[0]), int(res[1])

    # IMU -> cam0
    T_cam_imu = chain[cam0].get("T_cam_imu") or chain.get("T_cam_imu")
    if T_cam_imu is None:
        warn("no T_cam_imu found in cam0; imu_rig_from_imu left untouched")
    else:
        params["imu_rig_from_imu"] = matrix_to_qt(to_matrix("T_cam_imu", T_cam_imu))

    # cam1 -> cam0
    # Kalibr's T_cn_cnm1 maps cam(n-1) points into cam(n):  p_cam1 = T * p_cam0.
    # cuVSLAM wants the opposite direction (p_cam0 = R * p_cam1 + t), so invert it.
    T_rel = chain[cam1].get("T_cn_cnm1") or chain[cam1].get("T_cam0_cam1")
    if T_rel is None:
        warn(f"{cam1}: no T_cn_cnm1 / T_cam0_cam1 found; cam1_rig_from_camera left untouched")
    else:
        M = to_matrix("T_cn_cnm1", T_rel)
        params["cam1_rig_from_camera"] = matrix_to_qt(np.linalg.inv(M))

    params["auto_calibration"] = False

    # cuVSLAM refuses non-pinhole models in "rectified stereo" mode, and the D435i
    # IR stream is already undistorted, so keep pinhole unless asked otherwise.
    rectified = bool(params.get("rectified_stereo_camera", True))
    if rectified and not keep_distortion:
        for idx in (0, 1):
            if params.get(f"cam{idx}_distortion_model") != "pinhole":
                warn(f"cam{idx}: rectified_stereo_camera=true -> forcing pinhole (Kalibr "
                     f"distortion {params[f'cam{idx}_distortion']} ignored; the IR stream is "
                     f"already undistorted). Use --keep-distortion only for raw/unrectified input.")
                params[f"cam{idx}_distortion_model"] = "pinhole"
                # ROS 2 cannot infer the type of an empty list, so keep 5 zeros
                # (the node ignores them for the pinhole model).
                params[f"cam{idx}_distortion"] = [0.0, 0.0, 0.0, 0.0, 0.0]
    return params


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="src", required=True, help="Kalibr camchain-imucam.yaml")
    ap.add_argument("--out", dest="dst", default="", help="output YAML (default: rig_kalibr.yaml)")
    ap.add_argument("--base", default="", help="existing cuvslam.yaml to inherit non-calibration defaults")
    ap.add_argument("--node", default="cuvslam_bridge", help="ROS node name key in the YAML")
    ap.add_argument("--print", action="store_true", help="print the result instead of writing")
    ap.add_argument("--keep-distortion", action="store_true",
                    help="keep Kalibr distortion even in rectified mode (raw input only)")
    args = ap.parse_args()

    warnings = []

    def warn(msg):
        warnings.append(msg)
        print(f"WARN: {msg}", file=sys.stderr)

    chain = load_yaml(args.src)

    base_params = {}
    if args.base:
        base = load_yaml(args.base)
        base_params = base.get(args.node, {}).get("ros__parameters", {})

    params = convert(chain, base_params, warn, keep_distortion=args.keep_distortion)
    doc = {args.node: {"ros__parameters": params}}
    text = yaml.safe_dump(doc, sort_keys=False, default_flow_style=None, allow_unicode=True)

    print("# generated by kalibr2cuvslam.py from", os.path.basename(args.src))
    print(text)
    if args.print:
        return
    out = args.dst or "rig_kalibr.yaml"
    with open(out, "w", encoding="utf-8") as f:
        f.write("# generated by kalibr2cuvslam.py from %s\n" % os.path.basename(args.src))
        f.write(text)
    print(f"\nwritten: {out}")
    print(f"  fx={params['cam0_fx']:.2f}  cx={params['cam0_cx']:.2f}  "
          f"imu_rig_from_imu={['%.4f' % v for v in params['imu_rig_from_imu']]}")
    if warnings:
        print(f"  {len(warnings)} warning(s) above — check them before flying", file=sys.stderr)


if __name__ == "__main__":
    main()
