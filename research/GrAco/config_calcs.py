from getpass import getuser
import numpy as np
from pathlib import Path
from robotdataprocess import TransformationData, CoordinateFrame, TransformType

def fmt_T(t):
    """Format a 3-vector as a YAML inline list."""
    return "[" + ", ".join(f"{v:.5f}" for v in t) + "]"


def fmt_R_flat(R):
    """Format a 3x3 rotation matrix as a flat YAML inline list (row-major)."""
    vals = R.flatten()
    return "[" + ", ".join(f"{v:.5f}" for v in vals) + "]"


def fmt_R_multiline(R, indent=8):
    """Format a 3x3 rotation matrix as a 3-line YAML list, one row per line."""
    pad = " " * indent
    rows = []
    for i, row in enumerate(R):
        vals = ", ".join(f"{v:.5f}" for v in row)
        if i == 0:
            rows.append(f"[{vals},")
        elif i < 2:
            rows.append(f"{pad} {vals},")
        else:
            rows.append(f"{pad} {vals}]")
    return "\n".join(rows)

def main():
    sequence = "V1.0"
    robot_type = "aerial"
    settings_path = Path("/home") / getuser()/ "data" / "GrAco_dataset" / sequence / "data" / (robot_type + "-calibration")

    # Load IMU -> LiDAR
    H_I_to_L_in_ENU = TransformationData.from_GrAco_yaml(str(settings_path / 'imu-lidar.yaml'), 'T_Imu_Lidar')

    # Load IMU -> Camera Left Optical Frame
    H_I_to_CLO_in_ENU = TransformationData.from_GrAco_yaml(str(settings_path / 'stereo-imu.yaml'), 'T_Imu_cam0')

    # Calculate Camera Left Optical Frame -> LiDAR and assert it matches config files
    H_CLO_to_L_in_ENU_hat = H_I_to_CLO_in_ENU.invert().apply_transformation_right_side(H_I_to_L_in_ENU)
    H_CLO_to_L_in_ENU_gt = TransformationData.from_GrAco_yaml(str(settings_path / 'stereo-lidar.yaml'), 'T_cam0_Lidar')
    np.testing.assert_array_almost_equal(H_CLO_to_L_in_ENU_hat.as_matrix(), H_CLO_to_L_in_ENU_gt.as_matrix(), 1)

    # Extract R and T from each 4x4 homogeneous matrix
    M_imu_lidar = H_I_to_L_in_ENU.as_matrix()
    R_imu_lidar = M_imu_lidar[:3, :3]
    T_imu_lidar = M_imu_lidar[:3, 3]

    M_cam_lidar = H_CLO_to_L_in_ENU_gt.as_matrix()
    R_cam_lidar = M_cam_lidar[:3, :3]
    T_cam_lidar = M_cam_lidar[:3, 3]

    print("\nextrin_calib:")
    print(f"  extrinsic_T: {fmt_T(T_imu_lidar)}")
    print(f"  extrinsic_R: {fmt_R_flat(R_imu_lidar)}")
    print(f"  Rcl: {fmt_R_multiline(R_cam_lidar)}")
    print(f"  Pcl: {fmt_T(T_cam_lidar)}")


if __name__ == "__main__":
    main()
