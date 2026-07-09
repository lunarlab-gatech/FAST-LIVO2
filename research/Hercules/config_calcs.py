from getpass import getuser
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
    sequence = "V2.4.C"
    robot_name = "Husky1"

    user = getuser()
    settings_path = Path("/home") / user / "data" / "Hercules_datasets" / sequence / "data" / "settings.json"

    # Calculate IMU -> LiDAR
    H_I_to_L_in_NED = TransformationData.from_HERCULES_settings_json(str(settings_path), robot_name, "Sensor", "LidarSensor1")
    #H_I_to_L_in_FLU = H_I_to_L_in_NED.to_coordinate_frame(CoordinateFrame.FLU, TransformType.CHANGE_OF_BASIS)
    # print("Pose of LiDAR wrt IMU: ", H_I_to_L_in_FLU.as_matrix())

    # Calculate IMU -> Camera Left Optical Frame
    H_I_to_CL_in_NED = TransformationData.from_HERCULES_settings_json(str(settings_path), robot_name, "Camera", "stereo_left")
    #H_I_to_CL_in_FLU = H_I_to_CL_in_NED.to_coordinate_frame(CoordinateFrame.FLU, TransformType.CHANGE_OF_BASIS)
    H_CL_to_CLO_in_NED = TransformationData.optical_wrt_camera(CoordinateFrame.NED, frame_id="stereo_left", child_frame_id="stereo_left_optical")
    H_I_to_CLO_in_NED = H_I_to_CL_in_NED.apply_transformation_right_side(H_CL_to_CLO_in_NED)

    # Calculate Camera Left Optical Frame -> LiDAR
    H_CLO_to_L_in_NED = H_I_to_CLO_in_NED.invert().apply_transformation_right_side(H_I_to_L_in_NED)
    # print("Pose of LiDAR wrt Left Camera Optical Frame: ", H_CLO_to_L_in_FLU.as_matrix())

    # Visualize the transformations
    TransformationData.visualize([H_I_to_L_in_NED, H_I_to_CLO_in_NED], 0.3)

    # Extract R and T from each 4x4 homogeneous matrix
    M_imu_lidar = H_I_to_L_in_NED.as_matrix()
    R_imu_lidar = M_imu_lidar[:3, :3]
    T_imu_lidar = M_imu_lidar[:3, 3]

    M_cam_lidar = H_CLO_to_L_in_NED.as_matrix()
    R_cam_lidar = M_cam_lidar[:3, :3]
    T_cam_lidar = M_cam_lidar[:3, 3]

    print("\nextrin_calib:")
    print(f"  extrinsic_T: {fmt_T(T_imu_lidar)}")
    print(f"  extrinsic_R: {fmt_R_flat(R_imu_lidar)}")
    print(f"  Rcl: {fmt_R_multiline(R_cam_lidar)}")
    print(f"  Pcl: {fmt_T(T_cam_lidar)}")


if __name__ == "__main__":
    main()
