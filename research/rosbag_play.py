import argparse
from decimal import Decimal
from getpass import getuser
from pathlib import Path
from robotdataprocess import LiDARData, ImuData, ImageDataOnDisk, OdometryData, CoordinateFrame, publish_data_ROS_multiprocess, ROSMsgLibType

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset_num", required=True)
    parser.add_argument("--robot_name", required=True)
    args = parser.parse_args()
    dataset_num = args.dataset_num
    robot_name = args.robot_name

    # Define robot name to index mapping
    robot_name_to_index = {"Husky1": 0, "Husky2": 1, "Drone1": 2, "Drone2": 3}

    # Get crop times
    if dataset_num == "V2.4.C":
        robot_crop_start_times = [Decimal('0.0'), Decimal('0.0'), Decimal('0.0'), Decimal('0.0')]
        robot_crop_end_times = [Decimal('382.85'), Decimal('390.90'), Decimal('1100.00'), Decimal('1190.35')]
    elif dataset_num == "V2.3.AP":
        robot_crop_start_times = [Decimal('0.0'), Decimal('0.0'), Decimal('0.0'), Decimal('0.0')]
        robot_crop_end_times = [Decimal('772.15'), Decimal('741.45'), Decimal('1121.80'), Decimal('1193.80')]
    elif dataset_num == "V2.3.AC":
        robot_crop_start_times = [Decimal('0.0'), Decimal('0.0'), Decimal('0.0'), Decimal('0.0')]
        robot_crop_end_times = [Decimal('1125.00'), Decimal('1118.80'), Decimal('1025.50'), Decimal('892.60')]
    elif dataset_num == "V2.4.F":
        robot_crop_start_times = [Decimal('35.05'), Decimal('34.60'), Decimal('27.45'), Decimal('31.50')]
        robot_crop_end_times = [Decimal('575.55'), Decimal('762.35'), Decimal('898.10'), Decimal('906.85')]
    else:
        raise ValueError("Crop times not specified for this dataset number.")
    crop_start = robot_crop_start_times[robot_name_to_index[robot_name]]
    crop_end = robot_crop_end_times[robot_name_to_index[robot_name]]

    # Calculate dataset directory
    user = getuser()
    input_path = Path("/home") / user / "data" / "Hercules_datasets" / dataset_num / "data" / robot_name

    # Load the data to publish
    imu_data = ImuData.from_txt(input_path / "synthetic_imu_9axis_500Hz.txt", "base_link", CoordinateFrame.NED, nine_axis=True)
    lidar_data = LiDARData.from_npy_files(input_path / "lidar", "lidar_link", CoordinateFrame.NED)
    rgb_data = ImageDataOnDisk.from_image_files(input_path / "rgb_stereo_left", "left_camera_optical_link")
    gt_odom_data = OdometryData.from_txt(input_path / "pose_world_frame.txt", 'camera_init', 'base_link', CoordinateFrame.NED, False)

    # Prepare LiDAR data
    lidar_data.calculate_point_channels(16, -20, 20)
    lidar_data.make_dense()

    # Shift GT data to start at Identity to be roughly clsoe to odometry output
    gt_odom_data.shift_to_start_at_identity()

    # Crop the data
    imu_data.crop_data(crop_start, crop_end)
    lidar_data.crop_data(crop_start, crop_end)
    rgb_data.crop_data(crop_start, crop_end)
    gt_odom_data.crop_data(crop_start, crop_end)

    # Publish the data over ROS
    publish_data_ROS_multiprocess([imu_data, lidar_data, rgb_data, gt_odom_data],
                                  ["/imu_raw", "/points_raw", "/cam0/image_raw", "/odom/gt"],
                                  [None, None, None, "Path"],
                                  [500, 20, 20, 20],
                                  [1, 4, 3, 1],
                                  ROSMsgLibType.ROSPY, True, verbose=True)

if __name__ == "__main__":
    main()