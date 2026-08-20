import re
from getpass import getuser
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray
from robotdataprocess import TransformationData, CoordinateFrame, TransformType


SEQUENCES: List[str] = ["V2.4.C", "V2.4.F"]
ROBOT_NAMES: List[str] = ["Husky1", "Husky2", "Drone1", "Drone2"]
MATCH_TOLERANCE: float = 1e-12

Extrinsics = Dict[str, NDArray[np.float64]]
ResultKey = Tuple[str, str]  # (sequence, robot_name)

def fmt_T(t: NDArray[np.float64]) -> str:
    """Format a 3-vector as a YAML inline list."""
    return "[" + ", ".join(f"{v:.5f}" for v in t) + "]"


def fmt_R_flat(R: NDArray[np.float64]) -> str:
    """Format a 3x3 rotation matrix as a flat YAML inline list (row-major)."""
    vals = R.flatten()
    return "[" + ", ".join(f"{v:.5f}" for v in vals) + "]"


def fmt_R_multiline(R: NDArray[np.float64], indent: int = 8) -> str:
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


def robot_prefix(robot_name: str) -> str:
    """Strip the trailing instance number off a robot name, e.g. 'Husky1' -> 'Husky'."""
    return re.sub(r"\d+$", "", robot_name)


def compute_extrinsics(sequence: str, robot_name: str) -> Extrinsics:
    """Compute the IMU->LiDAR and Camera-Left-Optical->LiDAR extrinsics for one (sequence, robot) pair."""
    user = getuser()
    settings_path = Path("/home") / user / "data" / "Hercules_datasets" / sequence / "data" / "settings.json"

    # Calculate IMU -> LiDAR
    H_I_to_L_in_NED = TransformationData.from_HERCULES_settings_json(str(settings_path), robot_name, "Sensor", "LidarSensor1")

    # Calculate IMU -> Camera Left Optical Frame
    H_I_to_CL_in_NED = TransformationData.from_HERCULES_settings_json(str(settings_path), robot_name, "Camera", "stereo_left")
    H_CL_to_CLO_in_NED = TransformationData.optical_wrt_camera(CoordinateFrame.NED, frame_id="stereo_left", child_frame_id="stereo_left_optical")
    H_I_to_CLO_in_NED = H_I_to_CL_in_NED.apply_transformation_right_side(H_CL_to_CLO_in_NED)

    # Calculate Camera Left Optical Frame -> LiDAR
    H_CLO_to_L_in_NED = H_I_to_CLO_in_NED.invert().apply_transformation_right_side(H_I_to_L_in_NED)

    # Extract R and T from each 4x4 homogeneous matrix
    M_imu_lidar = H_I_to_L_in_NED.as_matrix()
    M_cam_lidar = H_CLO_to_L_in_NED.as_matrix()

    return {
        "R_imu_lidar": M_imu_lidar[:3, :3],
        "T_imu_lidar": M_imu_lidar[:3, 3],
        "R_cam_lidar": M_cam_lidar[:3, :3],
        "T_cam_lidar": M_cam_lidar[:3, 3],
    }


def print_extrin_calib(sequence: str, robot_name: str, extrinsics: Extrinsics) -> None:
    print(f"\n# {robot_name} ({sequence})")
    print("extrin_calib:")
    print(f"  extrinsic_T: {fmt_T(extrinsics['T_imu_lidar'])}")
    print(f"  extrinsic_R: {fmt_R_flat(extrinsics['R_imu_lidar'])}")
    print(f"  Rcl: {fmt_R_multiline(extrinsics['R_cam_lidar'])}")
    print(f"  Pcl: {fmt_T(extrinsics['T_cam_lidar'])}")


def extrinsics_match(a: Extrinsics, b: Extrinsics, tol: float = MATCH_TOLERANCE) -> bool:
    return (
        np.allclose(a["R_imu_lidar"], b["R_imu_lidar"], atol=tol)
        and np.allclose(a["T_imu_lidar"], b["T_imu_lidar"], atol=tol)
        and np.allclose(a["R_cam_lidar"], b["R_cam_lidar"], atol=tol)
        and np.allclose(a["T_cam_lidar"], b["T_cam_lidar"], atol=tol)
    )


def verify_results(results: Dict[ResultKey, Extrinsics]) -> List[str]:
    """
    Checks, over the full (sequence, robot_name) -> extrinsics map:
      1. Each robot's config matches itself across every sequence.
      2. Robots sharing a name prefix (Husky1/Husky2, Drone1/Drone2) match
         each other within every sequence.

    Returns a list of human-readable mismatch descriptions (empty if all match).
    """
    failures: List[str] = []

    # 1. Same robot, across all sequences, should match.
    for robot_name in ROBOT_NAMES:
        reference_sequence = SEQUENCES[0]
        reference = results[(reference_sequence, robot_name)]
        for sequence in SEQUENCES[1:]:
            if not extrinsics_match(reference, results[(sequence, robot_name)]):
                failures.append(f"{robot_name}: {reference_sequence} != {sequence}")

    # 2. Robots with the same name prefix, within a sequence, should match.
    prefixes: Dict[str, List[str]] = {}
    for robot_name in ROBOT_NAMES:
        prefixes.setdefault(robot_prefix(robot_name), []).append(robot_name)

    for sequence in SEQUENCES:
        for names in prefixes.values():
            reference_name = names[0]
            reference = results[(sequence, reference_name)]
            for other_name in names[1:]:
                if not extrinsics_match(reference, results[(sequence, other_name)]):
                    failures.append(f"{sequence}: {reference_name} != {other_name}")

    return failures


def main() -> None:
    results: Dict[ResultKey, Extrinsics] = {}
    for sequence in SEQUENCES:
        for robot_name in ROBOT_NAMES:
            extrinsics = compute_extrinsics(sequence, robot_name)
            results[(sequence, robot_name)] = extrinsics
            print_extrin_calib(sequence, robot_name, extrinsics)

    failures = verify_results(results)
    print()
    if failures:
        print("VERIFICATION FAILED:")
        for failure in failures:
            print(f"  - {failure}")
    else:
        print("VERIFICATION PASSED: all robots match across sequences and prefixes.")


if __name__ == "__main__":
    main()
