#!/usr/bin/env python3
"""
Isolate the IMU-only prediction vs the LiDAR ESIKF correction, per LiDAR frame,
and compare each modality's step-wise displacement against ground truth.

Data sources (no source-code changes needed):
  - Log/mat_pre.txt : state BEFORE the LiDAR correction each frame (LIVMapper.cpp:340,
    written right after processImu()'s pure-IMU-integration result is copied into
    state_propagat). This is the IMU-only prediction for that frame.
  - Log/mat_out.txt : state AFTER voxelmap_manager->StateEstimation() (the ESIKF
    LiDAR correction) has run (LIVMapper.cpp:480). This is the fused result.
  - /gnss/ground_truth in the rosbag: independent ground truth trajectory.

Column layout (confirmed against Log/plot.py, 0-indexed):
  0: time (relative to _first_lidar_time)
  1-3: roll, pitch, yaw (deg)
  4-6: pos x,y,z   (world / camera_init frame)
  7-9: vel x,y,z
  10-12: bias_g xyz
  13-15: bias_a xyz
  16: inv_expo_time
  (mat_out.txt has one extra trailing column: feats_undistort point count)

Since camera_init's horizontal orientation/origin is arbitrary relative to GT's
ENU frame, we compare *step-wise displacement magnitudes* (frame/origin-invariant)
rather than raw per-axis coordinates.

Run with: source /opt/ros/noetic/setup.bash && python3 compare_modalities.py
"""
import os
import numpy as np
import rosbag

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MAT_PRE = "/home/dbutterfield3/fast_livo2_ws/src/FAST-LIVO2/Log/mat_pre.txt"
MAT_OUT = "/home/dbutterfield3/fast_livo2_ws/src/FAST-LIVO2/Log/mat_out.txt"
BAG = "/home/dbutterfield3/data/GrAco_dataset/V1.0/data/ground-01/ground-01.bag"
GT_TOPIC = "/gnss/ground_truth"
LIDAR_TOPIC = "/velodyne/points"


def load_gt(bag_path):
    times, xs, ys, zs = [], [], [], []
    bag = rosbag.Bag(bag_path)
    for _, msg, _ in bag.read_messages(topics=[GT_TOPIC]):
        p = msg.pose.pose.position
        times.append(msg.header.stamp.to_sec())
        xs.append(p.x); ys.append(p.y); zs.append(p.z)
    bag.close()
    return np.array(times), np.array(xs), np.array(ys), np.array(zs)


def first_lidar_stamp(bag_path):
    bag = rosbag.Bag(bag_path)
    for _, msg, _ in bag.read_messages(topics=[LIDAR_TOPIC]):
        bag.close()
        return msg.header.stamp.to_sec()
    bag.close()
    raise RuntimeError("no lidar messages found")


def gt_interp(gt_t, gt_x, gt_y, gt_z, t_query):
    x = np.interp(t_query, gt_t, gt_x)
    y = np.interp(t_query, gt_t, gt_y)
    z = np.interp(t_query, gt_t, gt_z)
    return np.stack([x, y, z], axis=-1)


def main():
    a_pre = np.loadtxt(MAT_PRE)
    a_out = np.loadtxt(MAT_OUT)

    t_pre, pos_pre, vel_pre, bias_a_pre = a_pre[:, 0], a_pre[:, 4:7], a_pre[:, 7:10], a_pre[:, 13:16]
    t_out, pos_out, vel_out, bias_a_out = a_out[:, 0], a_out[:, 4:7], a_out[:, 7:10], a_out[:, 13:16]

    # mat_pre has extra rows on frames where feats_undistort was empty and
    # handleLIO() returned before reaching the mat_out write (LIVMapper.cpp:344-348).
    # Align pre->out by matching timestamps (same relative-time value per frame).
    out_index_by_t = {round(t, 4): i for i, t in enumerate(t_out)}
    matched_pre_idx = []
    matched_out_idx = []
    for i, t in enumerate(t_pre):
        j = out_index_by_t.get(round(t, 4))
        if j is not None:
            matched_pre_idx.append(i)
            matched_out_idx.append(j)
    matched_pre_idx = np.array(matched_pre_idx)
    matched_out_idx = np.array(matched_out_idx)

    t = t_pre[matched_pre_idx]
    pos_pre_m = pos_pre[matched_pre_idx]
    pos_out_m = pos_out[matched_out_idx]
    vel_pre_m = vel_pre[matched_pre_idx]
    vel_out_m = vel_out[matched_out_idx]
    bias_a_pre_m = bias_a_pre[matched_pre_idx]
    bias_a_out_m = bias_a_out[matched_out_idx]

    print(f"mat_pre.txt rows: {len(t_pre)}, mat_out.txt rows: {len(t_out)}, matched frames: {len(t)}")
    print(f"log time span: {t.min():.2f}s to {t.max():.2f}s")

    first_lidar_t = first_lidar_stamp(BAG)
    print(f"first /velodyne/points epoch time in bag: {first_lidar_t:.3f}")

    gt_t, gt_x, gt_y, gt_z = load_gt(BAG)
    print(f"loaded {len(gt_t)} ground_truth samples, "
          f"span {gt_t.min():.3f} to {gt_t.max():.3f}")

    # Absolute time for each logged frame.
    abs_t = t + first_lidar_t

    # per-frame deltas (skip the very first row, no previous frame to diff against)
    imu_step = pos_pre_m[1:] - pos_out_m[:-1]        # IMU-only prediction step
    lidar_corr = pos_out_m[1:] - pos_pre_m[1:]        # LiDAR's correction on top of that
    fused_step = pos_out_m[1:] - pos_out_m[:-1]       # net fused step (= imu_step + lidar_corr)
    t_step_end = abs_t[1:]
    t_step_start = abs_t[:-1]

    gt_start = gt_interp(gt_t, gt_x, gt_y, gt_z, t_step_start)
    gt_end = gt_interp(gt_t, gt_x, gt_y, gt_z, t_step_end)
    gt_step = gt_end - gt_start

    imu_mag = np.linalg.norm(imu_step, axis=1)
    corr_mag = np.linalg.norm(lidar_corr, axis=1)
    fused_mag = np.linalg.norm(fused_step, axis=1)
    gt_mag = np.linalg.norm(gt_step, axis=1)

    print("\n=== Per-step displacement magnitude (meters), first 15 steps ===")
    print(f"{'t(rel)':>8} {'GT':>8} {'IMU-only':>10} {'LiDAR-corr':>12} {'Fused':>8}")
    for i in range(min(15, len(t_step_end))):
        print(f"{t[1:][i]:8.2f} {gt_mag[i]:8.4f} {imu_mag[i]:10.4f} {corr_mag[i]:12.4f} {fused_mag[i]:8.4f}")

    print("\n=== Summary over full logged window ===")
    print(f"Total GT displacement (sum of step magnitudes):     {gt_mag.sum():.3f} m")
    print(f"Total IMU-only predicted displacement (sum of mags): {imu_mag.sum():.3f} m")
    print(f"Total LiDAR correction magnitude (sum of mags):      {corr_mag.sum():.3f} m")
    print(f"Total fused displacement (sum of mags):              {fused_mag.sum():.3f} m")
    print(f"Net fused displacement (start to end, straight line): "
          f"{np.linalg.norm(pos_out_m[-1] - pos_out_m[0]):.3f} m")
    print(f"Net GT displacement (start to end, straight line):    "
          f"{np.linalg.norm(gt_end[-1] - gt_start[0]):.3f} m")

    print(f"\nMean per-step GT displacement:       {gt_mag.mean():.4f} m")
    print(f"Mean per-step IMU-only displacement: {imu_mag.mean():.4f} m")
    print(f"Mean per-step LiDAR correction:       {corr_mag.mean():.4f} m")
    print(f"Mean per-step fused displacement:     {fused_mag.mean():.4f} m")

    corr = np.corrcoef(imu_mag, gt_mag)[0, 1]
    print(f"\nCorrelation(IMU-only step size, GT step size):   {corr:.3f}")
    corr2 = np.corrcoef(fused_mag, gt_mag)[0, 1]
    print(f"Correlation(fused step size, GT step size):      {corr2:.3f}")

    # === Velocity magnitude: does the estimator's own velocity state ever
    # build up to something realistic, or does it stay pinned near zero? ===
    speed_pre = np.linalg.norm(vel_pre_m, axis=1)
    speed_out = np.linalg.norm(vel_out_m, axis=1)
    dt = np.diff(abs_t)
    gt_speed = gt_mag / dt  # local GT speed over each step interval

    print("\n=== Velocity magnitude (m/s) ===")
    print(f"GT average speed over window:         {gt_speed.mean():.4f} m/s "
          f"(min {gt_speed.min():.4f}, max {gt_speed.max():.4f})")
    print(f"||vel_end|| (pre, IMU-only) average:  {speed_pre.mean():.4f} m/s "
          f"(min {speed_pre.min():.4f}, max {speed_pre.max():.4f})")
    print(f"||vel_end|| (out, fused) average:     {speed_out.mean():.4f} m/s "
          f"(min {speed_out.min():.4f}, max {speed_out.max():.4f})")

    print(f"\n{'t(rel)':>8} {'GT speed':>10} {'vel_pre':>10} {'vel_out':>10}")
    step = max(1, len(t) // 30)
    for i in range(0, len(t), step):
        gt_s = gt_speed[i - 1] if 0 < i <= len(gt_speed) else float('nan')
        print(f"{t[i]:8.2f} {gt_s:10.4f} {speed_pre[i]:10.4f} {speed_out[i]:10.4f}")

    # === Accelerometer bias: has it grown large enough to be cancelling
    # real driving acceleration rather than just tracking sensor bias? ===
    bias_a_mag_pre = np.linalg.norm(bias_a_pre_m, axis=1)
    bias_a_mag_out = np.linalg.norm(bias_a_out_m, axis=1)
    print("\n=== |bias_a| magnitude (m/s^2) ===")
    print(f"pre: start {bias_a_mag_pre[0]:.5f}, end {bias_a_mag_pre[-1]:.5f}, "
          f"min {bias_a_mag_pre.min():.5f}, max {bias_a_mag_pre.max():.5f}")
    print(f"out: start {bias_a_mag_out[0]:.5f}, end {bias_a_mag_out[-1]:.5f}, "
          f"min {bias_a_mag_out.min():.5f}, max {bias_a_mag_out.max():.5f}")
    print(f"\n{'t(rel)':>8} {'bias_a_x':>10} {'bias_a_y':>10} {'bias_a_z':>10} {'|bias_a|':>10}")
    for i in range(0, len(t), step):
        bx, by, bz = bias_a_out_m[i]
        print(f"{t[i]:8.2f} {bx:10.5f} {by:10.5f} {bz:10.5f} {bias_a_mag_out[i]:10.5f}")

    # === How abrupt/gentle was the real acceleration right after the robot
    # started moving? Use GT at its native ~200Hz rate, not resampled at 10Hz,
    # so we can actually see the shape of the initial acceleration event. ===
    window_end = first_lidar_t + 15.0
    in_window = (gt_t >= first_lidar_t - 1.0) & (gt_t <= window_end)
    wt = gt_t[in_window]
    wx, wy, wz = gt_x[in_window], gt_y[in_window], gt_z[in_window]
    dwt = np.diff(wt)
    dwt[dwt == 0] = np.nan
    vx = np.diff(wx) / dwt
    vy = np.diff(wy) / dwt
    vz = np.diff(wz) / dwt
    gt_native_speed = np.sqrt(vx**2 + vy**2 + vz**2)
    gt_native_t = wt[1:] - first_lidar_t
    # smooth a bit (native rate is noisy sample-to-sample) with a simple moving average
    k = 21
    kernel = np.ones(k) / k
    speed_smooth = np.convolve(gt_native_speed, kernel, mode='same')
    accel_smooth = np.gradient(speed_smooth, gt_native_t)

    print("\n=== GT speed profile at native rate (first ~15s), smoothed ===")
    print(f"{'t(rel)':>8} {'speed(m/s)':>12} {'accel(m/s^2)':>14}")
    idxs = np.linspace(0, len(gt_native_t) - 1, 40).astype(int)
    for i in idxs:
        print(f"{gt_native_t[i]:8.2f} {speed_smooth[i]:12.4f} {accel_smooth[i]:14.4f}")
    print(f"\nPeak smoothed acceleration in this window: {np.nanmax(np.abs(accel_smooth)):.4f} m/s^2 "
          f"at t={gt_native_t[np.nanargmax(np.abs(accel_smooth))]:.2f}s")

    # Save arrays for later plotting/inspection
    out_path = os.path.join(SCRIPT_DIR, "modality_compare.npz")
    np.savez(out_path,
             t=t[1:], imu_mag=imu_mag, corr_mag=corr_mag, fused_mag=fused_mag, gt_mag=gt_mag,
             pos_out=pos_out_m, pos_pre=pos_pre_m, abs_t=abs_t,
             speed_pre=speed_pre, speed_out=speed_out, gt_speed=gt_speed)
    print(f"\nSaved arrays to {out_path}")


if __name__ == "__main__":
    main()
