# Rerun visualization — planned changes

Status: **not yet implemented** — this is the plan to pick up from.

## 1. Move `RerunWrapper` ownership: `VoxelMapManager` → `LIVMapper`

`VoxelMapManager` shouldn't own the Rerun connection — `LIVMapper` is the actual top-level
node/runner (constructed in `main.cpp`, already injects `voxel_map_pub_` into
`voxelmap_manager` the same way). Change:

- `LIVMapper` gets `std::unique_ptr<RerunWrapper> rerun_wrapper_;` + a lazy accessor
  `RerunWrapper *GetRerunWrapper();` (constructs on first call only — don't spawn a viewer
  if nothing ever asks).
- `VoxelMapManager::pubVoxelMap()` signature becomes `pubVoxelMap(RerunWrapper &rerun)` —
  it stops owning/lazily-constructing its own; pure consumer, like `voxel_map_pub_`.
- Since `RerunWrapper`'s constructor now throws on failed connection (already done), the
  reference is always valid by the time `pubVoxelMap` runs — no null-check needed.
- Delete `VoxelMapManager::rerun_wrapper_` member.
- Update call site `LIVMapper.cpp:448` to fetch the wrapper and pass it through.

## 2. Throttle plane publishing to 1 Hz

`pubVoxelMap()` does a full `voxel_map_` traversal every call — expensive and growing with
map size (see lag investigation earlier in this session). Add a time gate at the top:

```cpp
ros::Time now = ros::Time::now();
if ((now - last_rerun_pub_time_).toSec() < 1.0) { return; }
last_rerun_pub_time_ = now;
```

`last_rerun_pub_time_` as a `ros::Time` member (default-zero-initialized, so the first call
always passes since `now - 0` is huge). Keep this gate self-contained in `pubVoxelMap()`.

(Already done, separately: RViz `MarkerArray` publish path inside `pubVoxelMap()` is
temporarily commented out, since Rerun now covers plane viz.)

## 3. Add estimated + GT trajectories to Rerun

New `RerunWrapper` method (mirrors `publishBoxes`'s full-snapshot "latest wins" style):

```cpp
void publishTrajectory(const std::string &entity_path,
                        const std::vector<std::array<double, 3>> &points,
                        uint8_t r, uint8_t g, uint8_t b, uint8_t a);
```
Implementation: `rerun::LineStrips3D(rerun::components::LineStrip3D(vec3d_points)).with_colors(...)`,
logged via `rec_.log(entity_path, ...)`. No new includes needed (`rerun.hpp` already pulls
in `LineStrips3D`/`Vec3D`/`Color`).

**Estimated trajectory** — `LIVMapper` accumulates its own history (nothing upstream
provides one): add `std::vector<std::array<double,3>> est_trajectory_;`, `push_back` from
`_state.pos_end` right after `publish_odometry(...)` (`LIVMapper.cpp:410`), every LIO step
(cheap — not part of the 1 Hz plane throttle).

**GT trajectory** — subscribe to `/odom/gt` (`nav_msgs::Path`, published at 20 Hz by
`research/Hercules/rosbag_play.py`, same `camera_init`/NED frame as everything else — no
transform needed). New subscriber + callback in `LIVMapper`:
```cpp
sub_gt_odom_ = nh.subscribe("/odom/gt", 100, &LIVMapper::gt_odom_cbk, this);
void gt_odom_cbk(const nav_msgs::Path::ConstPtr &msg); // replaces gt_trajectory_ wholesale
```
(The incoming `Path` is already the full accumulated/sub-sampled trajectory, so just copy
its poses each time — don't append.)

**Publish call site** — alongside the (now-throttled) `pubVoxelMap()` call at
`LIVMapper.cpp:448`:
```cpp
RerunWrapper *rerun = GetRerunWrapper();
voxelmap_manager->pubVoxelMap(*rerun);
rerun->publishTrajectory("trajectory/est", est_trajectory_, 255, 165, 0, 255);   // orange
rerun->publishTrajectory("trajectory/gt",  gt_trajectory_,  255, 255, 255, 255); // white
```
Trajectory logging is cheap (no eigen decomposition, just a growing point list) so it isn't
gated by the 1 Hz plane throttle — runs every LIO step for smooth lines.

## Files touched

- `include/rerun_wrapper.h`, `src/rerun_wrapper.cpp` — `publishTrajectory()`.
- `include/voxel_map.h`, `src/voxel_map.cpp` — drop `rerun_wrapper_` ownership, add
  `last_rerun_pub_time_`, change `pubVoxelMap()` signature + add throttle.
- `include/LIVMapper.h`, `src/LIVMapper.cpp` — own `RerunWrapper`, `GetRerunWrapper()`,
  `est_trajectory_`/`gt_trajectory_`, `sub_gt_odom_`/`gt_odom_cbk`, update call site.
