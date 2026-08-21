#include "rerun_wrapper.h"

RerunWrapper::RerunWrapper(const std::string &application_id) : rec_(application_id)
{
  // Fail loudly (throw std::runtime_error) if the viewer can't be spawned/reached, rather than
  // silently disabling logging -- callers should know immediately that visualization is dead.
  rec_.spawn().throw_on_failure();

  // Our world frame is NED (X=North, Y=East, Z=Down), but Rerun's default 3D view expects
  // Z-up. Rather than rotating every logged position/quaternion, just tell the viewer which
  // way is "up": this only re-orients the camera/grid for views rooted at or below "/" (i.e.
  // everything) -- it does not touch the logged data, so anything else consuming this
  // recording still sees plain NED coordinates.
  rec_.log_static("/", rerun::ViewCoordinates::FRD);
}

void RerunWrapper::publishBoxes(const std::string &entity_path, const std::vector<RerunBox> &boxes)
{
  std::vector<rerun::components::Translation3D> centers;
  std::vector<rerun::HalfSize3D> half_sizes;
  std::vector<rerun::Quaternion> quaternions;
  std::vector<rerun::Rgba32> colors;
  centers.reserve(boxes.size());
  half_sizes.reserve(boxes.size());
  quaternions.reserve(boxes.size());
  colors.reserve(boxes.size());

  for (const auto &box : boxes)
  {
    centers.emplace_back(
        static_cast<float>(box.center[0]), static_cast<float>(box.center[1]), static_cast<float>(box.center[2])
    );
    half_sizes.emplace_back(
        static_cast<float>(box.half_size[0]), static_cast<float>(box.half_size[1]), static_cast<float>(box.half_size[2])
    );
    quaternions.push_back(rerun::Quaternion::from_xyzw(
        static_cast<float>(box.quat_xyzw[0]), static_cast<float>(box.quat_xyzw[1]), static_cast<float>(box.quat_xyzw[2]),
        static_cast<float>(box.quat_xyzw[3])
    ));
    colors.push_back(rerun::Rgba32(box.r, box.g, box.b, box.a));
  }

  rec_.log(
      entity_path,
      rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes).with_quaternions(quaternions).with_colors(colors)
  );
}

template <typename PointT>
void RerunWrapper::publishPoints(const std::string &entity_path, const pcl::PointCloud<PointT> &cloud, uint8_t r, uint8_t g, uint8_t b,
                                  uint8_t a)
{
  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud.points.size());
  for (const auto &p : cloud.points) { positions.emplace_back(p.x, p.y, p.z); }

  std::vector<rerun::Rgba32> colors;
  if constexpr (std::is_same_v<PointT, PointTypeRGB>)
  {
    // PointTypeRGB carries its own per-point color; the r/g/b fallback is ignored.
    colors.reserve(cloud.points.size());
    for (const auto &p : cloud.points) { colors.push_back(rerun::Rgba32(p.r, p.g, p.b, a)); }
  }
  else { colors.push_back(rerun::Rgba32(r, g, b, a)); } // splat one color across all points

  rec_.log(entity_path, rerun::Points3D(positions).with_colors(colors));
}

// Explicit instantiation: these are the only two point types published as clouds.
template void RerunWrapper::publishPoints<PointType>(
    const std::string &, const pcl::PointCloud<PointType> &, uint8_t, uint8_t, uint8_t, uint8_t);
template void RerunWrapper::publishPoints<PointTypeRGB>(
    const std::string &, const pcl::PointCloud<PointTypeRGB> &, uint8_t, uint8_t, uint8_t, uint8_t);

void RerunWrapper::publishImage(const std::string &entity_path, const cv::Mat &image)
{
  if (image.empty() || image.type() != CV_8UC3 || !image.isContinuous()) { return; }

  rec_.log(entity_path, rerun::Image(image.data, rerun::WidthHeight(image.cols, image.rows), rerun::ColorModel::BGR));
}

void RerunWrapper::appendAndPublishTrajectoryPoints(const std::string &entity_path, const std::vector<std::array<double, 3>> &new_points,
                                                     uint8_t r, uint8_t g, uint8_t b, uint8_t a, bool reset)
{
  std::vector<rerun::Vec3D> &points = trajectory_points_[entity_path];
  if (reset) { points.clear(); }

  // No explicit reserve() here: reserve(n) allocates exactly n once n exceeds capacity
  // (unlike emplace_back's amortized doubling growth), so calling it every step with a
  // single new point would force a full reallocation+copy on every call -- exactly the
  // per-step O(total length) cost this function exists to avoid.
  for (const auto &point : new_points)
  {
    points.emplace_back(static_cast<float>(point[0]), static_cast<float>(point[1]), static_cast<float>(point[2]));
  }

  // Buffering above is cheap; the actual log() below re-sends the whole line strip, so
  // throttle that part per entity_path instead of skipping the buffering above.
  auto now = std::chrono::steady_clock::now();
  auto it = last_trajectory_pub_time_.find(entity_path);
  if (it != last_trajectory_pub_time_.end() &&
      std::chrono::duration<double>(now - it->second).count() < kTrajectoryPublishIntervalSec)
  {
    return;
  }
  last_trajectory_pub_time_[entity_path] = now;

  rec_.log(
      entity_path,
      rerun::LineStrips3D(rerun::components::LineStrip3D(points)).with_colors({rerun::Rgba32(r, g, b, a)})
  );
}
