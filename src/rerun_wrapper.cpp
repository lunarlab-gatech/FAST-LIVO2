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
