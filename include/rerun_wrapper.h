#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <rerun.hpp>
#include <utils/types.h>

/**
 * A single oriented box to visualize.
 *
 * @param center world-frame center position (x, y, z).
 * @param quat_xyzw orientation as an xyzw quaternion.
 * @param half_size half-extent along each local axis.
 * @param r red channel.
 * @param g green channel.
 * @param b blue channel.
 * @param a alpha channel.
 */
struct RerunBox
{
  double center[3];
  double quat_xyzw[4];
  double half_size[3];
  uint8_t r, g, b, a;
};


/**
 * Thin wrapper around a rerun::RecordingStream for publishing batches of oriented boxes.
 */
class RerunWrapper
{
public:
  /**
   * @param application_id identifies this recording stream to the Rerun viewer.
   * @throws std::runtime_error if the viewer could not be spawned/reached.
   */
  explicit RerunWrapper(const std::string &application_id);

  /**
   * Publishes the full current set of boxes to entity_path in a single batched call.
   *
   * Rerun's per-path "latest wins" semantics mean this fully replaces whatever was
   * previously published at entity_path, including dropping boxes that are no longer
   * present. Callers should always pass the complete, currently-live set (not just what
   * changed), and an empty vector clears entity_path entirely.
   *
   * @param entity_path the Rerun entity path to publish to.
   * @param boxes the complete, currently-live set of boxes to display.
   */
  void publishBoxes(const std::string &entity_path, const std::vector<RerunBox> &boxes);

  /**
   * Publishes cloud's points to entity_path, same "latest wins", no-accumulation semantics
   * as publishBoxes(). Takes the PCL cloud directly, so the one conversion into Rerun's
   * types happens here rather than through a caller-side struct.
   *
   * r/g/b are only used as a fallback for a PointT with no native color (e.g. PointType);
   * for a cloud that already carries color (e.g. PointTypeRGB), each point's own r/g/b wins
   * and the fallback is ignored. Explicitly instantiated in rerun_wrapper.cpp for the two
   * point types this project actually publishes as clouds.
   *
   * @param entity_path the Rerun entity path to publish to.
   * @param cloud the complete, currently-live set of points to display.
   * @param r fallback red channel.
   * @param g fallback green channel.
   * @param b fallback blue channel.
   * @param a alpha channel, applied to every point.
   */
  template <typename PointT>
  void publishPoints(const std::string &entity_path, const pcl::PointCloud<PointT> &cloud, uint8_t r, uint8_t g, uint8_t b,
                      uint8_t a = 255);

  /**
   * Appends new_points to entity_path's accumulated trajectory and republishes it, throttled
   * to kTrajectoryPublishIntervalSec -- points are buffered every call, never dropped.
   *
   * @param entity_path the Rerun entity path to publish to.
   * @param new_points points to append, in order, to whatever is already accumulated at
   *     entity_path.
   * @param r red channel.
   * @param g green channel.
   * @param b blue channel.
   * @param a alpha channel.
   * @param reset if true, discards whatever was previously accumulated at entity_path before
   *     appending new_points -- use when the caller's own trajectory was rebuilt from
   *     scratch rather than purely extended (e.g. an external source that restarted).
   */
  void appendAndPublishTrajectoryPoints(const std::string &entity_path, const std::vector<std::array<double, 3>> &new_points, uint8_t r,
                                        uint8_t g, uint8_t b, uint8_t a, bool reset = false);

  /**
   * Publishes a BGR8 image to entity_path, same "latest wins" semantics as publishBoxes().
   * image must be a non-empty, contiguous CV_8UC3 (BGR) cv::Mat -- this project's only
   * image format -- logged directly with no channel-swap copy; anything else is dropped.
   *
   * @param entity_path the Rerun entity path to publish to.
   * @param image the current BGR8 image.
   */
  void publishImage(const std::string &entity_path, const cv::Mat &image);

private:
  /** Min seconds between an entity_path's trajectory flushes; matches pubVoxelMap()'s cadence. */
  static constexpr double kTrajectoryPublishIntervalSec = 1.0;

  rerun::RecordingStream rec_;

  /** Per-entity accumulated trajectory points, in Rerun's own type. */
  std::unordered_map<std::string, std::vector<rerun::Vec3D>> trajectory_points_;

  /** Per-entity last flush time; an absent entry publishes immediately. */
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_trajectory_pub_time_;
};
