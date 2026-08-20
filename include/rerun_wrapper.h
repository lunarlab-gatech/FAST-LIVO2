#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rerun.hpp>

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

private:
  rerun::RecordingStream rec_;
};
