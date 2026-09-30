#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>
#include <cstddef>

namespace human_mocap
{

struct GeometryGroup
{
  std::string label;
  std::vector<int> geom_ids;
};

struct ClosestPart
{
  std::string label;
  double distance = 0.0;
  double human_point[3] = {0.0, 0.0, 0.0};
  double robot_point[3] = {0.0, 0.0, 0.0};
  bool valid = false;
};

struct TrajectoryFrame
{
  double time = 0.0;
  std::vector<double> qpos;
};

/** Deterministic CSV qpos playback with linear interpolation. */
class QposTrajectory
{
public:
  static QposTrajectory loadCsv(const std::string & path, std::size_t qpos_size);
  TrajectoryFrame sample(double time, bool loop) const;
  double duration() const noexcept;
  std::size_t qposSize() const noexcept;

private:
  explicit QposTrajectory(std::vector<TrajectoryFrame> frames);
  std::vector<TrajectoryFrame> frames_;
};

/** Computes closest tagged human geometry to configured robot geometry. */
class HumanMocapTracker
{
public:
  HumanMocapTracker(const mjModel & model,
                    std::vector<GeometryGroup> human_parts,
                    std::vector<int> robot_geoms,
                    double max_distance = 1.0);

  ClosestPart closest(const mjData & data) const;

private:
  const mjModel & model_;
  std::vector<GeometryGroup> human_parts_;
  std::vector<int> robot_geoms_;
  double max_distance_;
};

} // namespace human_mocap
