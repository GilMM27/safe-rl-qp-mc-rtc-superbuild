#include "human_mocap_tracker/HumanMocapTracker.h"

#include <limits>
#include <stdexcept>
#include <utility>

namespace human_mocap
{

HumanMocapTracker::HumanMocapTracker(const mjModel & model,
                                     std::vector<GeometryGroup> human_parts,
                                     std::vector<int> robot_geoms,
                                     double max_distance)
: model_(model), human_parts_(std::move(human_parts)), robot_geoms_(std::move(robot_geoms)),
  max_distance_(max_distance)
{
  if(max_distance_ <= 0.0)
  {
    throw std::invalid_argument("HumanMocapTracker max_distance must be positive");
  }
}

ClosestPart HumanMocapTracker::closest(const mjData & data) const
{
  ClosestPart result;
  auto best_distance = max_distance_;

  for(const auto & part : human_parts_)
  {
    for(const auto human_geom : part.geom_ids)
    {
      if(human_geom < 0 || human_geom >= model_.ngeom)
      {
        continue;
      }
      for(const auto robot_geom : robot_geoms_)
      {
        if(robot_geom < 0 || robot_geom >= model_.ngeom || robot_geom == human_geom)
        {
          continue;
        }

        mjtNum fromto[6];
        const auto distance = mj_geomDistance(&model_, &data, human_geom, robot_geom, max_distance_, fromto);
        if(distance < 0.0 || distance >= best_distance)
        {
          continue;
        }

        best_distance = distance;
        result.label = part.label;
        result.distance = distance;
        for(int i = 0; i < 3; ++i)
        {
          result.human_point[i] = fromto[i];
          result.robot_point[i] = fromto[i + 3];
        }
        result.valid = true;
      }
    }
  }
  return result;
}

} // namespace human_mocap
