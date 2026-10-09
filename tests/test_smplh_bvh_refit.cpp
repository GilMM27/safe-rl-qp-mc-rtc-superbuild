#include "../smplh_playback/SmplhDistanceWorker.h"

#include <cmath>
#include <stdexcept>

int main()
{
  using mc_mujoco::SmplhBVH;
  std::vector<fcl::Vector3d> vertices{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  std::vector<fcl::Triangle> faces{{0, 1, 2}};
  SmplhBVH model;
  model.beginModel();
  model.addSubModel(vertices, faces);
  model.endModel();

  // Move repeatedly, including a large jump and a return to the initial pose.
  for(double x : {10., -5., 0.})
  {
    for(auto & vertex : vertices) vertex.x() = x;
    if(model.beginReplaceModel() != fcl::BVH_OK
       || model.replaceSubModel(vertices) != fcl::BVH_OK
       || model.endReplaceModel(true, true) != fcl::BVH_OK)
      throw std::runtime_error("current-pose refit failed");
    if(model.prev_vertices != nullptr)
      throw std::runtime_error("refit retained previous-pose vertices");

    SmplhBVH fresh;
    fresh.beginModel();
    fresh.addSubModel(vertices, faces);
    fresh.endModel();
    // A refitted single-triangle bound must match a fresh current-pose bound.
    const auto & actual = model.getBV(0).bv;
    const auto & expected = fresh.getBV(0).bv;
    if((actual.obb.To - expected.obb.To).norm() > 1e-10
       || (actual.obb.extent - expected.obb.extent).norm() > 1e-10)
      throw std::runtime_error("refit bounds differ from current-pose bounds");
  }
}
