#include <human_mocap_tracker/HumanMocapTracker.h>

#include <cassert>
#include <cstdio>
#include <cmath>
#include <fstream>

int main()
{
  const char * path = "qpos_trajectory_test.csv";
  {
    std::ofstream output(path);
    output << "# time,qpos0,qpos1\n0,0,1\n1,2,3\n";
  }
  const auto trajectory = human_mocap::QposTrajectory::loadCsv(path, 2);
  const auto frame = trajectory.sample(0.25, false);
  assert(std::abs(frame.qpos[0] - 0.5) < 1e-12);
  assert(std::abs(frame.qpos[1] - 1.5) < 1e-12);
  assert(std::abs(trajectory.sample(1.5, true).qpos[0] - 1.0) < 1e-12);
  std::remove(path);
}
