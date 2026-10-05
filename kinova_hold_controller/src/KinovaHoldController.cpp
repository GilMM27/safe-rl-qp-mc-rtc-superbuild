#include "KinovaHoldController.h"

#include <map>
#include <vector>

KinovaHoldController::KinovaHoldController(mc_rbdyn::RobotModulePtr rm,
                                           double dt,
                                           const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({});

  postureTask->target({{"joint_1", {0.0}},
                       {"joint_2", {0.2618}},
                       {"joint_3", {3.14}},
                       {"joint_4", {-2.269}},
                       {"joint_5", {0.0}},
                       {"joint_6", {0.959878729}},
                       {"joint_7", {1.57}}});
  postureTask->stiffness(1.0);

  mc_rtc::log::success("KinovaHoldController initialized");
}

CONTROLLER_CONSTRUCTOR("KinovaHoldController", KinovaHoldController)
