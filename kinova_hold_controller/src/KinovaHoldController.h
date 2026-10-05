#pragma once

#include <mc_control/mc_controller.h>

#include "api.h"

struct KinovaHoldController_DLLAPI KinovaHoldController : public mc_control::MCController
{
  KinovaHoldController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
};
