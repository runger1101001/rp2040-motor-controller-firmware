
#pragma once

#include "SimpleFOC.h"

int estimate_pole_count(BLDCMotor* motor);
void dispatch_util(char* cmd, BLDCMotor* motor0, BLDCMotor* motor1);