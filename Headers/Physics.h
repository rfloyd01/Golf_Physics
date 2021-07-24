#pragma once

#include "Golf_Club.h"
#include "Math.h"
#include "eigen.h"

std::pair<Vector_3d, Vector_3d> getBallVelocities(Golf_Club &club, Golf_Ball &ball, Golf_Swing &swing);
