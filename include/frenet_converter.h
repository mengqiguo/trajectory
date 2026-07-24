#pragma once

#include <vector>

#include "trajectory_point.h"

class FrenetConverter
{
public:
    static void CartesianToFrenet(std::vector<TrajectoryPoint>& traj);
    static void FrenetToCartesian(std::vector<TrajectoryPoint>& traj);
};
