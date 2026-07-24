#pragma once

#include <vector>

#include "trajectory_point.h"

class TrajectorySmoother
{
public:
	std::vector<TrajectoryPoint> Process(const std::vector<TrajectoryPoint>& input);
};
