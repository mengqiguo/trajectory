#include "frenet_converter.h"

#include <cmath>

void FrenetConverter::CartesianToFrenet(std::vector<TrajectoryPoint>& traj)
{
	double x0 = traj[0].x;
	double y0 = traj[0].y;

	double theta = std::atan2(traj.back().y - y0, traj.back().x - x0);

	for (auto& p : traj) {
		double dx = p.x - x0;
		double dy = p.y - y0;

		p.s = dx * std::cos(theta) + dy * std::sin(theta);
		p.l = -dx * std::sin(theta) + dy * std::cos(theta);
	}
}

void FrenetConverter::FrenetToCartesian(std::vector<TrajectoryPoint>& traj)
{
	double x0 = traj[0].x;
	double y0 = traj[0].y;

	double theta = std::atan2(traj.back().y - y0, traj.back().x - x0);

	for (auto& p : traj) {
		p.x = x0 + p.s * std::cos(theta) - p.l * std::sin(theta);
		p.y = y0 + p.s * std::sin(theta) + p.l * std::cos(theta);
	}
}
