#include <cmath>
#include <iostream>
#include <vector>

#include "trajectory_smoother.h"

int main() {
	std::vector<TrajectoryPoint> input;

	for (int i = 0; i < 80; i++) {
		TrajectoryPoint p;
		double t = i * 0.1;

		p.x = t;
		p.y = std::sin(t) + 0.1 * std::sin(20 * t);
		p.cos_heading = 1;
		p.sin_heading = 0;

		input.push_back(p);
	}

	TrajectorySmoother smoother;
	auto output = smoother.Process(input);

	for (auto& p : output) {
		std::cout << p.x << " " << p.y << " " << p.cos_heading << " "
							<< p.sin_heading << std::endl;
	}
}
