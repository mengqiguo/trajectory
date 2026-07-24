#pragma once

#include <vector>

class QPSmoother
{
public:
	std::vector<double> Smooth(const std::vector<double>& l_ref);

private:
	double w_ref_ = 1.0;
	double w_smooth_ = 20.0;
};
