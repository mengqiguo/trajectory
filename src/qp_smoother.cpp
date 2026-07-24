#include "qp_smoother.h"

#include <Eigen/Dense>

std::vector<double> QPSmoother::Smooth(const std::vector<double>& l_ref)
{
	int N = l_ref.size();

	Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N, N);
	Eigen::VectorXd f = Eigen::VectorXd::Zero(N);

	// reference term
	for (int i = 0; i < N; i++) {
		H(i, i) += 2 * w_ref_;
		f(i) += -2 * w_ref_ * l_ref[i];
	}

	// smooth term
	for (int i = 1; i < N - 1; i++) {
		int a = i - 1;
		int b = i;
		int c = i + 1;

		double coef[3] = {1, -2, 1};
		int id[3] = {a, b, c};

		for (int m = 0; m < 3; m++) {
			for (int n = 0; n < 3; n++) {
				H(id[m], id[n]) += 2 * w_smooth_ * coef[m] * coef[n];
			}
		}
	}

	Eigen::VectorXd x = -H.ldlt().solve(f);

	std::vector<double> result;
	for (int i = 0; i < N; i++) {
		result.push_back(x(i));
	}

	return result;
}
