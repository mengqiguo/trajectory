#include "trajectory_smoother.h"

#include <cmath>
#include <vector>

#include "frenet_converter.h"
#include "qp_smoother.h"

std::vector<TrajectoryPoint> TrajectorySmoother::Process(
        const std::vector<TrajectoryPoint>& input)
{
    auto traj = input;

    // XY -> SL
    FrenetConverter::CartesianToFrenet(traj);

    // extract lateral
    std::vector<double> l_ref;
    for (auto& p : traj) {
        l_ref.push_back(p.l);
    }

    // QP
    QPSmoother qp;
    auto l_new = qp.Smooth(l_ref);

    // replace lateral
    for (int i = 0; i < traj.size(); i++) {
        traj[i].l = l_new[i];
    }

    // SL -> XY
    FrenetConverter::FrenetToCartesian(traj);

    // update heading
    for (int i = 0; i < traj.size() - 1; i++) {
        double dx = traj[i + 1].x - traj[i].x;
        double dy = traj[i + 1].y - traj[i].y;
        double norm = std::sqrt(dx * dx + dy * dy);

        traj[i].cos_heading = dx / norm;
        traj[i].sin_heading = dy / norm;
    }

    return traj;
}
