#ifndef TRAJECTORY_SMOOTHER_H
#define TRAJECTORY_SMOOTHER_H

#include <vector>
#include <utility>   // for std::pair

class FemDiscreteRefLineSmoother {
public:
    FemDiscreteRefLineSmoother();

    // 设置权重
    void setWeightFem(double w) { weight_fem_pos_deviation_ = w; }
    void setWeightLength(double w) { weight_path_length_ = w; }
    void setWeightRef(double w) { weight_ref_deviation_ = w; }

    // 主接口：输入原始点 (N×2)，输出平滑点
    std::vector<std::pair<double,double>> solve(
        const std::vector<std::pair<double,double>>& raw_xy);

private:
    double weight_fem_pos_deviation_ = 1000.0;
    double weight_path_length_       = 1.0;
    double weight_ref_deviation_     = 100.0;

    // 构建稀疏 P 矩阵（上三角）返回 OSQP 的 CSC 三元组
    // 数据格式：P_data, P_row_indices, P_col_pointers（长度 N+1）
    void buildP(int num_points,
                std::vector<double>& P_data,
                std::vector<int>& P_indices,
                std::vector<int>& P_indptr);

    // 构建 q 向量
    std::vector<double> buildQ(const std::vector<std::pair<double,double>>& raw_xy);

    // 构建等式约束 A（固定首尾点），返回 A 的 CSC 三元组、下界、上界
    void buildConstraint(const std::vector<std::pair<double,double>>& raw_xy,
                         std::vector<double>& A_data,
                         std::vector<int>& A_indices,
                         std::vector<int>& A_indptr,
                         std::vector<double>& lower,
                         std::vector<double>& upper);
};

#endif // TRAJECTORY_SMOOTHER_H