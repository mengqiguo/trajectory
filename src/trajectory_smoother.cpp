#include "trajectory_smoother.h"
#include <osqp/osqp.h>
#include <cmath>
#include <iostream>
#include <vector>

FemDiscreteRefLineSmoother::FemDiscreteRefLineSmoother() {}

void FemDiscreteRefLineSmoother::buildP(int num_points,
                                        std::vector<double>& P_data,
                                        std::vector<int>& P_indices,
                                        std::vector<int>& P_indptr) {
    int num_vars = 2 * num_points;
    double wf = weight_fem_pos_deviation_;
    double wl = weight_path_length_;
    double wr = weight_ref_deviation_;

    // 临时存储每列的 (row, value)，上三角
    std::vector<std::vector<std::pair<int,double>>> columns(num_vars);

    // ---- 第 0 点 (col 0,1) ----
    for (int col = 0; col < 2; ++col) {
        columns[col].push_back({col, wf + wl + wr});
    }

    // ---- 第 1 点 (col 2,3) ----
    for (int col = 2; col < 4; ++col) {
        columns[col].push_back({col-2, -2.0*wf - wl});
        columns[col].push_back({col,    5.0*wf + 2.0*wl + wr});
    }

    // ---- 中间点 [2, N-3] ----
    int second_last = num_points - 2;
    for (int pt = 2; pt < second_last; ++pt) {
        int base_col = pt * 2;
        for (int off = 0; off < 2; ++off) {
            int col = base_col + off;
            columns[col].push_back({col-4, wf});
            columns[col].push_back({col-2, -4.0*wf - wl});
            columns[col].push_back({col,   6.0*wf + 2.0*wl + wr});
        }
    }

    // ---- 倒数第二点 (N-2) ----
    int col_start = num_vars - 4;
    int col_end = num_vars - 2;
    for (int col = col_start; col < col_end; ++col) {
        columns[col].push_back({col-4, wf});
        columns[col].push_back({col-2, -4.0*wf - wl});
        columns[col].push_back({col,   5.0*wf + 2.0*wl + wr});
    }

    // ---- 最后点 (N-1) ----
    for (int col = num_vars - 2; col < num_vars; ++col) {
        columns[col].push_back({col-4, wf});
        columns[col].push_back({col-2, -2.0*wf - wl});
        columns[col].push_back({col,   wf + wl + wr});
    }

    // ---- 合并为 CSC 格式，并乘以 2.0 ----
    P_indptr.clear();
    P_indptr.push_back(0);
    int ptr = 0;
    for (int col = 0; col < num_vars; ++col) {
        for (auto& entry : columns[col]) {
            P_data.push_back(entry.second * 2.0);
            P_indices.push_back(entry.first);
            ptr++;
        }
        P_indptr.push_back(ptr);
    }
}

std::vector<double> FemDiscreteRefLineSmoother::buildQ(
    const std::vector<std::pair<double,double>>& raw_xy) {
    int n = raw_xy.size();
    std::vector<double> q(2*n, 0.0);
    double wr = weight_ref_deviation_;
    for (int i = 0; i < n; ++i) {
        q[2*i]     = -2.0 * wr * raw_xy[i].first;
        q[2*i+1]   = -2.0 * wr * raw_xy[i].second;
    }
    return q;
}

void FemDiscreteRefLineSmoother::buildConstraint(
    const std::vector<std::pair<double,double>>& raw_xy,
    std::vector<double>& A_data,
    std::vector<int>& A_indices,
    std::vector<int>& A_indptr,
    std::vector<double>& lower,
    std::vector<double>& upper) {
    int n = raw_xy.size();
    int num_vars = 2*n;
    // 固定起点和终点，共4个等式约束
    int ncons = 4;
    lower.resize(ncons);
    upper.resize(ncons);

    // 我们将构建稀疏 A 矩阵（行优先），但需要转为 CSC
    // 先构建行列表，每行一个非零元素
    std::vector<std::vector<std::pair<int,double>>> rows(ncons);

    // 起点 x
    rows[0].push_back({0, 1.0});
    lower[0] = raw_xy[0].first;
    upper[0] = raw_xy[0].first;

    // 起点 y
    rows[1].push_back({1, 1.0});
    lower[1] = raw_xy[0].second;
    upper[1] = raw_xy[0].second;

    // 终点 x
    int x_idx = 2*(n-1);
    rows[2].push_back({x_idx, 1.0});
    lower[2] = raw_xy[n-1].first;
    upper[2] = raw_xy[n-1].first;

    // 终点 y
    int y_idx = 2*(n-1) + 1;
    rows[3].push_back({y_idx, 1.0});
    lower[3] = raw_xy[n-1].second;
    upper[3] = raw_xy[n-1].second;

    // 转换为 CSC
    // 先统计每列的非零个数
    std::vector<int> col_count(num_vars, 0);
    for (int row = 0; row < ncons; ++row) {
        for (auto& entry : rows[row]) {
            col_count[entry.first]++;
        }
    }

    // 构建列指针
    A_indptr.clear();
    A_indptr.push_back(0);
    for (int col = 0; col < num_vars; ++col) {
        A_indptr.push_back(A_indptr.back() + col_count[col]);
    }

    // 填充数据（按列顺序）
    std::vector<int> cur_pos = A_indptr;  // 拷贝，用于当前填充位置
    // 但我们需要行索引，先暂存为 vector of pairs (row, value)
    // 因为我们按列填充，需要知道每列当前的填充位置
    // 更简单：先构建密集的A_data和A_indices按列顺序
    // 由于只有4个非零，直接手动填充

    // 重新初始化
    A_indptr.clear();
    A_indptr.push_back(0);
    // 创建临时存储每列的数据
    std::vector<std::vector<std::pair<int,double>>> cols(num_vars);
    for (int row = 0; row < ncons; ++row) {
        for (auto& entry : rows[row]) {
            cols[entry.first].push_back({row, entry.second});
        }
    }

    // 收集
    A_data.clear();
    A_indices.clear();
    for (int col = 0; col < num_vars; ++col) {
        for (auto& entry : cols[col]) {
            A_data.push_back(entry.second);
            A_indices.push_back(entry.first);
        }
        A_indptr.push_back(A_indptr.back() + cols[col].size());
    }
}

std::vector<std::pair<double,double>> FemDiscreteRefLineSmoother::solve(
    const std::vector<std::pair<double,double>>& raw_xy) {
    int n = raw_xy.size();
    if (n < 2) {
        return raw_xy;
    }
    int num_vars = 2 * n;

    // 1. 构建 P
    std::vector<double> P_data;
    std::vector<int> P_indices;
    std::vector<int> P_indptr;
    buildP(n, P_data, P_indices, P_indptr);

    // 2. 构建 q
    std::vector<double> q = buildQ(raw_xy);

    // 3. 构建约束
    std::vector<double> A_data;
    std::vector<int> A_indices_int, A_indptr_int;
    std::vector<double> lower, upper;
    buildConstraint(raw_xy, A_data, A_indices_int, A_indptr_int, lower, upper);

    // 将数据转换为 OSQP 类型
    std::vector<OSQPInt> P_indices_c(P_indices.begin(), P_indices.end());
    std::vector<OSQPInt> P_indptr_c(P_indptr.begin(), P_indptr.end());
    std::vector<OSQPFloat> P_data_c(P_data.begin(), P_data.end());

    std::vector<OSQPInt> A_indices_c(A_indices_int.begin(), A_indices_int.end());
    std::vector<OSQPInt> A_indptr_c(A_indptr_int.begin(), A_indptr_int.end());
    std::vector<OSQPFloat> A_data_c(A_data.begin(), A_data.end());

    std::vector<OSQPFloat> q_c(q.begin(), q.end());
    std::vector<OSQPFloat> lower_c(lower.begin(), lower.end());
    std::vector<OSQPFloat> upper_c(upper.begin(), upper.end());

    // 4. 设置 OSQP 问题
    OSQPCscMatrix* P = OSQPCscMatrix_new(
        static_cast<OSQPInt>(num_vars),
        static_cast<OSQPInt>(num_vars),
        static_cast<OSQPInt>(P_data_c.size()),
        P_data_c.data(),
        P_indices_c.data(),
        P_indptr_c.data());

    OSQPCscMatrix* A = OSQPCscMatrix_new(
        static_cast<OSQPInt>(lower_c.size()),
        static_cast<OSQPInt>(num_vars),
        static_cast<OSQPInt>(A_data_c.size()),
        A_data_c.data(),
        A_indices_c.data(),
        A_indptr_c.data());

    if (P == nullptr || A == nullptr) {
        std::cerr << "OSQP matrix allocation failed" << std::endl;
        if (P != nullptr) OSQPCscMatrix_free(P);
        if (A != nullptr) OSQPCscMatrix_free(A);
        return raw_xy;
    }

    OSQPSettings* settings = OSQPSettings_new();
    if (settings == nullptr) {
        std::cerr << "OSQP settings allocation failed" << std::endl;
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);
        return raw_xy;
    }

    osqp_set_default_settings(settings);
    settings->verbose = 0;
    settings->eps_abs = 1e-5;
    settings->eps_rel = 1e-5;
    settings->max_iter = 20000;

    OSQPSolver* solver = nullptr;
    OSQPInt setup_status = osqp_setup(
        &solver,
        P,
        q_c.data(),
        A,
        lower_c.data(),
        upper_c.data(),
        static_cast<OSQPInt>(lower_c.size()),
        static_cast<OSQPInt>(num_vars),
        settings);

    if (setup_status != 0 || solver == nullptr) {
        std::cerr << "OSQP setup failed, code: " << setup_status << std::endl;
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);
        OSQPSettings_free(settings);
        return raw_xy;
    }

    OSQPInt solve_status = osqp_solve(solver);

    std::vector<std::pair<double,double>> smooth_xy;
    if (solve_status != 0 || solver->info->status_val != OSQP_SOLVED) {
        std::cerr << "OSQP failed: " << solver->info->status << std::endl;
        smooth_xy = raw_xy;  // 返回原始值
    } else {
        OSQPFloat* sol = solver->solution->x;
        smooth_xy.resize(n);
        for (int i = 0; i < n; ++i) {
            smooth_xy[i].first  = sol[2*i];
            smooth_xy[i].second = sol[2*i+1];
        }
    }

    // 清理
    osqp_cleanup(solver);
    OSQPCscMatrix_free(P);
    OSQPCscMatrix_free(A);
    OSQPSettings_free(settings);

    return smooth_xy;
}