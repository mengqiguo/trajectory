#include "trajectory_smoother.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>

int main() {
    // 生成带噪声的参考线（类似 Python 示例）
    int n = 50;
    std::vector<std::pair<double,double>> raw_xy;
    raw_xy.reserve(n);

    // 使用随机数生成噪声
    std::default_random_engine gen(42);
    std::normal_distribution<double> noise(0.0, 0.15);

    double step = 40.0 / (n - 1);
    for (int i = 0; i < n; ++i) {
        double s = i * step;
        double x = s;
        double y = 0.7 * std::sin(s * 0.18) + noise(gen);
        raw_xy.push_back({x, y});
    }

    // 创建平滑器
    FemDiscreteRefLineSmoother smoother;
    // 可选调整权重（默认已设）
    // smoother.setWeightFem(1000.0);
    // smoother.setWeightLength(1.0);
    // smoother.setWeightRef(100.0);

    // 求解
    auto smooth_xy = smoother.solve(raw_xy);

    // 输出原始和平滑结果到文件（便于绘图）
    std::ofstream fout("result.txt");
    if (fout.is_open()) {
        for (int i = 0; i < n; ++i) {
            fout << raw_xy[i].first << " " << raw_xy[i].second << " "
                 << smooth_xy[i].first << " " << smooth_xy[i].second << "\n";
        }
        fout.close();
        std::cout << "结果已写入 result.txt，可用绘图工具查看。" << std::endl;
    }

    // 同时打印前几个点
    std::cout << "平滑后前5个点: " << std::endl;
    for (int i = 0; i < std::min(5, n); ++i) {
        std::cout << "(" << smooth_xy[i].first << ", " << smooth_xy[i].second << ")" << std::endl;
    }

    return 0;
}