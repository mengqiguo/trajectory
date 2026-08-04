#include "trajectory_smoother.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>

int main(int argc, char* argv[]) {
    bool stdout_mode = false;
    bool auto_plot = true;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--stdout") {
            stdout_mode = true;
        } else if (std::string(argv[i]) == "--no-plot") {
            auto_plot = false;
        }
    }

    // 生成带噪声原参考轨迹
    int n = 50;
    std::vector<std::pair<double,double>> raw_xy;
    raw_xy.reserve(n);
    std::default_random_engine gen(42);
    std::normal_distribution<double> noise(0.0, 0.15);
    double step = 40.0 / (n - 1);//轨迹40m
    for (int i = 0; i < n; ++i) {
        double s = i * step;
        double x = s;
        double y = 0.7 * std::sin(s * 0.18) + noise(gen);
        raw_xy.push_back({x, y});
    }

    FemDiscreteRefLineSmoother smoother;
    auto smooth_xy = smoother.solve(raw_xy);

    if (stdout_mode) {
        for (int i = 0; i < n; ++i) {
            std::cout << raw_xy[i].first << " " << raw_xy[i].second << " "
                      << smooth_xy[i].first << " " << smooth_xy[i].second << "\n";
        }
        return 0;
    }

    std::ofstream fout("result.txt");
    if (fout.is_open()) {
        for (int i = 0; i < n; ++i) {
            fout << raw_xy[i].first << " " << raw_xy[i].second << " "
                 << smooth_xy[i].first << " " << smooth_xy[i].second << "\n";
        }
        fout.close();
        std::cout << "结果已写入 result.txt。" << std::endl;
    }

    if (auto_plot) {
        const char* plot_cmd = R"(python3 -c "
import matplotlib.pyplot as plt

data = []
with open('result.txt', 'r', encoding='utf-8') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        data.append(list(map(float, line.split())))

if not data:
    print('result.txt is empty, cannot plot')
    exit(2)

rx = [r[0] for r in data]
ry = [r[1] for r in data]
sx = [r[2] for r in data]
sy = [r[3] for r in data]

plt.figure(figsize=(9, 5))
plt.plot(rx, ry, 'o--', alpha=0.6, label='raw')
plt.plot(sx, sy, '-', linewidth=2.2, label='smoothed')
plt.axis('equal')
plt.grid(alpha=0.25)
plt.legend()
plt.title('Trajectory Smoothing Result')
plt.savefig('result.png', dpi=160, bbox_inches='tight')
print('Plot saved to result.png')
")";
        int plot_status = std::system(plot_cmd);
        if (plot_status != 0) {
            std::cerr << "自动绘图失败。请确保已安装 matplotlib (pip install matplotlib)。" << std::endl;
        }
    }

    std::cout << "平滑后前5个点: " << std::endl;
    for (int i = 0; i < std::min(5, n); ++i) {
        std::cout << "(" << smooth_xy[i].first << ", " << smooth_xy[i].second << ")" << std::endl;
    }

    return 0;
}