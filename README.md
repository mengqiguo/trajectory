# trajectory_smoother

基于 FEM 离散平滑思想和 OSQP 二次规划求解器的二维轨迹平滑示例工程。

## 项目简介

本项目实现了一个简化版轨迹平滑器 `FemDiscreteRefLineSmoother`：

- 输入：一组二维离散点 `[(x0, y0), ..., (xN-1, yN-1)]`
- 输出：平滑后的二维离散点
- 优化目标：综合考虑
  - 二阶差分平滑项（FEM position deviation）
  - 路径长度项（path length）
  - 参考线偏差项（reference deviation）
- 约束：固定起点和终点（x、y 坐标均固定）

核心求解由 OSQP 完成。

## 目录结构

```text
.
├── CMakeLists.txt
├── include/
│   └── trajectory_smoother.h
├── src/
│   ├── main.cpp
│   └── trajectory_smoother.cpp
├── build/
└── result.txt
```

## 环境依赖

- Linux
- CMake >= 3.10
- C++14 编译器（如 g++）
- OSQP（需提供 CMake config，支持 `find_package(osqp CONFIG REQUIRED)`）

## 构建方式

### 方式 1：CMake（推荐）

```bash
cd /data/mengqi/trajectory_smoother
cmake -S . -B build
cmake --build build -j
```

生成可执行文件：`build/fem_smoother`

### 方式 2：直接使用 g++

```bash
cd /data/mengqi/trajectory_smoother
mkdir -p build
/usr/bin/g++ -fdiagnostics-color=always -g -std=c++14 \
  -I./include -I/usr/local/include \
  ./src/main.cpp ./src/trajectory_smoother.cpp \
  -L/usr/local/lib -Wl,-rpath,/usr/local/lib -losqp \
  -o ./build/fem_smoother
```

## 运行

```bash
cd /data/mengqi/trajectory_smoother
./build/fem_smoother
```

程序会：

- 随机生成一条带噪声的参考线（固定随机种子）
- 调用平滑器进行求解
- 输出结果到 `result.txt`
- 在终端打印前 5 个平滑点

## 输出文件格式

`result.txt` 每行包含 4 列数据：

```text
raw_x raw_y smooth_x smooth_y
```

可直接用于 Python/Matlab/gnuplot 可视化对比原始轨迹和平滑轨迹。

## 参数说明

默认权重定义于 `include/trajectory_smoother.h`：

- `weight_fem_pos_deviation_ = 1000.0`
- `weight_path_length_ = 1.0`
- `weight_ref_deviation_ = 100.0`

可在 `src/main.cpp` 中通过以下接口调整：

- `setWeightFem(double w)`
- `setWeightLength(double w)`
- `setWeightRef(double w)`

## 常见问题

1. 报错找不到 OSQP

- 确认已正确安装 OSQP 及其开发文件
- 确认 OSQP 的 CMake config 在 `CMAKE_PREFIX_PATH` 可搜索路径内

2. 运行后没有生成结果文件

- 确认当前目录有写权限
- 确认程序是否正常运行并输出 "结果已写入 result.txt"

## 许可证

当前仓库未声明许可证；如需开源发布，请补充 `LICENSE` 文件。
