#pragma once

struct TrajectoryPoint
{
    double x;
    double y;
    double cos_heading;
    double sin_heading;
    double s;  // TODO 转换坐标系？
    double l;
};
