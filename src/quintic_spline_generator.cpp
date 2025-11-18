#include "spline_planner/quintic_spline_generator.hpp"
#include <iostream>
#include <cassert>
#include <algorithm>
#include <iostream>

bool QuinticSpline2D::generate(
    const std::vector<double> &start,
    const std::vector<double> &end,
    std::vector<std::vector<double>> &path,
    bool forward)
{
    if (start.size()!=3 || end.size()!=3) return false;

    double x0=start[0], y0=start[1], yaw0=start[2];
    double x1=end[0], y1=end[1], yaw1=end[2];

    if(!forward) { yaw0 += M_PI; yaw1 += M_PI; }

    // ------------------ 生成虚拟终点 ------------------
    double total_dist = std::hypot(x1-x0, y1-y0);
    double end_len = (total_dist<=end_straight_len_)?0:end_straight_len_;

    std::cout << "total_dist" << total_dist << std::endl;
    std::cout << "end len" << end_len << std::endl;
    std::cout << "end_straight_len_" << end_straight_len_ << std::endl;
    double x1_virtual = x1 - end_len*cos(yaw1);
    double y1_virtual = y1 - end_len*sin(yaw1);

    double dx = x1_virtual - x0;
    double dy = y1_virtual - y0;
    double dist = std::hypot(dx, dy);
    if(dist<1e-6) return false;

    // ------------------ quintic spline 系数 ------------------
    double dx0 = std::cos(yaw0)*dist;
    double dy0 = std::sin(yaw0)*dist;
    double dx1 = std::cos(yaw1)*dist;
    double dy1 = std::sin(yaw1)*dist;

    double a0=x0, a1=dx0, a2=0;
    double a3=10*(x1_virtual-x0)-6*dx0-4*dx1;
    double a4=-15*(x1_virtual-x0)+8*dx0+7*dx1;
    double a5=6*(x1_virtual-x0)-3*dx0-3*dx1;

    double b0=y0, b1=dy0, b2=0;
    double b3=10*(y1_virtual-y0)-6*dy0-4*dy1;
    double b4=-15*(y1_virtual-y0)+8*dy0+7*dy1;
    double b5=6*(y1_virtual-y0)-3*dy0-3*dy1;

    // ------------------ 等距采样 ------------------
    int N_spline = std::max(2,int(dist/spacing_));
    path.clear();
    path.reserve(N_spline + 10);

    for(int i=0;i<=N_spline;i++)
    {
        double s = double(i)/N_spline;
        double x = a0 + a1*s + a2*s*s + a3*s*s*s + a4*s*s*s*s + a5*s*s*s*s*s;
        double y = b0 + b1*s + b2*s*s + b3*s*s*s + b4*s*s*s*s + b5*s*s*s*s*s;
        path.push_back({x,y});
    }

    // ------------------ 末端直线段 ------------------
    if(total_dist>end_straight_len_){
        int N_end = std::max(1,int(end_straight_len_/spacing_));
        for(int i=1;i<=N_end;i++)
        {
            double t = double(i)/N_end;
            double x = x1_virtual + t*(x1 - x1_virtual);
            double y = y1_virtual + t*(y1 - y1_virtual);
            path.push_back({x,y});
        }
    }

    return true;
}