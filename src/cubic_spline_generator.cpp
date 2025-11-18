#include "spline_planner/cubic_spline_generator.hpp"

bool CubicSplineGenerator::generate(
    const std::vector<double> &start,
    const std::vector<double> &end,
    std::vector<std::vector<double>> &path,
    bool forward)
{
    if (start.size() != 3 || end.size() != 3)
        return false;

    double x0 = start[0];
    double y0 = start[1];
    double yaw0 = start[2];

    double x1 = end[0];
    double y1 = end[1];
    double yaw1 = end[2];

    // 如果 backward，则反向 yaw
    if (!forward)
    {
        yaw0 += M_PI;
        yaw1 += M_PI;
    }

    double dx = x1 - x0;
    double dy = y1 - y0;
    double total_dist = std::hypot(dx, dy);

    if (total_dist < 1e-6)
        return false;

    // ----------- 构造贝塞尔控制点 -----------
    double p0x = x0;
    double p0y = y0;

    double p1x = x0 + std::cos(yaw0) * total_dist * ctrl_ratio_ * 0.5;
    double p1y = y0 + std::sin(yaw0) * total_dist * ctrl_ratio_ * 0.5;

    double p2x = x1 - std::cos(yaw1) * total_dist * ctrl_ratio_;
    double p2y = y1 - std::sin(yaw1) * total_dist * ctrl_ratio_;

    double p3x = x1;
    double p3y = y1;

    // ----------- 高密采样（计算弧长）-----------
    int N_tmp = 300;   // 越大越精确
    std::vector<double> tx(N_tmp+1), ty(N_tmp+1), s(N_tmp+1);
    double total_len = 0.0;

    auto bezier = [&](double t, double &x, double &y) {
        double B0 = std::pow(1-t, 3);
        double B1 = 3 * std::pow(1-t, 2) * t;
        double B2 = 3 * (1-t) * t * t;
        double B3 = t * t * t;
        x = B0 * p0x + B1 * p1x + B2 * p2x + B3 * p3x;
        y = B0 * p0y + B1 * p1y + B2 * p2y + B3 * p3y;
    };

    bezier(0, tx[0], ty[0]);
    s[0] = 0.0;

    for (int i=1; i<=N_tmp; i++)
    {
        double t = double(i) / N_tmp;
        bezier(t, tx[i], ty[i]);

        double dx = tx[i] - tx[i-1];
        double dy = ty[i] - ty[i-1];
        total_len += std::hypot(dx, dy);
        s[i] = total_len;
    }

    // ----------- 等距采样 -----------
    int N = std::max(1, int(total_len / step_dist_));
    path.clear();
    path.reserve(N+1);

    for (int k=0; k<=N; k++)
    {
        double target_s = k * step_dist_;
        if (target_s > total_len)
            target_s = total_len;

        // 找 s[] 中对应区间
        int idx = 1;
        while (idx < N_tmp && s[idx] < target_s)
            idx++;

        // 线性插值 s → t
        double s_low = s[idx-1];
        double s_high = s[idx];
        double t_low = double(idx-1) / N_tmp;
        double t_high = double(idx) / N_tmp;

        double t = t_low + 
            (target_s - s_low) / (s_high - s_low) * (t_high - t_low);

        double x, y;
        bezier(t, x, y);

        path.push_back({x, y});
    }

    return true;
}


// #include "bezier_curve_planner/cubic_spline_generator.hpp"

// bool CubicSplineGenerator::generate(
//     const std::vector<double> &start,
//     const std::vector<double> &end,
//     std::vector<std::vector<double>> &path,
//     bool forward)
// {
//     if (start.size() != 3 || end.size() != 3)
//         return false;

//     double x0 = start[0];
//     double y0 = start[1];
//     double yaw0 = start[2];

//     double x1 = end[0];
//     double y1 = end[1];
//     double yaw1 = end[2];

//     if (!forward)
//     {
//         yaw0 += M_PI;
//         yaw1 += M_PI;
//     }

//     double dx = x1 - x0;
//     double dy = y1 - y0;
//     double total_dist = std::hypot(dx, dy);

//     if (total_dist < 1e-6)
//         return false;

//     double p0x = x0;
//     double p0y = y0;

//     // 控制点，用 yaw 作为切线方向
//     double ctrl_ratio = 0.6;  // 控制点距离比例，越大终点越直

//     double p1x = x0 + std::cos(yaw0) * total_dist * ctrl_ratio * 0.5;
//     double p1y = y0 + std::sin(yaw0) * total_dist * ctrl_ratio * 0.5;

//     double p2x = x1 - std::cos(yaw1) * total_dist * ctrl_ratio;
//     double p2y = y1 - std::sin(yaw1) * total_dist * ctrl_ratio;


//     double p3x = x1;
//     double p3y = y1;

//     // 采样
//     int N = std::max(2, int(total_dist / step_dist_));

//     for (int i = 0; i <= N; i++)
//     {
//         double t = double(i) / N;

//         // cubic Bezier curve
//         double B0 = std::pow(1 - t, 3);
//         double B1 = 3 * std::pow(1 - t, 2) * t;
//         double B2 = 3 * (1 - t) * t * t;
//         double B3 = t * t * t;

//         double x = B0 * p0x + B1 * p1x + B2 * p2x + B3 * p3x;
//         double y = B0 * p0y + B1 * p1y + B2 * p2y + B3 * p3y;

//         std::vector<double> waypoint;
//         waypoint.push_back(x);
//         waypoint.push_back(y);
//         path.push_back(waypoint);
//     }

//     return true;
// }
