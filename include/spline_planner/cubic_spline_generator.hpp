#pragma once
#include <vector>
#include <cmath>

class CubicSplineGenerator
{
public:
    CubicSplineGenerator(double step_dist = 0.05, double ctrl_ratio = 0.6)
        : step_dist_(step_dist), ctrl_ratio_(ctrl_ratio) {}

    void setStepDistance(double d) { step_dist_ = d; }
    void setControlRatio(double r) { ctrl_ratio_ = r; }

    bool generate(
        const std::vector<double> &start,   // {x, y, yaw}
        const std::vector<double> &end,     // {x, y, yaw}
        std::vector<std::vector<double>> &path,   // [[x,y], [x,y] ...]
        bool forward = true);

private:
    double step_dist_;
    double ctrl_ratio_;
};

// #pragma once
// #include <vector>
// #include <cmath>
// #include <Eigen/Dense>

// class CubicSplineGenerator
// {
// public:
//     CubicSplineGenerator(double step_dist = 0.05)
//         : step_dist_(step_dist) {}

//     void setStepDistance(double d) { step_dist_ = d; }

//     bool generate(
//         const std::vector<double> &start,  // x, y, yaw
//         const std::vector<double> &end,    // x, y, yaw
//         std::vector<std::vector<double>> &path, // x_list, y_list
//         bool forward = true);

// private:
//     double step_dist_;
// };
