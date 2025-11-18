#pragma once
#include <vector>
#include <cmath>

class QuinticSpline2D
{
public:
    QuinticSpline2D(double spacing=0.05, double end_straight_len=0.3)
        : spacing_(spacing), end_straight_len_(end_straight_len) {}

    bool generate(
        const std::vector<double> &start,
        const std::vector<double> &end,
        std::vector<std::vector<double>> &path,
        bool forward=true);

private:
    double spacing_;
    double end_straight_len_;
};

// #pragma once
// #include <vector>
// #include <cmath>

// class QuinticSpline2D
// {
// public:
//     QuinticSpline2D(double spacing=0.05) : spacing_(spacing) {}

//     // start/end: {x, y, yaw}
//     bool generate(
//         const std::vector<double> &start,
//         const std::vector<double> &end,
//         std::vector<std::vector<double>> &path,
//         bool forward=true);

// private:
//     double spacing_;
// };
