#ifndef TRACKER_SIM_COMPAT_ANGLES_ANGLES_H
#define TRACKER_SIM_COMPAT_ANGLES_ANGLES_H

#include <cmath>

namespace angles
{

inline double normalize_angle(double angle)
{
    double two_pi = 2.0 * std::acos(-1.0);
    angle = std::fmod(angle + std::acos(-1.0), two_pi);
    if (angle < 0.0) {
        angle += two_pi;
    }
    return angle - std::acos(-1.0);
}

inline double normalize_angle_positive(double angle)
{
    double two_pi = 2.0 * std::acos(-1.0);
    angle = std::fmod(angle, two_pi);
    if (angle < 0.0) {
        angle += two_pi;
    }
    return angle;
}

inline double shortest_angular_distance(double from, double to)
{
    return normalize_angle(to - from);
}

inline double from_degrees(double degrees)
{
    return degrees * std::acos(-1.0) / 180.0;
}

inline double to_degrees(double radians)
{
    return radians * 180.0 / std::acos(-1.0);
}

} // namespace angles

#endif
