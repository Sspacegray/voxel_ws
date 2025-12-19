#ifndef PP_CONTROLLER__MATH_UTILS_HPP_
#define PP_CONTROLLER__MATH_UTILS_HPP_

#include <cmath>
#include <algorithm>

namespace pp_controller
{

/**
 * @brief 将角度归一化到 [-π, π]
 */
inline double normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/**
 * @brief 限幅函数
 */
template<typename T>
inline T clamp(T value, T min_val, T max_val)
{
    return std::max(min_val, std::min(max_val, value));
}

/**
 * @brief 平滑阶跃函数 (S曲线)
 * t ∈ [0, 1] → output ∈ [0, 1]
 * 使用 smoothstep: 3t² - 2t³
 */
inline double smoothStep(double t)
{
    t = clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/**
 * @brief 计算两点之间的距离
 */
inline double distance(double x1, double y1, double x2, double y2)
{
    return std::hypot(x2 - x1, y2 - y1);
}

}  // namespace pp_controller

#endif  // PP_CONTROLLER__MATH_UTILS_HPP_
