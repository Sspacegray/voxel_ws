#ifndef PP_CONTROLLER__SPIN_CONTROLLER_HPP_
#define PP_CONTROLLER__SPIN_CONTROLLER_HPP_

#include <tuple>
#include "pp_controller/math_utils.hpp"

namespace pp_controller
{

/**
 * @brief 原地旋转控制器
 */
class SpinController
{
public:
    SpinController(double kp, double max_omega, double angle_tolerance)
        : kp_(kp), max_omega_(max_omega), angle_tolerance_(angle_tolerance) {}

    /**
     * @brief 计算旋转控制
     * @param current_yaw 当前航向角 (rad)
     * @param target_yaw 目标航向角 (rad)
     * @return {vx, omega, is_done} 线速度, 角速度, 是否完成
     */
    std::tuple<double, double, bool> compute(double current_yaw, double target_yaw)
    {
        double angle_diff = normalizeAngle(target_yaw - current_yaw);

        if (std::abs(angle_diff) < angle_tolerance_) {
            return {0.0, 0.0, true};  // 旋转完成
        }

        // P控制 + 速度限幅
        double omega = clamp(kp_ * angle_diff, -max_omega_, max_omega_);

        return {0.0, omega, false};
    }

private:
    double kp_;
    double max_omega_;
    double angle_tolerance_;
};

}  // namespace pp_controller

#endif  // PP_CONTROLLER__SPIN_CONTROLLER_HPP_
