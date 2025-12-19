#ifndef PP_CONTROLLER__LINE_TRACKER_HPP_
#define PP_CONTROLLER__LINE_TRACKER_HPP_

#include <tuple>
#include <cmath>
#include "pp_controller/math_utils.hpp"
#include "pp_controller/path_parser.hpp"

namespace pp_controller
{

/**
 * @brief 控制输出结构
 */
struct ControlOutput
{
    double vx;              // 线速度
    double omega;           // 角速度
    bool is_done;           // 是否到达终点
    bool lateral_warning;   // 横向偏差警告
    double lateral_dev;     // 横向偏差值
    double curvature;       // 曲率值
};

/**
 * @brief 直线跟踪控制器 (Regulated Pure Pursuit)
 * 
 * 参考 Nav2 RPP 算法增强：
 * - 最大前瞻距离限制
 * - 曲率调节减速
 * - 横向偏差检测
 */
class LineTracker
{
public:
    LineTracker(double lookahead_gain, double lookahead_min, double lookahead_max,
                double max_omega, double position_tolerance, double min_vel,
                double curvature_min_radius, double max_lateral_deviation)
        : lookahead_gain_(lookahead_gain)
        , lookahead_min_(lookahead_min)
        , lookahead_max_(lookahead_max)
        , max_omega_(max_omega)
        , position_tolerance_(position_tolerance)
        , min_vel_(min_vel)
        , curvature_min_radius_(curvature_min_radius)
        , max_lateral_deviation_(max_lateral_deviation)
        , use_curvature_regulation_(true)
    {}

    /**
     * @brief 计算直线跟踪控制
     * @param current_x 当前X位置 (m)
     * @param current_y 当前Y位置 (m)
     * @param current_yaw 当前航向角 (rad)
     * @param current_vel 当前速度 (m/s) 用于自适应前瞻
     * @param segment 当前路径段
     * @param cost_factor Costmap 代价因子 (0-1, 1=无代价)
     * @return ControlOutput 控制输出
     */
    ControlOutput compute(
        double current_x, double current_y, double current_yaw,
        double current_vel,
        const PathSegment& segment,
        double cost_factor = 1.0)
    {
        ControlOutput output{0.0, 0.0, false, false, 0.0, 0.0};

        // 转换坐标到米
        double start_x = PathParser::mmToMeters(segment.start_point.x);
        double start_y = PathParser::mmToMeters(segment.start_point.y);
        double end_x = PathParser::mmToMeters(segment.end_point.x);
        double end_y = PathParser::mmToMeters(segment.end_point.y);

        // 1. 计算路径方向向量
        double path_dx = end_x - start_x;
        double path_dy = end_y - start_y;
        double path_length = std::hypot(path_dx, path_dy);
        
        if (path_length < 1e-6) {
            output.is_done = true;
            return output;
        }

        // 2. 计算横向偏差
        output.lateral_dev = computeLateralDeviation(
            current_x, current_y, start_x, start_y, end_x, end_y);
        
        if (output.lateral_dev > max_lateral_deviation_) {
            output.lateral_warning = true;
            // 超限停止
            output.is_done = false;
            return output;
        }

        // 3. 到点判断
        double dist_to_end = distance(current_x, current_y, end_x, end_y);
        if (dist_to_end < position_tolerance_) {
            output.is_done = true;
            return output;
        }

        // 4. 计算沿路径的投影距离（已行驶距离）
        double to_current_x = current_x - start_x;
        double to_current_y = current_y - start_y;
        double progress = (to_current_x * path_dx + to_current_y * path_dy) / path_length;

        // 5. 计算前瞻距离 (带上限限制)
        double ld = clamp(
            lookahead_gain_ * std::abs(current_vel) + lookahead_min_,
            lookahead_min_,
            lookahead_max_
        );

        // 6. 计算前瞻点
        double lookahead_progress = std::min(progress + ld, path_length);
        double lookahead_x = start_x + (lookahead_progress / path_length) * path_dx;
        double lookahead_y = start_y + (lookahead_progress / path_length) * path_dy;

        // 7. Pure Pursuit 计算曲率
        double dx = lookahead_x - current_x;
        double dy = lookahead_y - current_y;
        double alpha = std::atan2(dy, dx) - current_yaw;

        // 倒退时航向反转
        if (segment.dir == -1) {
            alpha = normalizeAngle(alpha + M_PI);
        }

        double curvature = 2.0 * std::sin(alpha) / ld;
        output.curvature = curvature;

        // 8. 基础速度 (S曲线规划)
        double base_vel = sCurveVelocity(progress, path_length, segment.target_v);

        // 9. 曲率调节减速
        double curvature_factor = 1.0;
        if (use_curvature_regulation_) {
            curvature_factor = 1.0 / (1.0 + std::abs(curvature) * curvature_min_radius_);
        }

        // 10. Costmap 代价调节
        double regulated_vel = base_vel * curvature_factor * cost_factor;

        // 11. 确保最小速度
        regulated_vel = std::max(regulated_vel, min_vel_);

        // 12. 计算输出
        output.vx = regulated_vel * segment.dir;
        output.omega = output.vx * curvature;

        // 13. 角速度限幅
        output.omega = clamp(output.omega, -max_omega_, max_omega_);

        return output;
    }

    void setUseCurvatureRegulation(bool use) { use_curvature_regulation_ = use; }

private:
    /**
     * @brief 计算点到直线的垂直距离（横向偏差）
     */
    double computeLateralDeviation(double x, double y, 
                                   double x1, double y1, double x2, double y2)
    {
        double line_length = std::hypot(x2 - x1, y2 - y1);
        if (line_length < 1e-6) return 0.0;
        
        // 点到直线距离公式: |Ax + By + C| / sqrt(A² + B²)
        return std::abs((y2-y1)*x - (x2-x1)*y + x2*y1 - y2*x1) / line_length;
    }

    /**
     * @brief S曲线速度规划
     */
    double sCurveVelocity(double progress, double total_length, double target_v)
    {
        const double acc_ratio = 0.2;  // 加速段占比
        const double dec_ratio = 0.2;  // 减速段占比

        double acc_dist = total_length * acc_ratio;
        double dec_dist = total_length * dec_ratio;

        double v;
        if (progress < 0) {
            // 还未进入路径
            v = min_vel_;
        } else if (progress < acc_dist) {
            // 加速段
            double t = progress / acc_dist;
            v = target_v * smoothStep(t);
        } else if (progress > total_length - dec_dist) {
            // 减速段
            double t = (total_length - progress) / dec_dist;
            v = target_v * smoothStep(t);
        } else {
            // 匀速段
            v = target_v;
        }

        return std::max(v, min_vel_);
    }

    double lookahead_gain_;
    double lookahead_min_;
    double lookahead_max_;
    double max_omega_;
    double position_tolerance_;
    double min_vel_;
    double curvature_min_radius_;
    double max_lateral_deviation_;
    bool use_curvature_regulation_;
};

}  // namespace pp_controller

#endif  // PP_CONTROLLER__LINE_TRACKER_HPP_
