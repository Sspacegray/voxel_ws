#ifndef PP_CONTROLLER__STATE_MACHINE_HPP_
#define PP_CONTROLLER__STATE_MACHINE_HPP_

#include <string>

namespace pp_controller
{

/**
 * @brief 控制器状态枚举
 */
enum class ControllerState
{
    IDLE,           // 待命
    SPINNING,       // 原地旋转
    LINE_TRACKING,  // 直线跟踪
    COMPLETED       // 完成
};

/**
 * @brief 将状态转换为字符串
 */
inline std::string stateToString(ControllerState state)
{
    switch (state) {
        case ControllerState::IDLE:          return "IDLE";
        case ControllerState::SPINNING:      return "SPINNING";
        case ControllerState::LINE_TRACKING: return "LINE_TRACKING";
        case ControllerState::COMPLETED:     return "COMPLETED";
        default:                             return "UNKNOWN";
    }
}

}  // namespace pp_controller

#endif  // PP_CONTROLLER__STATE_MACHINE_HPP_
