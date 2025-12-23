#ifndef WAYPOINT_EDITOR__IO__WAYPOINT_JSON_HPP_
#define WAYPOINT_EDITOR__IO__WAYPOINT_JSON_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include "waypoint_editor/core/waypoint.hpp"

namespace waypoint_editor::io
{

/**
 * @brief JSON 路径文件 I/O 类
 * 
 * 将 waypoint 序列转换为 pp_controller 可用的 JSON 路径格式
 */
class WaypointJson
{
public:
    /**
     * @brief 保存 waypoints 为 JSON 路径文件
     * 
     * 输出格式:
     * {
     *   "task_id": "path_YYYYMMDD_HHMMSS",
     *   "paths": [
     *     {
     *       "dir": 1,
     *       "target_v": 0.5,
     *       "start_spin": 1,
     *       "start_point": { "x": 0, "y": 0 },
     *       "end_point": { "x": 2000, "y": 0 }
     *     },
     *     ...
     *   ]
     * }
     * 
     * @param waypoints Waypoint 序列 (坐标单位: m)
     * @param path 输出文件路径
     * @param error 错误信息
     * @param target_v 目标速度 (m/s), 默认 0.5
     * @return true 保存成功
     */
    static bool Save(
        const std::vector<Waypoint>& waypoints,
        const std::string& path,
        std::string& error,
        double target_v = 0.5);

    /**
     * @brief 从 JSON 路径文件加载 waypoints
     * @param path 输入文件路径
     * @param waypoints 输出 Waypoint 序列
     * @param error 错误信息
     * @return true 加载成功
     */
    static bool Load(
        const std::string& path,
        std::vector<Waypoint>& waypoints,
        std::string& error);

    /**
     * @brief 将米转换为毫米
     */
    static int metersToMm(double meters) { return static_cast<int>(meters * 1000.0); }
    
    /**
     * @brief 将毫米转换为米
     */
    static double mmToMeters(int mm) { return static_cast<double>(mm) / 1000.0; }
};

}  // namespace waypoint_editor::io

#endif  // WAYPOINT_EDITOR__IO__WAYPOINT_JSON_HPP_
