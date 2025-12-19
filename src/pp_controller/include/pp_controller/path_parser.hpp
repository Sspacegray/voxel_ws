#ifndef PP_CONTROLLER__PATH_PARSER_HPP_
#define PP_CONTROLLER__PATH_PARSER_HPP_

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pp_controller
{

/**
 * @brief 路径点结构
 */
struct Point
{
    int x;  // 毫米
    int y;  // 毫米
};

/**
 * @brief 路径段结构
 */
struct PathSegment
{
    int dir;            // 移动方向: 1=正向, -1=倒退
    double target_v;    // 目标速度 (m/s)
    int start_spin;     // 起始旋转: 1=启用, 0=禁用
    Point start_point;  // 起点 (mm)
    Point end_point;    // 终点 (mm)
};

/**
 * @brief 路径任务结构
 */
struct PathTask
{
    std::string task_id;
    std::vector<PathSegment> paths;
};

/**
 * @brief JSON 路径解析器
 * 
 * 简单的 JSON 解析器，用于解析路径文件
 */
class PathParser
{
public:
    /**
     * @brief 从文件加载路径
     * @param file_path 文件路径
     * @return 解析后的路径任务
     */
    static PathTask loadFromFile(const std::string& file_path);

    /**
     * @brief 将毫米坐标转换为米
     */
    static double mmToMeters(int mm) { return static_cast<double>(mm) / 1000.0; }

    /**
     * @brief 计算路径段的目标航向角
     */
    static double calculateTargetYaw(const PathSegment& segment);
};

}  // namespace pp_controller

#endif  // PP_CONTROLLER__PATH_PARSER_HPP_
