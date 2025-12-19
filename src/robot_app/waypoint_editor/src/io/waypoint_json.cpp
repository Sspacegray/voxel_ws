#include "waypoint_editor/io/waypoint_json.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>

namespace waypoint_editor::io
{

bool WaypointJson::Save(
    const std::vector<Waypoint>& waypoints,
    const std::string& path,
    std::string& error,
    double target_v)
{
    if (waypoints.size() < 2) {
        error = "需要至少 2 个 waypoint 才能生成路径";
        return false;
    }

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        error = "无法打开文件: " + path;
        return false;
    }

    // 生成 task_id
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* local_tm = std::localtime(&time_t_now);
    std::ostringstream task_id_ss;
    task_id_ss << "path_" 
               << std::put_time(local_tm, "%Y%m%d_%H%M%S");
    std::string task_id = task_id_ss.str();

    // 开始写入 JSON
    ofs << "{\n";
    ofs << "  \"task_id\": \"" << task_id << "\",\n";
    ofs << "  \"paths\": [\n";

    // 将 waypoints 转换为路径段
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const auto& start_wp = waypoints[i];
        const auto& end_wp = waypoints[i + 1];

        int start_x = metersToMm(start_wp.pose.pose.position.x);
        int start_y = metersToMm(start_wp.pose.pose.position.y);
        int end_x = metersToMm(end_wp.pose.pose.position.x);
        int end_y = metersToMm(end_wp.pose.pose.position.y);

        ofs << "    {\n";
        ofs << "      \"dir\": 1,\n";
        ofs << "      \"target_v\": " << std::fixed << std::setprecision(1) << target_v << ",\n";
        ofs << "      \"start_spin\": 1,\n";
        ofs << "      \"start_point\": { \"x\": " << start_x << ", \"y\": " << start_y << " },\n";
        ofs << "      \"end_point\": { \"x\": " << end_x << ", \"y\": " << end_y << " }\n";
        ofs << "    }";

        if (i < waypoints.size() - 2) {
            ofs << ",";
        }
        ofs << "\n";
    }

    ofs << "  ]\n";
    ofs << "}\n";

    error.clear();
    return true;
}

}  // namespace waypoint_editor::io
