#include "waypoint_editor/io/waypoint_json.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <cmath>

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

bool WaypointJson::Load(
    const std::string& path,
    std::vector<Waypoint>& waypoints,
    std::string& error)
{
    waypoints.clear();
    try {
        YAML::Node config = YAML::LoadFile(path);
        
        if (!config["paths"]) {
            error = "JSON format error: 'paths' key missing";
            return false;
        }

        const auto& paths = config["paths"];
        if (!paths.IsSequence()) {
            error = "JSON format error: 'paths' is not a sequence";
            return false;
        }

        for (size_t i = 0; i < paths.size(); ++i) {
            const auto& segment = paths[i];
            
            // Start Point
            if (segment["start_point"] && segment["start_point"]["x"] && segment["start_point"]["y"]) {
                int mm_x = segment["start_point"]["x"].as<int>();
                int mm_y = segment["start_point"]["y"].as<int>();
                
                Waypoint wp;
                wp.pose.pose.position.x = mmToMeters(mm_x);
                wp.pose.pose.position.y = mmToMeters(mm_y);
                wp.pose.pose.position.z = 0.0;
                // Orientation usually default (0)
                wp.pose.pose.orientation.w = 1.0;
                
                waypoints.push_back(wp);
            }

            // If it is the LAST segment, also add the End Point
            if (i == paths.size() - 1) {
                if (segment["end_point"] && segment["end_point"]["x"] && segment["end_point"]["y"]) {
                    int mm_x = segment["end_point"]["x"].as<int>();
                    int mm_y = segment["end_point"]["y"].as<int>();
                    
                    Waypoint wp;
                    wp.pose.pose.position.x = mmToMeters(mm_x);
                    wp.pose.pose.position.y = mmToMeters(mm_y);
                    wp.pose.pose.position.z = 0.0;
                    wp.pose.pose.orientation.w = 1.0;
                    
                    waypoints.push_back(wp);
                }
            }
        }
    } catch (const YAML::Exception& e) {
        error = "Failed to parse JSON/YAML: " + std::string(e.what());
        return false;
    } catch (const std::exception& e) {
        error = "Error: " + std::string(e.what());
        return false;
    }

    if (waypoints.empty()) {
        error = "No waypoints found in file";
        return false;
    }

    // Compute orientations based on path geometry
    for (size_t i = 0; i < waypoints.size(); ++i) {
        double yaw = 0.0;
        if (i < waypoints.size() - 1) {
            // Point towards next waypoint
            double dx = waypoints[i+1].pose.pose.position.x - waypoints[i].pose.pose.position.x;
            double dy = waypoints[i+1].pose.pose.position.y - waypoints[i].pose.pose.position.y;
            yaw = std::atan2(dy, dx);
        } else if (i > 0) {
            // Last point: keep same orientation as previous segment
            // (already set by quaternion copy, but let's recalculate from prev point for consistency)
            double dx = waypoints[i].pose.pose.position.x - waypoints[i-1].pose.pose.position.x;
            double dy = waypoints[i].pose.pose.position.y - waypoints[i-1].pose.pose.position.y;
            yaw = std::atan2(dy, dx);
        }

        // Convert Yaw to Quaternion (z-axis rotation)
        // q = cos(theta/2) + k*sin(theta/2)
        waypoints[i].pose.pose.orientation.w = std::cos(yaw * 0.5);
        waypoints[i].pose.pose.orientation.x = 0.0;
        waypoints[i].pose.pose.orientation.y = 0.0;
        waypoints[i].pose.pose.orientation.z = std::sin(yaw * 0.5);
    }

    return true;
}

}  // namespace waypoint_editor::io
