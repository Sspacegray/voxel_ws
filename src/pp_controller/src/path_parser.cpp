#include "pp_controller/path_parser.hpp"
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdexcept>

// 简单的 JSON 值提取帮助函数
namespace
{

std::string trim(const std::string& str)
{
    size_t first = str.find_first_not_of(" \t\n\r\"");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\n\r\"");
    return str.substr(first, last - first + 1);
}

int extractInt(const std::string& json, const std::string& key)
{
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) throw std::runtime_error("Key not found: " + key);
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) throw std::runtime_error("Invalid JSON format");
    
    size_t start = json.find_first_of("-0123456789", pos);
    size_t end = json.find_first_not_of("-0123456789", start);
    
    return std::stoi(json.substr(start, end - start));
}

double extractDouble(const std::string& json, const std::string& key)
{
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) throw std::runtime_error("Key not found: " + key);
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) throw std::runtime_error("Invalid JSON format");
    
    size_t start = json.find_first_of("-0123456789.", pos);
    size_t end = json.find_first_not_of("-0123456789.", start);
    
    return std::stod(json.substr(start, end - start));
}

std::string extractString(const std::string& json, const std::string& key)
{
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) throw std::runtime_error("Key not found: " + key);
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) throw std::runtime_error("Invalid JSON format");
    
    size_t start = json.find("\"", pos + 1);
    if (start == std::string::npos) throw std::runtime_error("Invalid JSON format");
    
    size_t end = json.find("\"", start + 1);
    if (end == std::string::npos) throw std::runtime_error("Invalid JSON format");
    
    return json.substr(start + 1, end - start - 1);
}

}  // namespace

namespace pp_controller
{

PathTask PathParser::loadFromFile(const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + file_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string json = buffer.str();

    PathTask task;
    
    // 提取 task_id
    try {
        task.task_id = extractString(json, "task_id");
    } catch (...) {
        task.task_id = "unknown";
    }

    // 查找 paths 数组
    size_t paths_start = json.find("\"paths\"");
    if (paths_start == std::string::npos) {
        throw std::runtime_error("No 'paths' array found in JSON");
    }

    // 解析每个路径段
    size_t pos = json.find("[", paths_start);
    size_t end_pos = json.rfind("]");
    
    if (pos == std::string::npos || end_pos == std::string::npos) {
        throw std::runtime_error("Invalid paths array format");
    }

    std::string paths_content = json.substr(pos + 1, end_pos - pos - 1);
    
    // 按 '}' 分割路径段
    size_t segment_start = 0;
    while (true) {
        size_t obj_start = paths_content.find("{", segment_start);
        if (obj_start == std::string::npos) break;
        
        size_t obj_end = paths_content.find("}", obj_start);
        if (obj_end == std::string::npos) break;
        
        // 查找匹配的结束括号 (处理嵌套对象)
        int brace_count = 1;
        size_t search_pos = obj_start + 1;
        while (brace_count > 0 && search_pos < paths_content.size()) {
            if (paths_content[search_pos] == '{') brace_count++;
            else if (paths_content[search_pos] == '}') brace_count--;
            search_pos++;
        }
        obj_end = search_pos - 1;
        
        std::string segment_json = paths_content.substr(obj_start, obj_end - obj_start + 1);
        
        PathSegment segment;
        segment.dir = extractInt(segment_json, "dir");
        segment.target_v = extractDouble(segment_json, "target_v");
        segment.start_spin = extractInt(segment_json, "start_spin");
        
        // 提取 start_point
        size_t sp_pos = segment_json.find("\"start_point\"");
        if (sp_pos != std::string::npos) {
            size_t sp_obj_start = segment_json.find("{", sp_pos);
            size_t sp_obj_end = segment_json.find("}", sp_obj_start);
            std::string sp_json = segment_json.substr(sp_obj_start, sp_obj_end - sp_obj_start + 1);
            segment.start_point.x = extractInt(sp_json, "x");
            segment.start_point.y = extractInt(sp_json, "y");
        }
        
        // 提取 end_point
        size_t ep_pos = segment_json.find("\"end_point\"");
        if (ep_pos != std::string::npos) {
            size_t ep_obj_start = segment_json.find("{", ep_pos);
            size_t ep_obj_end = segment_json.find("}", ep_obj_start);
            std::string ep_json = segment_json.substr(ep_obj_start, ep_obj_end - ep_obj_start + 1);
            segment.end_point.x = extractInt(ep_json, "x");
            segment.end_point.y = extractInt(ep_json, "y");
        }
        
        task.paths.push_back(segment);
        segment_start = obj_end + 1;
    }

    return task;
}

double PathParser::calculateTargetYaw(const PathSegment& segment)
{
    double dx = segment.end_point.x - segment.start_point.x;
    double dy = segment.end_point.y - segment.start_point.y;
    return std::atan2(dy, dx);
}

}  // namespace pp_controller
