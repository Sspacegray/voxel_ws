#include "industrial_route_core/geojson_loader.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

namespace industrial_route_core {
namespace {

std::optional<double> jsonNumber(const nlohmann::json& j) {
  if (j.is_number_float()) return j.get<double>();
  if (j.is_number_integer()) return static_cast<double>(j.get<int64_t>());
  if (j.is_number_unsigned()) return static_cast<double>(j.get<uint64_t>());
  return std::nullopt;
}

std::optional<bool> jsonBool(const nlohmann::json& j) {
  if (j.is_boolean()) return j.get<bool>();
  if (j.is_number_integer()) return j.get<int64_t>() != 0;
  if (j.is_number_unsigned()) return j.get<uint64_t>() != 0u;
  if (j.is_string()) {
    const auto s = j.get<std::string>();
    if (s == "true" || s == "TRUE" || s == "1") return true;
    if (s == "false" || s == "FALSE" || s == "0") return false;
  }
  return std::nullopt;
}

std::string jsonStringOr(const nlohmann::json& j, const std::string& fallback) {
  if (j.is_string()) return j.get<std::string>();
  if (j.is_number_integer()) return std::to_string(j.get<int64_t>());
  if (j.is_number_unsigned()) return std::to_string(j.get<uint64_t>());
  return fallback;
}

}  // namespace

GeoJsonLoader::LoadResult GeoJsonLoader::loadFromFile(const std::string& path, Graph* graph) {
  if (!graph) return {false, "null graph"};

  std::ifstream in(path);
  if (!in) return {false, "failed to open file: " + path};

  nlohmann::json root;
  try {
    in >> root;
  } catch (const std::exception& e) {
    return {false, std::string("failed to parse json: ") + e.what()};
  }

  if (!root.contains("features") || !root["features"].is_array()) {
    return {false, "GeoJSON missing features[]"};
  }

  graph->clear();

  // Pass 1: nodes
  for (const auto& f : root["features"]) {
    if (!f.contains("geometry") || !f.contains("properties")) continue;
    const auto& geom = f["geometry"];
    const auto& props = f["properties"];
    if (!geom.contains("type") || !geom["type"].is_string()) continue;
    const auto type = geom["type"].get<std::string>();
    if (type != "Point") continue;
    if (!geom.contains("coordinates") || !geom["coordinates"].is_array() || geom["coordinates"].size() < 2) continue;
    const auto x = jsonNumber(geom["coordinates"][0]);
    const auto y = jsonNumber(geom["coordinates"][1]);
    if (!x || !y) continue;

    std::string id;
    if (props.contains("id")) {
      id = jsonStringOr(props["id"], "");
    }
    if (id.empty()) continue;

    Node node;
    node.id = id;
    node.position = Point2D{*x, *y};
    if (props.contains("theta")) {
      if (auto t = jsonNumber(props["theta"])) node.theta = *t;
    }
    if (props.contains("type")) node.type = jsonStringOr(props["type"], "NORMAL");

    for (auto it = props.begin(); it != props.end(); ++it) {
      if (!it.key().empty()) {
        if (it.value().is_string()) node.properties[it.key()] = it.value().get<std::string>();
        else if (it.value().is_number()) node.properties[it.key()] = jsonStringOr(it.value(), "");
      }
    }

    graph->addNode(std::move(node));
  }

  // Pass 2: edges
  for (const auto& f : root["features"]) {
    if (!f.contains("geometry") || !f.contains("properties")) continue;
    const auto& geom = f["geometry"];
    const auto& props = f["properties"];
    if (!geom.contains("type") || !geom["type"].is_string()) continue;
    const auto type = geom["type"].get<std::string>();
    if (type != "MultiLineString" && type != "LineString") continue;

    std::string id;
    if (props.contains("id")) id = jsonStringOr(props["id"], "");
    if (id.empty()) continue;

    std::string start_id;
    std::string end_id;
    if (props.contains("startid")) start_id = jsonStringOr(props["startid"], "");
    if (props.contains("endid")) end_id = jsonStringOr(props["endid"], "");

    std::vector<Point2D> polyline;
    if (type == "LineString") {
      const auto& coords = geom["coordinates"];
      if (!coords.is_array() || coords.size() < 2) continue;
      for (const auto& c : coords) {
        if (!c.is_array() || c.size() < 2) continue;
        const auto x = jsonNumber(c[0]);
        const auto y = jsonNumber(c[1]);
        if (!x || !y) continue;
        polyline.push_back(Point2D{*x, *y});
      }
    } else {
      const auto& lines = geom["coordinates"];
      if (!lines.is_array() || lines.empty()) continue;
      const auto& coords = lines[0];
      if (!coords.is_array() || coords.size() < 2) continue;
      for (const auto& c : coords) {
        if (!c.is_array() || c.size() < 2) continue;
        const auto x = jsonNumber(c[0]);
        const auto y = jsonNumber(c[1]);
        if (!x || !y) continue;
        polyline.push_back(Point2D{*x, *y});
      }
    }

    // Infer endpoints if missing
    if (start_id.empty() && polyline.size() >= 1) {
      for (const auto& nid : graph->nodeIds()) {
        if (distance(graph->node(nid).position, polyline.front()) < 1e-6) {
          start_id = nid;
          break;
        }
      }
    }
    if (end_id.empty() && polyline.size() >= 1) {
      for (const auto& nid : graph->nodeIds()) {
        if (distance(graph->node(nid).position, polyline.back()) < 1e-6) {
          end_id = nid;
          break;
        }
      }
    }

    if (start_id.empty() || end_id.empty()) {
      return {false, "edge " + id + " missing startid/endid and could not infer"};
    }

    Edge edge;
    edge.id = id;
    edge.start_id = start_id;
    edge.end_id = end_id;
    edge.polyline = std::move(polyline);

    if (props.contains("bidirectional")) {
      if (auto b = jsonBool(props["bidirectional"])) {
        edge.bidirectional = *b;
      }
    }
    if (props.contains("weight")) {
      if (auto w = jsonNumber(props["weight"])) edge.weight = *w;
    }
    if (props.contains("max_speed")) {
      if (auto ms = jsonNumber(props["max_speed"])) edge.max_speed = *ms;
    }
    if (props.contains("enabled")) {
      if (auto b = jsonBool(props["enabled"])) {
        edge.enabled = *b;
      }
    }

    for (auto it = props.begin(); it != props.end(); ++it) {
      if (!it.key().empty()) {
        if (it.value().is_string()) edge.properties[it.key()] = it.value().get<std::string>();
        else if (it.value().is_number()) edge.properties[it.key()] = jsonStringOr(it.value(), "");
      }
    }

    try {
      graph->addEdge(std::move(edge));
    } catch (const std::exception& e) {
      return {false, std::string("failed to add edge ") + id + ": " + e.what()};
    }
  }

  return {true, ""};
}

}  // namespace industrial_route_core
