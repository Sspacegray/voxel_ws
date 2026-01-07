#pragma once

#include "industrial_route_core/graph.hpp"

#include <string>

namespace industrial_route_core {

class GeoJsonLoader {
public:
  struct LoadResult {
    bool success{false};
    std::string error_msg;
  };

  static LoadResult loadFromFile(const std::string& path, Graph* graph);
};

}  // namespace industrial_route_core

