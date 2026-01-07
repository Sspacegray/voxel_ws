#include "industrial_route_core/geojson_loader.hpp"

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: industrial_route_validate_graph <path.geojson>\n";
    return 2;
  }

  const std::string path = argv[1];
  industrial_route_core::Graph graph;
  const auto res = industrial_route_core::GeoJsonLoader::loadFromFile(path, &graph);
  if (!res.success) {
    std::cerr << "ERROR: failed to load graph: " << res.error_msg << "\n";
    return 1;
  }

  std::cout << "OK: loaded graph\n";
  std::cout << "  nodes: " << graph.nodeIds().size() << "\n";
  std::cout << "  edges: " << graph.edgeIds().size() << "\n";

  if (graph.nodeIds().empty() || graph.edgeIds().empty()) {
    std::cerr << "WARN: graph is empty or has no edges\n";
    return 3;
  }
  return 0;
}

