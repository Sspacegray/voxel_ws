#pragma once

#include "industrial_route_core/types.hpp"

#include <stdexcept>
#include <unordered_map>

namespace industrial_route_core {

class Graph {
public:
  void clear();

  void addNode(Node node);
  void addEdge(Edge edge);

  bool hasNode(const std::string& id) const;
  bool hasEdge(const std::string& id) const;

  const Node& node(const std::string& id) const;
  const Edge& edge(const std::string& id) const;

  std::vector<std::string> nodeIds() const;
  std::vector<std::string> edgeIds() const;

  std::vector<EdgeTraversal> outgoing(const std::string& from_id, const PlannerOptions& opts) const;

  void setEdgeEnabled(const std::string& edge_id, bool enabled);
  void setEdgeMaxSpeed(const std::string& edge_id, double max_speed);
  void setEdgeBidirectional(const std::string& edge_id, bool bidirectional);
  void setEdgeWeight(const std::string& edge_id, double weight);

  double edgeLength(const Edge& e) const;
  std::vector<Point2D> edgePolylineDirected(const Edge& e, const std::string& from_id, const std::string& to_id) const;

private:
  std::unordered_map<std::string, Node> nodes_;
  std::unordered_map<std::string, Edge> edges_;
  std::unordered_map<std::string, std::vector<std::string>> adjacency_;
};

}  // namespace industrial_route_core

