#include "industrial_route_core/graph.hpp"

#include <algorithm>
#include <limits>

namespace industrial_route_core {

void Graph::clear() {
  nodes_.clear();
  edges_.clear();
  adjacency_.clear();
}

void Graph::addNode(Node node) {
  if (node.id.empty()) {
    throw std::invalid_argument("Graph::addNode: empty node id");
  }
  const std::string node_id = node.id;
  nodes_[node_id] = std::move(node);
  adjacency_.try_emplace(node_id, std::vector<std::string>{});
}

void Graph::addEdge(Edge edge) {
  if (edge.id.empty()) {
    throw std::invalid_argument("Graph::addEdge: empty edge id");
  }
  if (edge.start_id.empty() || edge.end_id.empty()) {
    throw std::invalid_argument("Graph::addEdge: empty start/end id");
  }
  if (!hasNode(edge.start_id) || !hasNode(edge.end_id)) {
    throw std::invalid_argument("Graph::addEdge: start/end node does not exist");
  }
  if (edge.polyline.size() < 2) {
    edge.polyline = {nodes_.at(edge.start_id).position, nodes_.at(edge.end_id).position};
  }
  const std::string edge_id = edge.id;
  edges_[edge_id] = std::move(edge);
  adjacency_[edges_.at(edge_id).start_id].push_back(edge_id);
  if (edges_.at(edge_id).bidirectional) {
    adjacency_[edges_.at(edge_id).end_id].push_back(edge_id);
  }
}

bool Graph::hasNode(const std::string& id) const { return nodes_.count(id) > 0; }
bool Graph::hasEdge(const std::string& id) const { return edges_.count(id) > 0; }

const Node& Graph::node(const std::string& id) const { return nodes_.at(id); }
const Edge& Graph::edge(const std::string& id) const { return edges_.at(id); }

std::vector<std::string> Graph::nodeIds() const {
  std::vector<std::string> ids;
  ids.reserve(nodes_.size());
  for (const auto& [id, _] : nodes_) ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::vector<std::string> Graph::edgeIds() const {
  std::vector<std::string> ids;
  ids.reserve(edges_.size());
  for (const auto& [id, _] : edges_) ids.push_back(id);
  std::sort(ids.begin(), ids.end());
  return ids;
}

double Graph::edgeLength(const Edge& e) const {
  if (e.polyline.size() < 2) return 0.0;
  double len = 0.0;
  for (std::size_t i = 1; i < e.polyline.size(); ++i) {
    len += distance(e.polyline[i - 1], e.polyline[i]);
  }
  return len;
}

std::vector<Point2D> Graph::edgePolylineDirected(const Edge& e, const std::string& from_id, const std::string& to_id) const {
  if (from_id == e.start_id && to_id == e.end_id) return e.polyline;
  if (from_id == e.end_id && to_id == e.start_id) {
    std::vector<Point2D> reversed = e.polyline;
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
  }
  return {node(from_id).position, node(to_id).position};
}

std::vector<EdgeTraversal> Graph::outgoing(const std::string& from_id, const PlannerOptions& opts) const {
  std::vector<EdgeTraversal> out;
  auto it = adjacency_.find(from_id);
  if (it == adjacency_.end()) return out;

  for (const auto& edge_id : it->second) {
    const auto& e = edges_.at(edge_id);
    if (!e.enabled) continue;

    std::string to_id;
    if (from_id == e.start_id) {
      to_id = e.end_id;
    } else if (from_id == e.end_id && e.bidirectional) {
      to_id = e.start_id;
    } else {
      continue;
    }

    const double len = edgeLength(e);
    double cost = len;
    if (opts.use_edge_weight) cost = e.weight * len;
    if (opts.use_max_speed_cost && e.max_speed > 1e-3) cost = len / e.max_speed;

    out.push_back(EdgeTraversal{edge_id, from_id, to_id, len, cost});
  }
  return out;
}

void Graph::setEdgeEnabled(const std::string& edge_id, bool enabled) {
  edges_.at(edge_id).enabled = enabled;
}
void Graph::setEdgeMaxSpeed(const std::string& edge_id, double max_speed) {
  edges_.at(edge_id).max_speed = max_speed;
}
void Graph::setEdgeBidirectional(const std::string& edge_id, bool bidirectional) {
  // adjacency needs rebuild; keep simple: update and rebuild adjacency list
  edges_.at(edge_id).bidirectional = bidirectional;
  adjacency_.clear();
  for (const auto& [id, _] : nodes_) adjacency_[id] = {};
  for (const auto& [id, edge] : edges_) {
    adjacency_[edge.start_id].push_back(id);
    if (edge.bidirectional) adjacency_[edge.end_id].push_back(id);
  }
}
void Graph::setEdgeWeight(const std::string& edge_id, double weight) {
  edges_.at(edge_id).weight = weight;
}

}  // namespace industrial_route_core
