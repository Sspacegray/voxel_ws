#include "industrial_route_core/planner.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>

namespace industrial_route_core {

PathPlan Planner::dijkstra(const Graph& graph,
                           const std::string& start_node,
                           const std::string& goal_node,
                           const PlannerOptions& opts) {
  PathPlan out;
  if (!graph.hasNode(start_node) || !graph.hasNode(goal_node)) {
    out.success = false;
    out.error_msg = "start/goal node missing";
    return out;
  }

  struct QItem {
    double dist;
    std::string node;
    bool operator>(const QItem& o) const { return dist > o.dist; }
  };

  std::unordered_map<std::string, double> dist;
  std::unordered_map<std::string, std::string> prev_node;
  std::unordered_map<std::string, std::string> prev_edge;

  for (const auto& id : graph.nodeIds()) dist[id] = std::numeric_limits<double>::infinity();
  dist[start_node] = 0.0;

  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
  pq.push(QItem{0.0, start_node});

  while (!pq.empty()) {
    const auto cur = pq.top();
    pq.pop();
    if (cur.dist > dist[cur.node]) continue;
    if (cur.node == goal_node) break;

    for (const auto& tr : graph.outgoing(cur.node, opts)) {
      const double nd = cur.dist + tr.cost;
      if (nd < dist[tr.to_id]) {
        dist[tr.to_id] = nd;
        prev_node[tr.to_id] = cur.node;
        prev_edge[tr.to_id] = tr.edge_id;
        pq.push(QItem{nd, tr.to_id});
      }
    }
  }

  if (!std::isfinite(dist[goal_node])) {
    out.success = false;
    out.error_msg = "no path in graph";
    return out;
  }

  out.success = true;
  out.total_cost = dist[goal_node];

  // reconstruct
  std::vector<std::string> nodes;
  std::vector<std::string> edges;
  std::string cur = goal_node;
  nodes.push_back(cur);
  while (cur != start_node) {
    auto it = prev_node.find(cur);
    if (it == prev_node.end()) break;
    const std::string p = it->second;
    edges.push_back(prev_edge[cur]);
    nodes.push_back(p);
    cur = p;
  }
  std::reverse(nodes.begin(), nodes.end());
  std::reverse(edges.begin(), edges.end());
  out.node_sequence = std::move(nodes);
  out.edge_sequence = std::move(edges);
  return out;
}

}  // namespace industrial_route_core
