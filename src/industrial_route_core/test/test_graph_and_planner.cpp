#include "industrial_route_core/geojson_loader.hpp"
#include "industrial_route_core/route_planner.hpp"

#include <fstream>
#include <gtest/gtest.h>

TEST(IndustrialRouteCore, LoadGeoJsonAndPlan) {
  industrial_route_core::Graph graph;
  const char* tmp_path = "/tmp/industrial_route_core_test_graph.geojson";
  {
    std::ofstream out(tmp_path);
    out << R"({
      "type": "FeatureCollection",
      "name": "graph",
      "features": [
        { "type": "Feature", "properties": { "id": 0, "frame": "map" },
          "geometry": { "type": "Point", "coordinates": [0.0, 0.0] } },
        { "type": "Feature", "properties": { "id": 1, "frame": "map" },
          "geometry": { "type": "Point", "coordinates": [2.0, 0.0] } },
        { "type": "Feature",
          "properties": { "id": 0, "startid": 0, "endid": 1, "weight": 1.0, "bidirectional": true },
          "geometry": { "type": "MultiLineString", "coordinates": [[[0.0,0.0],[2.0,0.0]]] } }
      ]
    })";
  }

  const auto result = industrial_route_core::GeoJsonLoader::loadFromFile(tmp_path, &graph);
  ASSERT_TRUE(result.success) << result.error_msg;
  ASSERT_EQ(graph.nodeIds().size(), 2u);
  ASSERT_EQ(graph.edgeIds().size(), 1u);

  industrial_route_core::RoutePlanner planner(&graph);
  industrial_route_core::PlannerOptions opts;
  opts.connect_max_dist = 10.0;
  opts.on_graph_max_dist = 2.0;
  opts.densify_resolution = 0.5;

  // Plan node->node
  const auto node_plan = planner.planNodeToNode("0", "1", opts);
  ASSERT_TRUE(node_plan.success) << node_plan.error_msg;
  ASSERT_GE(node_plan.node_sequence.size(), 2u);
  ASSERT_FALSE(node_plan.dense_path.empty());

  // Plan pose->pose (off-graph)
  industrial_route_core::Pose2D s;
  s.x = 1.0;
  s.y = 0.0;
  industrial_route_core::Pose2D g;
  g.x = 2.5;
  g.y = 0.0;
  const auto pose_plan = planner.planPoseToPose(s, g, opts, nullptr);
  ASSERT_TRUE(pose_plan.success) << pose_plan.error_msg;
  ASSERT_FALSE(pose_plan.dense_path.empty());
}
