#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (RouteModel::Node* neighbor_node : current_node->neighbors) {
    if (!neighbor_node->visited) {
      neighbor_node->parent = current_node;
      float dist = current_node->distance(*neighbor_node);
      neighbor_node->g_value = dist + current_node->g_value;
      neighbor_node->h_value = CalculateHValue(neighbor_node);
      neighbor_node->visited = true;
      this->open_list.push_back(neighbor_node);
    }
  }
}


RouteModel::Node *RoutePlanner::NextNode() {
  sort(open_list.begin(), open_list.end(),
       [](const RouteModel::Node* a, const RouteModel::Node* b) -> bool
       {
         return (a->g_value + a->h_value) > (b->g_value + b->h_value);
       });
  RouteModel::Node* node = open_list.back();
  open_list.pop_back();
  return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node) {
      path_found.push_back(*current_node);
      if (current_node->parent) {
        distance += current_node->distance(*(current_node->parent));
      }
      current_node = current_node->parent;
    }
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);
    while (open_list.size() > 0) {
      current_node = NextNode();
      if (current_node->distance(*end_node) == 0) {
        m_Model.path = ConstructFinalPath(current_node);
        return;
      }
      AddNeighbors(current_node);
    }
  cout << "No path found!" << "\n";
}
