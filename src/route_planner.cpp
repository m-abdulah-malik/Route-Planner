#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Finding the closest nodes to the starting and ending coordinates 
    // Then Storing them in the RoutePlanner's start_node and end_node attributes
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Calculate the distance from the current node to the end node, that's equivalent to h value, using the distance method
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //    Identifying neighboring nodes
    current_node->FindNeighbors();
    //    Iterating through neighbors:
    //    Processing each neighboring node of the current node.
    for (auto neighbor : current_node->neighbors) {
        //    Establishing parent-child relationship:
        neighbor->parent = current_node;
        //    Calculating g-value (cost from start):
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        //    Estimating h-value (distance to goal):
        neighbor->h_value = CalculateHValue(neighbor);
        //    Adding to open list for exploration:
        open_list.push_back(neighbor);
        //    Marking as visited
        neighbor->visited = true;
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    //  Sort open_list by lowest sum of h-value (goal estimate) and g-value (cost from start).
    std::sort(open_list.begin(), open_list.end(), [](const auto &a, const auto &b) {
        return a->h_value + a->g_value < b->h_value + b->g_value;
    });
    //  Identify node with lowest total cost
    RouteModel::Node* next_node = open_list.front();
    //  Remove chosen node from open list
    open_list.erase(open_list.begin());
    //  Return the selected node
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Initialize path distance to 0.
    distance = 0.0f;
    // Create an empty vector to store the constructed path.
    std::vector<RouteModel::Node> path_found;
    // Placeholder for parent nodes during path construction.
    RouteModel::Node parent;
    // Iterate backwards from the final node to the starting node.
    while (current_node->parent != nullptr) {
    // Add the current node to the path vector.
    path_found.push_back(*current_node);
    // Accumulate distance between the current node and its parent.
    distance += current_node->distance(*(current_node->parent));
    // Move to the parent node for the next iteration.
    current_node = current_node->parent;
    }
    // Add the starting node (which doesn't have a parent) to the path.
    path_found.push_back(*current_node);
    // Apply map scale to calculate distance in meters.
    distance *= m_Model.MetricScale();
    // Return the constructed path in starting-to-ending order.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    // Mark the starting node as visited.
    start_node->visited = true;
    // Add the starting node to the open list for exploration.
    open_list.push_back(start_node);
    // Initialize a pointer to keep track of the current node.
    RouteModel::Node *current_node = nullptr;
    // Continue searching while there are nodes to explore.
    while (open_list.size() > 0) {
    // Get the next most promising node from the open list.
    current_node = NextNode();
    // If the current node is the goal, construct and return the final path.
    if (current_node->distance(*end_node) == 0) {
        m_Model.path = ConstructFinalPath(current_node);
        return;
    }
    // Otherwise, expand the search by adding neighboring nodes to the open list.
    else {
        AddNeighbors(current_node);
        }
    }
}