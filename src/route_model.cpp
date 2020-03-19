#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int count = 0;
  for (Model::Node node: this->Nodes()) {
    m_Nodes.push_back(Node(count, this, node));
    count++;
  }
  CreateNodeToRoadHashmap();
}

/*
Create node to road map
{node index : vector<road pointer>}
each node is possible to map to multi roads
*/
void RouteModel::CreateNodeToRoadHashmap() {
  for(const Model::Road &road: Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_idx: Ways()[road.way].nodes) {
        if (node_to_road.find(node_idx) == node_to_road.end()) {
          node_to_road[node_idx] = std::vector<const Model::Road*>();
        }
        node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

// Find the closest and non visited node in the range node_indices
RouteModel::Node * RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
  // Create a pointer
  Node *closest_node = nullptr; // RouteModel::Node
  // Declare a temporary Node variable node
  Node node;
  // For each node_index in node_indices
  for (int node_idx: node_indices) {
    // Set node equal to the parent_model->SNodes()[node_index]
    node = parent_model->SNodes()[node_idx];
    // If the distance from this to node is nonzero, and the node has not been visited:
    if (this->distance(node) != 0 && !node.visited) {
    //     If the closest_node equals nullptr, _or_ the distance from this to node is less
    //     than the distance from this to *closest_node:

      if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
        // Set closest_node to point to the address of parent_model->SNodes()[node_index].
        closest_node = &parent_model->SNodes()[node_idx];
      }
    }
  }
  // finally, return the closest_node.
  return closest_node;
}

// find the closest node in each related road
void RouteModel::Node::FindNeighbors() {
  // traverse all roads related to this node
  // find the closest node in each road
  for (auto &road: parent_model->node_to_road[this->index]) {
    RouteModel::Node* neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (neighbor != nullptr) {
      this->neighbors.push_back(neighbor);
    }
  }
}

// find the closest node to (x,y) among roads
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
  Node input_node = Node();
  input_node.x = x;
  input_node.y = y;
  float dist;
  float min_dist = std::numeric_limits<float>::max();
  int closest_idx;

  for(const Model::Road &road: Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_idx: Ways()[road.way].nodes) {
        dist = input_node.distance(SNodes()[node_idx]);
        if (dist < min_dist) {
          min_dist = dist;
          closest_idx = node_idx;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}
