#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

   start_node = &(m_Model.FindClosestNode(start_x, start_y));
   end_node   = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return end_node->distance(*node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
 {
    current_node->FindNeighbors();

    for(RouteModel::Node* node : current_node->neighbors)
    {
        node->parent = current_node;
        node->g_value = current_node->g_value + node->distance(*current_node);
        node->h_value = CalculateHValue(node);

        open_list.push_back(node);
        node->visited = true;
    }
}

bool CompareNode(RouteModel::Node* a,  RouteModel::Node* b)
{
    return (*a).g_value + (*a).h_value > (*b).g_value + (*b).h_value;
}

RouteModel::Node *RoutePlanner::NextNode()
{
    sort(open_list.begin(), open_list.end(), CompareNode);
    RouteModel::Node* nextNode = open_list.back();
    open_list.pop_back();

    return nextNode;

}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    std::vector<RouteModel::Node> pathNodeList;
    RouteModel::Node* pathNode = current_node;

    while(pathNode != start_node)
    {
        pathNodeList.push_back(*pathNode);
        distance += pathNode->distance(*(pathNode->parent));

        pathNode = pathNode->parent;
        
    }

    pathNodeList.push_back(*start_node);
    int pathSize = int(pathNodeList.size());

    //reverse pathNodeList to get path in correct order
    path_found.resize(pathSize);
    for(int n=0; n<pathSize; ++n)
    {
        path_found[n] = pathNodeList[pathSize - 1 - n];
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    //Add starting node to open list
    start_node->visited = true;
    open_list.push_back(start_node);
    
    // TODO: Implement your solution here.
    while (open_list.size() > 0)
    {
        current_node = NextNode();

        if( current_node->distance(*end_node) == 0.0f )
        {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        AddNeighbors(current_node);
    }
}