#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    //start_node and end_node are both Node pointers found in route_planner.h. I can use the FindClosetNode function from the Model class (m_Model in this case) and assign the addresses of those closest nodes to the start/end Nodes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

    //The node pointer parameter can use the distance function apart of the Node class. I just need to use the arrow operator to dereference the memory address and access the float that is returned using the distance function from the Node object.
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    //First step is to call FindNeighbors() on the current Node.
    current_node->FindNeighbors();
    //Now I need a loop for every neighbor node in the "current_node.neighbors vector" and set the parent node, h_value, and g_value.
    for(auto neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        //g_value = g + d of the current node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        //Once the neighbor node is created then we can push it to the open_list and set visited to true.
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

// 
// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

    //I can use the sort library to iterate through the open_list and evaluate which h+g value is smallest.
    std::sort(open_list.begin(), open_list.end(), [](const auto &a, const auto &b){
        //Move onto the next node in the list if the first h+g sum is larger than the second. When this is finished the lowest sum node will be at the back of the list.
        return (a->h_value + a->g_value) > (b->h_value + b->g_value);
    });
    
    //Create the pointer to the node with the lowest sum from sort.
    auto node = open_list.back();
    //Removing the lowest sum node from the list and returning that node.
    open_list.pop_back();
    return node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    //I need to continue to loop as long as there's a parent node for the node I am examining. If there isn't a parent then I need to exit because the destination is found.
    while(current_node->parent != nullptr){
        //If there is a parent then we need to push the node onto the path_found.
        path_found.push_back(*current_node);
        //I need to add the distance from the node to its parent to the distance variable.
        distance = distance + current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    //Once I've exited this loop then the start node should be the first element of the path_found vector. I want to push_back the node onto path_found to generate the final path.
    path_found.push_back(*current_node);
    //I had some trouble passing the test applications and it turns out it's because the order of my path_found is reversed. I found that there is a reverse function in the standard library which would correct this.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    //I need to flag the start node as visited and add it to the open list. I then loop through the open_list to add all of the neighbors of the current node to the open_list.
    start_node->visited = true;
    open_list.push_back(start_node);

    while(open_list.size() != 0){
        //I need to evaluate if the current node in the list contains the coordinates of the final destination. 
        //If it doesn't then I need to add the node neighbors to the open_list and set the current to the next node. 
        //Else I can call ConstructFinalPath and exit.
        current_node = NextNode();

        if( (current_node->x != end_node->x) || (current_node->y != end_node-> y) ){
            AddNeighbors(current_node);           
        }
        else{
            m_Model.path = ConstructFinalPath(current_node);
        }
    }
}