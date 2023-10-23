#include "AStar.h"
#include "AMPCore.h"

#include <queue>
#include <unordered_set>


myAStarAlgo::GraphSearchResult myAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){
    amp::Graph graph = *problem.graph;
    GraphSearchResult result;
    
    std::priority_queue<nodeInfo, std::vector<nodeInfo>> nodesToSearch;
    myAStarAlgo::nodeInfo initNode{problem.init_node, 0.0, heuristic(problem.init_node),nullptr}; //initialize first node


    nodesToSearch.push(initNode);
    nodeInfo checkNode;
    nodeInfo currentNode;
    
    std::unordered_set<uint32_t> searchedNodes; //list of searched nodes to avoid duplicate searches
    // int count =0;
    while(!nodesToSearch.empty()){
        // initialize stuff to add to queue
        // what in the hells is happening
        checkNode = nodesToSearch.top();
        if(checkNode.parent != nullptr){
            std::cout << "check node: " << checkNode.node << "\ncheck parent: " << checkNode.parent->node << std::endl;
        }
        // myAStarAlgo::nodeInfo testNode = nodesToSearch.top();
        currentNode = checkNode;
        if(currentNode.parent != nullptr){
            std::cout << "current node: " << checkNode.node << "\ncurrent parent: " << checkNode.parent->node << std::endl;
        }
        
        nodesToSearch.pop(); //remove top node so it doesn't end up on top after loop finishes and recurse infinitely

        
        std::vector<uint32_t> childNodes = graph.children(currentNode.node);
        std::vector<double> edges = graph.outgoingEdges(currentNode.node);

        searchedNodes.emplace(currentNode.node); //add current node to searched nodes
        
        //check if goal has been reached and construct result struct if true
        if(currentNode.node == problem.goal_node){
            std::cout << "goal found!\n";
            std::cout << "goal node parent: " << currentNode.parent->node << std::endl;
            result.success = true;
            // result.node_path = constructPath({checkNode.node, checkNode.distFromStart, checkNode.heuristic, checkNode.parent});
            result.path_cost = currentNode.distFromStart;
            return result;
        }
        
        // loop through every child node and add to priority queue
        for (int i=0; i<childNodes.size(); i++){
            // std::cout << "child node: " << childNodes[i] << std::endl;
            // std::cout << "has been searched: " << (searchedNodes.find(childNodes[i]) != searchedNodes.end()) << std::endl; //<< searchedNodes.end() << std::endl;
            // check if the child node has already been searched
            if(searchedNodes.find(childNodes[i]) == searchedNodes.end()){
                // std::cout << "adding node...\n";
                // get edge length and heuristic
                double nextHeuristic = heuristic(childNodes[i]);
                double distFromStart = currentNode.distFromStart + edges[i];
                
                // create and push node to priority queue
                myAStarAlgo::nodeInfo nextNode{childNodes[i], distFromStart, nextHeuristic, &currentNode};
                // std::cout << "current node: " << currentNode.node << std::endl;
                // std::cout << "next node: " << nextNode.node << std::endl;
                // std::cout << "next parent: " << nextNode.parent->node << std::endl;
                nodesToSearch.push(nextNode);
            }
        }
        // myAStarAlgo::nodeInfo topNode = nodesToSearch.top();
        // checkNode = topNode;
        // std::cout << "top node: " << checkNode.node << std::endl << "top node parent: " << checkNode.parent->node << std::endl;
        // count++;
    }
    //if there is no path return null
    std::cout << "no path found\n";
    result.success = false;
    return result;

}

std::list<amp::Node> myAStarAlgo::constructPath(const myAStarAlgo::nodeInfo& finalNode){
    std::cout << "constructing path...\n";
    std::list<amp::Node> path;
    const myAStarAlgo::nodeInfo* currentNode = &finalNode; //create pointer to goal node 
    std::cout << "goal parent: " << currentNode->parent->node << std::endl;

    while(currentNode != nullptr){
        path.push_front(currentNode->node); // push node value to path 
        currentNode = currentNode->parent; // get pointer to parent node
        // std::cout << "currentNode:" << currentNode->node << std::endl;
    }
    return path;
}