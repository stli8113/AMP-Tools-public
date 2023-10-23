#pragma once
#include "AMPCore.h"
#include "hw/HW6.h"

class myAStarAlgo : public amp::AStar{
    public:
        struct nodeInfo{
            uint32_t node;
            double distFromStart;
            double heuristic;
            const nodeInfo* parent;

            bool operator<(const myAStarAlgo::nodeInfo& other) const {
                return (distFromStart + heuristic) > (other.distFromStart + other.heuristic);
            }
        };

        /// @brief Find the shortest path from an init node to a goal node on a graph, while using a heuristic.
        /// @param problem Search problem containing init/goal nodes and graph
        /// @param heuristic Heuristic function that maps each node to a "cost-to-go"
        /// @return The optimal node path and path cost (if the heuristic is admissible)
        AStar::GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;

        /// @brief constructs the graph path from the final node found by A*
        std::list<amp::Node> constructPath(const myAStarAlgo::nodeInfo &finalNode);
    public:
        
    
};
