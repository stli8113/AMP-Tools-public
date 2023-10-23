#ifndef CSpaceFunctions_H
#define CSpaceFunctions_H
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class

using namespace amp;

///@brief constructs a c space obstacle for 2d convex obstacles and convex translating robot
///@param A, B vectors in CCW order of robot and workspace obstacle respectively
///@return Polygon object of cspace obstacle
amp::Polygon constructCSpaceObstacle(std::vector<Eigen::Vector2d> A, std::vector<Eigen::Vector2d> B); 

///@brief constructs c space obstacle for rotating and translating convex robot and obstacle
///@return vector of polygons of c space obstacles
std::vector<Polygon> constructSE2Obstacle(std::vector<Eigen::Vector2d> A, std::vector<Eigen::Vector2d> B, std::vector<double> thetas);

//comment stuff after this for main to actually compile and run
// class CspaceLink2D : public amp::GridCSpace2D{
//     using GridCSpace2D :: GridCSpace2D;
//     CspaceLink2D(std::vector<double> m_link_lengths, std::vector<Obstacle2D> obstacles,double x0_min, double x0_max, double x1_min, double x1_max, std::size_t x0_cells, std::size_t x1_cells) 
//         :GridCSpace2D(x0_cells, x1_cells,  x0_min,  x0_max,  x1_min,  x1_max){
//             m_link_lengths = m_link_lengths;
//             obstacles = obstacles;
//         }
//     public:
//         bool inCollision(double x0, double x1) const override;
//     private:
//         std::vector<double> m_link_lengths;
//         std::vector<Obstacle2D> obstacles;
// }

#endif