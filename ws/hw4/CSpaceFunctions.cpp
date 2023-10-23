#include "AMPCore.h"
#include "CSpaceFunctions.h"
#include "HelpfulClass.h"
#include "hw/HW4.h"
#include "MyManipulator.h"
#define _USE_MATH_DEFINES 
using namespace amp;

amp::Obstacle2D constructCSpaceObstacle(std::vector<Eigen::Vector2d> A, std::vector<Eigen::Vector2d> B){
    Eigen::Matrix2d rotate;
    rotate = rotationMatrix(M_PI);

    std::vector<Eigen::Vector2d> obstacleVertices;
    std::vector<Eigen::Vector2d> robotInv;
    std::vector<double> y, yB; //this is bad naming but it's also 1am
    int Asize = A.size(), Bsize = B.size();

    for(int i = 0; i < A.size(); i++){
        Eigen::Vector2d newvec = rotate * A[i];
        robotInv.push_back(newvec);
        y.push_back(newvec(1));
    }
    for(int i = 0; i < B.size(); i++){
        yB.push_back(B[i](1));
    }
    //find iterate of minimum y value so algo works
    auto yMinIter = std::min_element(y.begin(), y.end());
    if(yMinIter == y.begin() && y.back() == y.front()){
        yMinIter = y.end()--;
    }
    
    auto yObstacleMinIter = std::min_element(yB.begin(), yB.end());
    if(yMinIter == yB.begin() && yB.back() == yB.front()){
        yObstacleMinIter = yB.end()--;
    }
    int yObstacleMinIndex = std::distance(yB.begin(), yObstacleMinIter);
    int yMinIndex = std::distance(y.begin(), yMinIter);
    
    //rotate vectors so robot and obstacle start from lower left
    std::rotate(robotInv.begin(), robotInv.begin() + yMinIndex, robotInv.end());
    std::rotate(B.begin(), B.begin() + yObstacleMinIndex, B.end());

    // std::cout << "B: " << B[0] << B[1] << B[2] << std::endl;
        
    int i=0,j=0;
    double robotAngle, obstacleAngle;
    do{
        // std::cout << "i,j: " << i << ", " << j << std::endl;
        Eigen::Vector2d newVert(robotInv[i%Asize] + B[j%Bsize]);
        obstacleVertices.push_back(newVert);

        robotAngle = std::atan2(robotInv[(i+1)%Asize](1) - robotInv[i%Asize](1), robotInv[(i+1)%Asize](0) - robotInv[i%Asize](0));
        obstacleAngle = std::atan2(B[(j+1)%Bsize](1) - B[j%Bsize](1), B[(j+1)%Bsize](0) - B[j%Bsize](0));
        
        //force angle to 0,2pi range
        if (robotAngle < 0){
            robotAngle += 2*M_PI;
        }
        if (obstacleAngle < 0){
            obstacleAngle += 2*M_PI;
        }

        //check which iterate to increment
        if(robotAngle < obstacleAngle){
            i++;
        }
        else if(obstacleAngle < robotAngle){
            j++;
        }
        else{
            i++;
            j++;
        }

    }while (i <= Asize && j <= Bsize); 
    amp::Obstacle2D cSpaceObstacle(obstacleVertices);
    return cSpaceObstacle;
} 

std::vector<Polygon> constructSE2Obstacle(std::vector<Eigen::Vector2d> A, std::vector<Eigen::Vector2d> B, std::vector<double> thetas){
    std::vector<Polygon> CspaceSlices;

    for(int i = 0; i < thetas.size(); i++){
        std::vector<Eigen::Vector2d> rotatedRobot;
        Eigen::Matrix2d rotate = rotationMatrix(thetas[i]);
        for(int i = 0; i < A.size(); i++){
            Eigen::Vector2d newvec = rotate * A[i];
            rotatedRobot.push_back(newvec);
        }
        amp::Obstacle2D Cspace = constructCSpaceObstacle(rotatedRobot, B);
        CspaceSlices.push_back(Cspace);
    }
    return CspaceSlices;
}

//comment stuff after this for main to actually compile and run
// bool amp::CspaceLink2D::inCollision(double x0, double x1){
//     // manipulator
//     std::vector<double> state = {x0,x1};
//     std::vector<Eigen::Vector2d> jointLocations;
//     MyManipulator manipulator(m_link_lengths);

//     jointLocations.push_back(manipulator.getJointLocation(state, 0));
//     jointLocations.push_back(manipulator.getJointLocation(state, 1));
//     jointLocations.push_back(manipulator.getJointLocation(state, 2));


//     // check if any joints are inside an obstacle
//     for (int i = 0; i < 3; i++){
//         for(int j = 0; j < obstacles.size(); j++){
//             if(HelpfulClass::isInObstacle(jointLocations[i], obstacles[j])){
//                 return true;
//             }
//         }
//     }
//     //check if any links cross through an obstacle
//     for(int i = 0; i < 2; i++){
//         for(int j = 0; j < obstacles.size(); j++){
//             std::vector<Eigen::Vector2d> obstacleVerts = obstacles[j].getVerticesCCW();
//             for(int k = 0; k < obstacleVerts.size(); k++){
//                 Eigen::Vector2d a1, a2, b1, b2;
//                 a1 = obstacleVerts[k];
//                 a2 = obstacleVerts[(k+1)%obstacleVerts.size()];
//                 b1 = jointLocations[i];
//                 b2 = jointLocations[(i+1)%3];
//                 if(HelpfulClass::isIntersecting(a1,a2,b1,b2)){
//                     return true;
//                 }
//             }
//         }
//     }
//     return false;
// }

// std::unique_ptr<amp::GridCSpace2D> amp::HW4::GridCSpace2DConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
//     // env;
//     std::size_t x0_cells = 100, x1_cells = 100;
//     double x0_min = 0, x0_max = 2*M_PI, x1_min = 0, x1_max = 2M_PI;
//     std::vector<double> m_link_lengths = manipulator.getLinkLengths();
//     std::vector<Obstacle2D> obstacles = env.obstacles;
//     amp::GridCSpace2D Cspace(m_link_lengths,  obstacles,x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);

//     //check collision of every configuration
//     for (int i = 0; i < x0_cells; i++){
//         for (int j = 0; j < x1_cells; j++){
//             double x0 = i*2*M_PI / x0_cells;
//             double x1 = j*2*M_PI / x1_cells;
//             Cspace(i,j) = inCollision(x0, x1);
//         }
//     }

//     return std::make_unique<MyGridCSpace2D>(Cspace);
// }