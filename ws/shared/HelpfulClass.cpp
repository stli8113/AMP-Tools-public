#include "HelpfulClass.h"

Eigen::MatrixXd LinkTransform(double theta, double linkLength){
    Eigen::MatrixXd m(3,3);
    m(0,0) = std::cos(theta);
    m(1,0) = std::sin(theta);
    m(2,0) = 0;
    m(0,1) = -std::sin(theta);
    m(1,1) = std::cos(theta);
    m(2,1) = 0;
    m(0,2) = linkLength;
    m(1,2) = 0;
    m(2,2) = 1;
    // std::cout << "theta: "<< theta << "\n matrix:" <<  m << std::endl;
    return m;
}

Eigen::Matrix2d rotationMatrix(double theta){
    Eigen::MatrixXd m(2,2);
    m(0,0) = std::cos(theta);
    m(1,0) = std::sin(theta);
    m(0,1) = -std::sin(theta);
    m(1,1) = std::cos(theta);
    // std::cout << "theta: "<< theta << "\n matrix:" <<  m << std::endl;
    return m;
}

bool isIntersecting(Eigen::Vector2d a1,Eigen::Vector2d a2,Eigen::Vector2d b1,Eigen::Vector2d b2){
  
    int o1 = orientation(a1, a2, b1); 
    int o2 = orientation(a1, a2, b2); 
    int o3 = orientation(b1, b2, a1); 
    int o4 = orientation(b1, b2, a2);

    // std::cout << "orientations: " << o1 << ", " << o2 << ", " << o3 << ", " << o4 << std::endl;

    if (o1 != o2 && o3 != o4){
        // std::cout << "different orientation!\n";
        return true;
    }
    else if(o1 ==0 || o2==0 || o3 ==0 || o4==0){
        // std::cout << "colinear\n";
        return true;
    }

    return false;
}

int orientation(Eigen::Vector2d p,Eigen::Vector2d q,Eigen::Vector2d r) 
{ 
    double val = (q(1) - p(1)) * (r(0) - q(0)) - (q(0) - p(0)) * (r(1) - q(1));
    // std::cout << "val: " << val << std::endl;
    if (val == 0) {
        return 0;
    };  // collinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

bool isInObstacle(Eigen::Vector2d point, amp::Obstacle2D obstacle){
    int i = 0;
    std::vector<Eigen::Vector2d> vertices(obstacle.verticesCCW());
    int n = vertices.size();
    bool isInPrimitive = true;

    do{ 
        // std::cout << "primitive iter: " << i << std::endl;
        
        Eigen::Vector2d p1(vertices[i]);
        Eigen::Vector2d p2(vertices[(i + 1) % n]);
        double primitiveValue;

        if(p1[0] < p2[0]){
            // std::cout << "going left \n";
            primitiveValue = (p1[1] - p2[1])/(p1[0] - p2[0]) * (point[0] - p1[0]) + p1[1] - point[1];
            isInPrimitive =  primitiveValue <= 0;
        }
        else if(p1[0] > p2[0]){
            // std::cout << "going right \n";
            primitiveValue = (p1[1] - p2[1])/(p1[0] - p2[0]) * (point[0] - p1[0]) + p1[1] - point[1];
            isInPrimitive = primitiveValue >= 0;
        }
        else if(p1[1] < p2[1]){
            // std::cout << "going up \n";
            isInPrimitive = point[0] <= p1[0];
        }
        else{
            // std::cout << "going down \n";
            isInPrimitive = point[0] >= p1[0];
        }
        // std::cout << "in primitive: " << isInPrimitive << std::endl;

        i = (i + 1) % n;
    } while (i != 0 && isInPrimitive);
    return isInPrimitive;
}

bool polygonIntersecting(amp::Obstacle2D A, amp::Obstacle2D B){
    std::vector<Eigen::Vector2d> AVertices(A.verticesCCW());
    std::vector<Eigen::Vector2d> BVertices(B.verticesCCW());

    // Check if any vertex is inside the other obstacle
    for (int i=0; i<AVertices.size(); i++){
        if(isInObstacle(AVertices[i], B)){
            return true;
        }
    }
    for (int i=0; i<BVertices.size(); i++){
        if(isInObstacle(BVertices[i], A)){
            return true;
        }
    }

    // loop through every line segment and check for intersections
    for (int i=0; i<AVertices.size(); i++){
        for (int j=0; j<BVertices.size(); j++){
            Eigen::Vector2d a1(AVertices[i]);
            Eigen::Vector2d a2(AVertices[(i+1)%AVertices.size()]);
            Eigen::Vector2d b1(BVertices[j]);
            Eigen::Vector2d b2(BVertices[(j+1)%BVertices.size()]);
            if(isIntersecting(a1, a2, b1, b2)){
                return true;
            }
        }
    }
    // if nothing returns, return false, no collision
    return false;
}

bool lineIntersectingPolygon(amp::Obstacle2D obstacle, Eigen::Vector2d a1, Eigen::Vector2d a2){
    std::vector<Eigen::Vector2d> obsJoints = obstacle.verticesCCW();
    Eigen::Vector2d b1, b2; //initialize second set of line segment points
    for(int i=0; i<obsJoints.size(); i++){
        b1 = obsJoints[i];
        b2 = obsJoints[(i+1)%obsJoints.size()];
        // std::cout << "vertex1: " << b1(0) << ", " << b1(1) << "\nvertex2: " << b2(0) << ", " << b2(1) << std::endl;
        if(isIntersecting(a1, a2, b1, b2)){
            return true;
        }
    } 
    return false;
}