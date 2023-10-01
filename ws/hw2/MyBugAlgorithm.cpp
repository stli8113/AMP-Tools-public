#include "MyBugAlgorithm.h"
#include <math.h>

#define _USE_MATH_DEFINES 

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    currentPosition = (problem.q_init);
    heading = (problem.q_goal - problem.q_init);
    distToGoal = heading.norm();
    heading.normalize();
    path.waypoints.push_back(problem.q_init);

    std::cout << "starting loop \n";
    //*
    bool following = false;

    while (distToGoal > tol){
        if(following){
            std::cout << "follow obstacle \n";
            follow(problem, path);
            following = false;
            heading = problem.q_goal - currentPosition;
            heading.normalize();
        }
        else{
            std::cout << "go to goal \n";
            goToGoal(problem, path);
            following = true;
        }
        // path.print();
        std::cout << "current pos: " << currentPosition << std::endl;
    }
    //*/
    std::cout << "path found!";
    path.print();
    path.waypoints.push_back(problem.q_goal);
    return path;
}

//*/Has bug circumnavigate obstacle and go to closest point to goal
void MyBugAlgorithm::follow(const amp::Problem2D& problem, amp::Path2D& path){
    // initialize vectors and starting position
    Eigen::Vector2d nextStep(heading * stepSize);
    rightHand = rotationMatrix(-M_PI / 2) * nextStep;
    Eigen::Vector2d q_start(currentPosition);
    int startingWaypoints = path.waypoints.size();
    int numOfWaypoints;

    Eigen::Vector2d vecToStart;
    double distToStart;
    

    Eigen::Vector2d vecToGoal(currentPosition - problem.q_goal);
    double distToGoal;
    double lastDist = vecToGoal.norm();
    int count = 0;
    int qLeaveCount = 0;
    // std::cout << "starting follow \n";
    std::cout << "waypoint number: " << startingWaypoints << std::endl;
    do{        
        //check for ANY collision
        bool stepWillCollide = false;
        bool rightWillCollide = false;  
        // std::cout << "check next step \n";
        nextStep = (heading * stepSize);
        for(int i = 0; i < problem.obstacles.size(); i++){
            stepWillCollide = collision(problem.obstacles[i], nextStep + currentPosition);
            if(stepWillCollide){
                break;
            }
        }
        // std::cout << "check right hand \n";
        for(int i = 0; i < problem.obstacles.size(); i++){
            rightWillCollide = collision(problem.obstacles[i], rightHand + currentPosition);
            if(rightWillCollide){
                break;
            }
        }
        // std::cout << "collision results:\n" << stepWillCollide << std::endl << rightWillCollide << std::endl;
        // if right hand is satisfied, increment position
        if(rightWillCollide && !stepWillCollide){
            // std::cout << "moving a step... \n";
            path.waypoints.push_back(nextStep + currentPosition);
            currentPosition += nextStep;
            count++;
        }
        //otherwise, rotate heading and try again
        else{
            // std::cout << "rotating heading... \n";
            heading = rotationMatrix(M_PI / 180)*heading; // MATRIX COMES FIRST YOU DIMWIT
            // std::cout << "Heading: \n" << heading << std::endl;
            rightHand(0) = heading(1);
            rightHand(1) = -heading(0);
            // std::cout << "Right hand: \n" << rightHand << std::endl;
        }
        // std::cout << "check distance to goal \n";
        vecToGoal = problem.q_goal - currentPosition;
        distToGoal = vecToGoal.norm();
        // std::cout << "dist to goal: " << distToGoal - lastDist << std::endl;

        
        if(distToGoal < lastDist){
            // std::cout << "changing leave point \n";
            qLeaveCount = count;
            lastDist = distToGoal;
        }
        vecToStart = currentPosition - q_start;
        distToStart = vecToStart.norm();
        // std::cout << "dist to start: " << distToStart << std::endl;
        
        numOfWaypoints = path.waypoints.size();

    }while(distToStart > tol || abs(startingWaypoints - numOfWaypoints) < 20);
    // path.print();
    std::cout << "circumnavigated! \n";
    std::cout << "qleavecoint: " << qLeaveCount << std::endl;
    std::cout << "waypoint num: " << numOfWaypoints << std::endl;
    if(count/2 >= qLeaveCount){
        std::cout << "go forwards \n";
        // std::cout << "length: " << path.waypoints.size() << std::endl;
        std::vector<Eigen::Vector2d> subvector = std::vector<Eigen::Vector2d>(path.waypoints.begin() + startingWaypoints, path.waypoints.begin() + startingWaypoints + qLeaveCount);
        std::cout << "waypoint end: " << path.waypoints[startingWaypoints + qLeaveCount] << std::endl;
        path.waypoints.insert(path.waypoints.end(), subvector.begin(), subvector.end());
        currentPosition = path.waypoints.back();
        // std::cout <<"appended length: " << path.waypoints.size() << std::endl;
    }
    else{
        std::cout << "go backwards \n";
        std::vector<Eigen::Vector2d> subvector = std::vector<Eigen::Vector2d>(path.waypoints.end() - qLeaveCount, path.waypoints.end());
        std::reverse(subvector.begin(),subvector.end());
        std::cout << "waypoint end: " << path.waypoints[startingWaypoints + qLeaveCount] << std::endl;
        // std::cout << "subvector created\n";
        // std::cout << "length: " << path.waypoints.size() << std::endl;
        path.waypoints.insert(path.waypoints.end(), subvector.begin(), subvector.end());
        currentPosition = path.waypoints.back();
        // std::cout <<"appended length: " << path.waypoints.size() << std::endl;
        // path.print();

    }
    
    return;
}

//moves bug toward goal until it hits an obstacle
void MyBugAlgorithm::goToGoal(const amp::Problem2D& problem, amp::Path2D& path){
    Eigen::Vector2d nextStep(heading * stepSize);
    Eigen::Vector2d vecToGoal(problem.q_goal - currentPosition);
    Eigen::Vector2d nextPos;
    distToGoal = vecToGoal.norm();
    // std::cout << "number obstacles: " << problem.obstacles.size() << std::endl;

    while(distToGoal > tol){
        bool willCollide = false;
        nextPos = nextStep + currentPosition;
        // std::cout << "current pos: " << currentPosition << std::endl;
        for(int i = 0; i < problem.obstacles.size(); i++){
            
            willCollide = collision(problem.obstacles[i], nextPos);
            // std::cout <<"\n \n willColide:" << willCollide << std::endl;
            if(willCollide){
                std::cout << "\n \n collision detected! leaving gotogoal \n";
                return;
            }
        }
        // path.print();
        path.waypoints.push_back(nextPos);
        currentPosition = nextPos;
        vecToGoal = problem.q_goal - currentPosition;
        distToGoal = vecToGoal.norm();
    }
    return;
}
// problem.obstacles[i]
//generates and returns 2d rotation matrix
Eigen::Matrix2d MyBugAlgorithm::rotationMatrix(double theta){
    Eigen::MatrixXd m(2,2);
    m(0,0) = std::cos(theta);
    m(1,0) = std::sin(theta);
    m(0,1) = -std::sin(theta);
    m(1,1) = std::cos(theta);
    // std::cout << "theta: "<< theta << "\n matrix:" <<  m << std::endl;
    return m;
}

//checks if point is inside an obstacle
bool MyBugAlgorithm::collision(amp::Obstacle2D obstacle, Eigen::Vector2d point){
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
//v.insert(v.end(),v.begin()+1,v.end()) appending vector elements 

//*/