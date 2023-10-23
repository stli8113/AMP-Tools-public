#include "AMPCore.h"
#include "gradientDescent.h"
#include "HelpfulClass.h"
#define _USE_MATH_DEFINES

using namespace amp;

amp::Path2D myGradientDescent::plan(const amp::Problem2D& problem){
    amp::Path2D path;
    Eigen::Vector2d currentPos(problem.q_init);
    path.waypoints.push_back(currentPos);
    // path.waypoints.push_back(Eigen::Vector2d(0,3));
    // currentPos = Eigen::Vector2d(0,3);

    Eigen::Vector2d Uatt, Urep, gradU;

    Eigen::Vector2d vecToGoal(currentPos - problem.q_goal);
    double distToGoal = vecToGoal.norm();
    int count = 0;
    int left = 0;
    while(distToGoal > epsilon && count < 2000){
        // std::cout << "current pos: " << currentPos(0) << " " << currentPos(1) << std::endl;
        // std::cout << "dist to goal: " << distToGoal << std::endl;
        // std::cout << count << std::endl;
        Uatt = getAttractGradient(currentPos, problem.q_goal);
        Urep = getRepulseGradient(problem.obstacles, currentPos);

        gradU = Uatt + Urep;
        currentPos += gradU;
        if (gradU.norm() < 1e-6){
            Eigen::Vector2d noise(.05 * pow(-1,left),.01);
            // std::cout << pow(-1,left);
            left++;
            currentPos += noise*Q_star*0.25;
        }
        Eigen::Vector2d vecToGoal(currentPos - problem.q_goal);
        distToGoal = vecToGoal.norm();
        path.waypoints.push_back(currentPos);
        count++;
    }
    // if(distToGoal <= epsilon){
        path.waypoints.push_back(problem.q_goal);
    // }
    std::cout << "path finished\n";
    return path;
}

///@brief returns a vector of the attractive gradient
///@param q current robot position
///@param q_goal goal position
///@return Uatt, vector of attractive component of gradient descent
Eigen::Vector2d myGradientDescent::getAttractGradient(Eigen::Vector2d q, Eigen::Vector2d q_goal){
    Eigen::Vector2d Uatt, vecToGoal;
    double distToGoal;

    //find distance to goal
    vecToGoal = q_goal - q;
    distToGoal = vecToGoal.norm();
    //check if distance to goal is within d* and evaluate gradient
    if (distToGoal <= d_star){
        Uatt = xi * (q_goal - q);
    } 
    else{
        Uatt = (d_star * xi * (q_goal - q)) / distToGoal;
    }


    return Uatt;
}

///@brief finds the component repulsive gradient from a single specified obstacle, assumes obstacle is within Q*
///@param q is current robot position
///@param obstacle is 2d obstacle in cartesian workspace
Eigen::Vector2d myGradientDescent::getClosestPoint(Eigen::Vector2d q, amp::Obstacle2D obstacle){
    Eigen::Vector2d closestPoint;

    std::vector<Eigen::Vector2d> obs = obstacle.verticesCCW();
    Eigen::Vector2d closestVert(obs[0]); // initialize vertex to compare to

    Eigen::Vector2d vecToVert(closestVert - q);
    double smallestDistToVert = vecToVert.norm();
    double distToVert;
    int closestIndex = 0;
    bool Intersecting;

    //find closest free vertex to point on obstacle
    for(int i = 0; i < obs.size(); i++){
        Intersecting = false;
        int j = 0;
        
        do{
            if(j == i || (j+1)%obs.size() == i){
                j++;
                // std::cout << "edge is from vertex being checked, continuing\n";
                continue;
            }
            Intersecting = isIntersecting(q, obs[i], obs[j], obs[(j+1)%obs.size()]); //checks for line intersection with segment j to j+1
            j++;
        }while(!Intersecting && j < obs.size());
        
        //if vertex checked is not free, move on
        if (Intersecting){
            continue;
        }
        
        vecToVert = (q - obs[i]);
        distToVert = vecToVert.norm();
        if (distToVert < smallestDistToVert){
            // std::cout << "changing closest vert index to: " << i << std::endl;
            closestVert = obs[i];
            smallestDistToVert = distToVert;
            closestIndex = i;
        }
    }

    //check if point is closer to segments coming from closest vertex
    Eigen::Vector2d backVert, foreVert; // initialize 2 vertices from closest vertex
    backVert = obs[(closestIndex-1 + obs.size()) % obs.size()]; //dumb jank to make sure index is in bounds
    foreVert = obs[(closestIndex+1) % obs.size()];
    if(closestIndex == 3){
    // std::cout << "closest index: " << closestIndex << std::endl << "backvert: " <<(closestIndex-1 + obs.size()) % obs.size() << std::endl;
    }
    std::vector<Eigen::Vector2d> verts = {backVert, closestVert, foreVert};
    closestPoint = checkProjections(verts, q);

    // std::cout << "\nclosest point: " << closestPoint(0) << " " << closestPoint(1);

    return closestPoint;
}

Eigen::Vector2d myGradientDescent::checkProjections(std::vector<Eigen::Vector2d> verts, Eigen::Vector2d q){
    Eigen::Vector2d closestPoint, a, b, projAB;
    closestPoint = verts[1]; // initialize to closest vector
    for(int i =0; i < 2; i++){
        b = verts[i+1] - verts[i];
        a = q - verts[i];

        double dot = a.dot(b);
        // std::cout << "verts[i++]: " << verts[i+1](0) << " " << verts[i+1](1) << std::endl;
        // std::cout << "verts[i]: " << verts[i](0) << " " << verts[i](1) << std::endl;
        if (dot < 0){
            continue;
        }
        projAB = (dot / b.dot(b)) * b;
        // std::cout << "projection norm: " << projAB.norm() << "\nb norm: " << b.norm() << std::endl;
        if(b.norm() > projAB.norm()){
            // std::cout << "closest point is on edge\n";
            closestPoint = verts[i] + projAB;
            break;
        }
    }
    return closestPoint;
}

Eigen::Vector2d myGradientDescent::getRepulseGradient(std::vector<Obstacle2D> obstacles, Eigen::Vector2d q){
    std::vector<Eigen::Vector2d> closests;
    Eigen::Vector2d best;
    
    for(int i =0; i < obstacles.size(); i++){
        best = getClosestPoint(q, obstacles[i]);
        closests.push_back(best);
    }
    

    Eigen::Vector2d di, obGrad;
    Eigen::Vector2d Urep(0,0);
    for (int i = 0; i < closests.size() ; i++){
        di = q - closests[i];
        // std::cout << "dist to ob: " << di.norm() << std::endl;
        // std::cout << "Q*: " << Q_star << std::endl;
        if (di.norm() > Q_star){
            // std::cout << "dist to ob: " << di.norm() << std::endl;
            // std::cout << "obstacle too far, continuing\n";
            continue;
        }    
        // std::cout << "close enough\ndist to ob: " << di.norm() << std::endl;
        
        obGrad = eta*(1/di.norm()-1/Q_star) * (1/pow(di.norm(),2)) * di.normalized();
        Urep += obGrad;
        // std::cout << "normalized: " << di.normalized() << std::endl << "Urep: " << Urep(0) << " " << Urep(1) << std::endl;
    }
    return Urep;
} 