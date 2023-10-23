#pragma once
#include "AMPCore.h"
#include "hw/HW5.h"

using namespace amp;

class myGradientDescent : public amp::GDAlgorithm{
    public:
        myGradientDescent(double xi, double eta,double d_star,double Q_star, double eps) :
        amp::GDAlgorithm(), 
        xi(xi), eta(eta), d_star(d_star), Q_star(Q_star), epsilon(eps){}

        ///@brief creates the plannned path
        amp::Path2D plan(const amp::Problem2D& problem);

        // double getDistance(Eigen::Vector2d q, std::vector<amp::Obstacle2D> obstacles); what does this do??
        ///@brief evaluates attractive gradient from position
        Eigen::Vector2d getAttractGradient(Eigen::Vector2d q, Eigen::Vector2d q_goal);

        ///@brief finds closest point on obstacle to position
        Eigen::Vector2d getClosestPoint(Eigen::Vector2d q, amp::Obstacle2D obstacle);

        ///@brief checks if point is over a line or vertex
        ///@param verts is vector of 3 vertexes closest to the robot
        ///@return closest point on obstacle to robot
        Eigen::Vector2d checkProjections(std::vector<Eigen::Vector2d> verts, Eigen::Vector2d q);

        ///@brief finds the repulsive function for gradient descent
        ///@param closests is a std vector of the closest point on every obstacle to the current position
        ///@return gradient push function evaluation
        Eigen::Vector2d getRepulseGradient(std::vector<Obstacle2D> obstacles, Eigen::Vector2d q);

        
    private:
        double xi, eta, d_star, Q_star, epsilon;
};