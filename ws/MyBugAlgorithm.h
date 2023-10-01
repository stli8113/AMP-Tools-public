#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        bool collision(amp::Obstacle2D obstacle, Eigen::Vector2d point);
        void follow(const amp::Problem2D& problem, amp::Path2D& path);
        void goToGoal(const amp::Problem2D& problem, amp::Path2D& path);
        Eigen::Matrix2d rotationMatrix(double theta);
    private:
        // Add any member variables here...
        Eigen::Vector2d currentPosition;
        Eigen::Vector2d rightHand;
        Eigen::Vector2d heading;
        const double stepSize = 0.01;
        const double tol = 0.02;
        double distToGoal;
};