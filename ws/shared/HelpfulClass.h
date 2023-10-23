#pragma once

#include "AMPCore.h"

//transformation matrices;
Eigen::MatrixXd LinkTransform(double theta, double linkLength);
Eigen::Matrix2d rotationMatrix(double theta);

///@brief checks if 2 line segments intersect, assumes endpoints do not lie on either line
///@param a1, a2 are endpoints of line a, b1, b2 are endpoints of line b
bool isIntersecting(Eigen::Vector2d a1,Eigen::Vector2d a2,Eigen::Vector2d b1,Eigen::Vector2d b2);

///@brief checks if given point is inside 2d polygon obstacle
bool isInObstacle(Eigen::Vector2d point, amp::Obstacle2D obstacle);

///@brief finds orientation of ordered set of 3 points
int orientation(Eigen::Vector2d p,Eigen::Vector2d q,Eigen::Vector2d r);

///@brief checks if 2 polygons are intersecting
bool polygonIntersecting(amp::Obstacle2D A, amp::Obstacle2D B);

///@brief checks if a line segment intersects a polygon
bool lineIntersectingPolygon(amp::Obstacle2D obstacle, Eigen::Vector2d a1, Eigen::Vector2d a2);