#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    Eigen::Vector2d currentPosition = (0.0, 0.0);
    Eigen::Vector2d heading = problem.q_goal - problem.q_init;
    double stepSize = 0.01;

    while (currentPosition != problem.q_goal) {
        Eigen::Vector2d nextStep = heading * stepSize;
        if ()
    }

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(Eigen::Vector2d(5.0, 0.0));
    path.waypoints.push_back(Eigen::Vector2d(5.0, 10.0));
    path.waypoints.push_back(problem.q_goal);

    return path;
}
amp::Path2D 