// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "MyManipulator.h"
#include "CSpaceFunctions.h"
#define _USE_MATH_DEFINES 

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    amp::Obstacle2D Ex1Obstacle = HW4::getEx1TriangleObstacle();
    amp::Obstacle2D Ex1Robot = HW4::getEx1TriangleObstacle();

    //create vector of rotations, my genius is sometimes frightening
    std::vector<double> thetas(12);
    std::iota(thetas.begin(), thetas.end(), 1);
    for (int i = 0; i < thetas.size(); i++){
        thetas[i] *= (M_PI/6);
    }
    thetas.insert(thetas.begin(),0);

    //Exercise 1**********************************
    std::vector<Obstacle2D> Cspace3d = constructSE2Obstacle( Ex1Robot.verticesCCW(),  Ex1Obstacle.verticesCCW(), thetas);
    amp::Obstacle2D Ex1CSpace = constructCSpaceObstacle(Ex1Robot.verticesCCW(), Ex1Obstacle.verticesCCW());

    Visualizer::makeFigure(std::vector<Obstacle2D> {Ex1CSpace});
    Visualizer::makeFigure(Cspace3d, thetas);

    //Exercise 2***********************************
    std::vector<double> state3Link = {M_PI/6, M_PI/3, 7*M_PI/4};
    std::vector<double> lengthsA = {.5,1,.5};
    std::vector<double> lengthsB = {1,.5,1};

    MyManipulator Ex2FK(lengthsA);
    MyManipulator Ex2IK(lengthsB);

    Eigen::Vector2d endLocation(2,0);
    ManipulatorState IKState = Ex2IK.getConfigurationFromIK(endLocation);
    
    // Visualizer::makeFigure(Ex2FK, state3Link);
    // Visualizer::makeFigure(Ex2IK, IKState);

    //Exercise 3***********************************
    Visualizer::showFigures();

    // Grade method
    // amp::HW4::grade<MyManipulator>(constructCSpaceObstacle, "stli8113@colorado.edu", argc, argv);
    return 0;
}
