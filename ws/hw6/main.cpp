#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"
#include "AStar.h"
#include "waveFront.h"
#include "MyManipulator.h"
#include "HelpfulClass.h"

#define _USE_MATH_DEFINES

using namespace amp;

int main(int argc, char** argv) {

    //get problems
    amp::Problem2D p1 = HW2::getWorkspace1(), p2 = HW6::getHW4Problem1(), p3 = HW6::getHW4Problem3();

    // initialize member vars for wavefront
    double xSize = .25, ySize = .25, xSizeManip = M_PI/50, ySizeManip = M_PI/50;

    // point wavefront stuff
    myPointWaveFront pointWavefront(xSize, ySize, p1.x_min, p1.y_min);
    amp::Path2D p1Path = pointWavefront.plan(p1);

    // manipulator wavefront initialization
    myManipulatorCSpaceConstructor manipulatorWaveFront;
    std::shared_ptr<myManipulatorCSpaceConstructor> CSpaceConstructorPointer = std::make_shared<myManipulatorCSpaceConstructor>(manipulatorWaveFront);
    myManipulatorWaveFront manipWavefront(CSpaceConstructorPointer, xSizeManip, ySizeManip, 0.0,0.0);
    MyManipulator manipulator;

    // manipulator paths
    amp::ManipulatorTrajectory2Link p2Path = manipWavefront.plan(manipulator, p2);
    amp::ManipulatorTrajectory2Link p3Path = manipWavefront.plan(manipulator, p3);

    // troubleshooting cspace creation for showing paths in cspace
    myManipulatorCSpaceConstructor test;
    std::unique_ptr<amp::GridCSpace2D> p3CSpace = test.construct( manipulator, p3);
    std::unique_ptr<amp::GridCSpace2D> p2CSpace = test.construct( manipulator, p2);
    // create figures
    Visualizer::makeFigure(p1, p1Path);

    Visualizer::makeFigure(*p3CSpace, p3Path);
    Visualizer::makeFigure(*p2CSpace, p2Path);
    Visualizer::makeFigure(p2, manipulator, p2Path);
    Visualizer::makeFigure(p3, manipulator, p3Path);

    std::cout << "path length 1: " << p1Path.length();

    // amp::ShortestPathProblem Ex3SPP = HW6::getEx3SPP();
    // amp::LookupSearchHeuristic Ex3Heuristic = HW6::getEx3Heuristic();

    // myAStarAlgo astar;
    // myAStarAlgo::GraphSearchResult result = astar.search(Ex3SPP, Ex3Heuristic);
    // int result_length = result.node_path.size();

    // for(int i = 0; i < result_length; i++){
    //     std::cout << result.node_path.front() << ", ";
    // }
    
    Visualizer::showFigures();

    return 0;
}