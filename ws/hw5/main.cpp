#include "AMPCore.h"
#include "gradientDescent.h"
#include "hw/HW5.h"
#include "hw/HW2.h"

using namespace amp;

int main(int argc, char** argv) {
    myGradientDescent algo(.05, .05, .2, 1, .25); //for some reason grade() only works if Q* = 1
    amp::Problem2D Workspace1 = HW5::getWorkspace1();
    amp::Problem2D Workspace2 = HW2::getWorkspace1();
    amp::Problem2D Workspace3 = HW2::getWorkspace2();


    amp::Path2D path;
    path = algo.plan(Workspace2);
    // // std::cout << "Path Length: " << path.length() << std::endl;

    // HW5::grade(algo, "stli8113@colorado.edu",  argc,  argv);
    // // HW5::generateAndCheck(algo);

    Visualizer::makeFigure(Workspace2, path);
    Visualizer::showFigures();

    return 0;
}