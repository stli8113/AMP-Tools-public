// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "MyManipulator.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    
    std::vector<double> teststate;
    teststate.push_back(0.5);
    teststate.push_back(0.5);
    MyManipulator testlink;
    Visualizer::makeFigure(testlink, teststate);
    std::cout <<"test";

    // Grade method
    //amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}

Eigen::Vector2d MyManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index){
    Eigen::Vector3d JointLoc(0,0,1);

    for(uint32_t i = 0; i <= joint_index; i++){
        Eigen::MatrixXd T = LinkTransform(state(i), m_link_lengths(i));
        JointLoc = T*JointLoc;
    }
    Eigen::Vector2d location(JointLoc(0), JointLoc(1));
    return location;
}
using ManipulatorState = std::vector<double>;
ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location){
    //immense pain inbound
    //assume 3 link manipulator
    //create return variable
    std::vector<double> state;

    //initialize and define 2 link problem for first 2 links
    double x3 = end_effector_location(0);
    double y3 = end_effector_location(1);
    double lastlength = m_link_lengths.back();
    double x2 = x3;
    double y2 = std::sqrt(std::pow(lastlength,2) - std::pow((x2 - x3), 2)) - y3; //CHECK REACH

    double phi;
    phi = std::atan2(x2-x3, y2-y3); //find total angle of last link to backsolve later

    //solve 2 link problem

    return state    
}

//@brief creates a transformation matrix for a link in a chain
//@param theta relative rotation angle of link to last frame
//@param linkLength length of link
//@return 3x3 homogenous tranformation matrix
Eigen::MatrixXd LinkTransform(double theta, double linkLength){
    Eigen::MatrixXd m(3,3);
    m(0,0) = std::cos(theta);
    m(1,0) = std::sin(theta);
    m(2,0) = 0;
    m(0,1) = -std::sin(theta);
    m(1,1) = std::cos(theta);
    m(2,1) = 0;
    m(2,0) = linkLength;
    m(2,1) = 0;
    m(2,2) = 1;
    // std::cout << "theta: "<< theta << "\n matrix:" <<  m << std::endl;
    return m;
}