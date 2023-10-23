#include "AMPCore.h"
#include "MyManipulator.h"
#include "HelpfulClass.h"

Eigen::Vector2d MyManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const{
    Eigen::Vector3d JointLoc(0,0,1);
    // std::cout <<"\n index:"<< joint_index<<std::endl;
    uint32_t i = joint_index;
    
    //check if base joint
    if (joint_index == 0){
        // std::cout << "base joint \n";
        Eigen::Vector2d location(0,0);
        return location;
    }

    //calculate last translation
    Eigen::MatrixXd T = LinkTransform(0, m_link_lengths[joint_index-1]);
    JointLoc = T*JointLoc;
    //do intermediate transforms
    while(i>1){
        // std::cout <<i;
        Eigen::MatrixXd T = LinkTransform(state[i-1], m_link_lengths[i-2]);
        JointLoc = T*JointLoc;
        i--;
    }
    //find initial tranform T1
    // std::cout << "loop done \n";
    T = LinkTransform(state[0],0);
    JointLoc = T*JointLoc;

    Eigen::Vector2d location(JointLoc(0), JointLoc(1));
    // std::cout << location << std::endl;
    location += m_base_location;
    return location;
}

ManipulatorState MyManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const{
    //immense pain inbound
    //create return variable
    
    ManipulatorState state(m_link_lengths.size());
    double x2;
    double y2;
    double phi;

    if(m_link_lengths.size() == 3){
        //initialize and define 2 link problem for first 2 links
        std::cout << "3 links \n";
        double x3 = end_effector_location(0);
        double y3 = end_effector_location(1);
        double lastlength = m_link_lengths.back();
        x2 = x3-lastlength;
        y2 = -std::sqrt(pow(lastlength,2) - pow((x2 - x3), 2)) + y3; //CHECK REACH

        // while(){
        //     x2 += .1;
        //     y2 = -std::sqrt(pow(lastlength,2) - pow((x2 - x3), 2)) + y3; 
        // }


        phi = std::atan2( y3-y2,x3-x2); //find total angle of last link to backsolve later
        // std::cout << "x2,y2: " << x2 << " "<<y2 << std::endl;
    }else if(m_link_lengths.size() ==2){
        std::cout << "2 links \n";
        x2 = end_effector_location(0);
        y2 = end_effector_location(1);
    }
    else{
        //returns 0s for higher number of links
        std::cout << "More than 3 links detected, returning 0 \n";
        state.setZero();
        return state;
    }

    //solve 2 link problem
    double a1 = m_link_lengths[0];
    double a2 = m_link_lengths[1];

    double cost2 = 1/(2*a1*a2) * ((pow(x2,2) + pow(y2,2))-(pow(a1,2) + pow(a2,2)));
    double sint2 = sqrt(1 - pow(cost2,2));
    double cost1 = 1/(pow(x2,2) + pow(y2,2)) * (x2 * (a1 + a2*cost2)) + y2 * a2 * std::sqrt(1 - pow(cost2,2));
    double sint1 = 1/(pow(x2,2) + pow(y2,2)) * (y2 * (a1 + a2*cost2)) - x2 * a2 * std::sqrt(1 - pow(cost2,2));

    // std::cout << "\nacos1: "<< std::acos(cost1) << "\nasin1: " << std::asin(sint1) << std::endl;
    //calculate inverse trig then quadrant check
    double acos1 = std::acos(cost1);
    double acos2 = std::acos(cost2);
    double asin1 = std::asin(sint1);

    double theta2 = acos2; //second angle we say is positive
    double theta1;
    double theta3;
    if(asin1 >= 0){//DEALS WITH THETA1!!
        theta1 = acos1;
    }
    else if(acos1 <= M_PI/2 && asin1 < 0){
        theta1 = asin1;
    }else{
        theta1 = - acos1;
    }
    
    state(0) = theta1;
    state(1) = theta2;
    if(m_link_lengths.size() == 3){
        theta3 = phi - std::fmod((theta1 + theta2),(2*M_PI));
        state(2) = theta3;
    }
    // std::cout << "thetas:\n" << theta1 << std::endl << theta2 << std::endl << theta3;
    return state;
}

