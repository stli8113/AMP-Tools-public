#include "AMPCore.h"

// terribad copy of stuff from amp tools to try to get this to compile
/// @brief Vector of angles (radians) for each joint. The size of the vector should match the 
/// number of links (and hence joints) of the manipulator
using ManipulatorState = std::vector<double>;

/// @brief List of manipulator states in chronological order
using ManipulatorTrajectory = std::list<ManipulatorState>;

class MyManipulator : public amp::LinkManipulator2D{
    /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
    Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;
    /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
    /// @param end_effector_location End effector coordinate
    /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()    
    ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
};