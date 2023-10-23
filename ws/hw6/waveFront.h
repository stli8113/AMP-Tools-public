#pragma once

#include "MyManipulator.h"
#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW4.h"

class myPointWaveFront : public amp::PointWaveFrontAlgorithm{
    public:
        myPointWaveFront(double xSize, double ySize, double xmin, double ymin):
        amp::PointWaveFrontAlgorithm(),
        m_cell_size_x(xSize), m_cell_size_y(ySize), m_x_min(xmin), m_y_min(ymin) {}

        /// @brief Create a discretized planning space of an environment (point agent). If you abstracted your GridCSpace2DConstructor, you may be
        /// able to use that code here to construct a discretized C-space for a point agent.
        /// @param environment Workspace and/or C-space (point agent)
        /// @return Unique pointer to a GridCSpace2D object (see HW4)
        std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;

        /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
        /// @param grid_cspace Your grid discretization C-space from HW4. 
        /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
        /// NOTE: For Exercise 2), You can use 
        /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of 
        /// representative points for each cell in the sequence.
        amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
        
        ///@brief checks if a cell is has an obstacle inside it
        bool obstacleInCell(const amp::Environment2D& environment, int row, int col);

        private:
            double m_cell_size_x, m_cell_size_y, m_x_min, m_y_min;
};

class myManipulatorWaveFront : public amp::ManipulatorWaveFrontAlgorithm{
    public:
        myManipulatorWaveFront(const std::shared_ptr<amp::GridCSpace2DConstructor>& c_space_constructor, double xSize, double ySize, double xmin, double ymin) :
        ManipulatorWaveFrontAlgorithm(c_space_constructor),
        m_cell_size_x(xSize), m_cell_size_y(ySize), m_x_min(xmin), m_y_min(ymin){}

        /// @brief Return a non-colliding path through a grid C-space using the WaveFront algorithm. Override this method and implement your WaveFront planner
        /// @param grid_cspace Your grid discretization C-space from HW4. 
        /// NOTE: For Exercise 1), you will need to manually construct the discretization for HW2 Exercise2.
        /// NOTE: For Exercise 2), You can use 
        /// @return A path inside the C-space. Your WaveFront planner will find a sequence of cells. This path should be a sequence of 
        /// representative points for each cell in the sequence.
        amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;

    private:
        double m_cell_size_x, m_cell_size_y, m_x_min, m_y_min;
};

class myManipulatorCSpaceConstructor : public amp::GridCSpace2DConstructor {
    public:
        /// @brief Create a configuration space object given a maniplator and an environment.
        /// @param manipulator Two link manipulator (consider ussing `ASSERT` to make sure the manipulator is 2D)
        /// @param env Environment
        /// @return Unique pointer to your constructed C-space object. 
        /// NOTE: We use a unique pointer here to be able to move the C-space without copying it, since grid discretization
        /// C-spaces can contain a LOT of memory, so copying would be a very expensive operation. Additionally, a pointer is polymorphic
        /// which allows the type to pose as a GridCSpace2D (even though GridCSpace2D is abstract)
        std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

    // private:
        ///@brief checks if a manipulator state is colliding in a workspace
        bool manipulatorCollision(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env, ManipulatorState state);
};

class myCSpace2D : public amp::GridCSpace2D{
    public:
        myCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max, double xCellSize, double yCellSize):
        amp::GridCSpace2D(x0_cells, x1_cells,  x0_min,  x0_max,  x1_min,  x1_max),
        m_cell_size_x(xCellSize), m_cell_size_y(yCellSize), x_min(x0_min), y_min(x1_min){}
        /// @brief Given a point in continuous space that is between the bounds, determine what cell (i, j) that the point is in
        /// @param x0 Value of the first configuration space variable
        /// @param x1 Value of the second configuration space variable
        /// @return A pair (i, j) of indices that correspond to the cell that (x0, x1) is in
        std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

        ///@brief finds the centerpoint of the cell that position q occupies, might not be neccesary?
        // Eigen::Vector2d getCellCenter(std::pair<size_t,size_t> cell);
    private:
        double m_cell_size_x, m_cell_size_y, x_min, y_min;
};