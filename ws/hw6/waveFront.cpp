#include "AMPCore.h"
#include "waveFront.h"
#include "HelpfulClass.h"
#include "MyManipulator.h"
#include <queue>

#define _USE_MATH_DEFINES

// point robot implementation
amp::Path2D myPointWaveFront::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    amp::Path2D path;
    auto gridSize = grid_cspace.size();
    std::pair<std::size_t, std::size_t> cellInit = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    std::pair<std::size_t, std::size_t> cellGoal = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1));
    // std::cout << "cellinit: " << cellInit.first << ", " << cellInit.second << std::endl;
    // std::cout << "starting pathing...\n";
    // std::cout << "start cell: " << cellInit.first << ", " << cellInit.second << "\nend cell: " << cellGoal.first << ", " << cellGoal.second << std::endl;

    std::pair<std::size_t, std::size_t> waveGrid[gridSize.first][gridSize.second]; //grid of pairs, each element points to previous cell in wavefront
    int testGrid[gridSize.first][gridSize.second]; //testing grid for shenanigans

    //initialize for obstacles
    for(std::size_t i = 0; i < gridSize.first; i++){
        for(std::size_t j = 0; j < gridSize.second; j++){
            if(grid_cspace(i,j)){
                waveGrid[i][j] = std::make_pair(gridSize.first+1,0); //prevents the grid from having a open space with same value as q_init or q_goal
                testGrid[i][j] = 1;
            }
            else{
                waveGrid[i][j] = std::make_pair(0,gridSize.second + 1);
                testGrid[i][j] = 0;
            }
        }
    }
    std::pair<size_t,size_t>freeSpace(0,gridSize.second + 1);
    // for(int i = 0; i < gridSize.first;i++){
    //     for(int j=0; j < gridSize.second;j++){
    //         std::cout <<std::setw (2) << testGrid[i][j] << " ";
    //     }
    //     std::cout<<std::endl;
    // }
    // std::cout << "\n\n\n";


    //initialize cell queue
    std::queue<std::pair<std::size_t, std::size_t> > cellToCheck;
    cellToCheck.push(cellGoal);
    waveGrid[cellGoal.first][cellGoal.second] = cellGoal;
    // testGrid[cellGoal.first][cellGoal.second] = 2;
    int dr[] = {1,0,-1,0};
    int dc[] = {0,1,0,-1};

    while(!cellToCheck.empty()){
        // std::cout << "beginning neighbor check...\n";
        std::pair<std::size_t, std::size_t> cell = cellToCheck.front(); //initialize current cell to check around for readablility

        for (int i = 0; i <= 4; i++) {
            // std::cout << "gonna subtract from size_t\nrow: " << cell.first << "\ncol: " << cell.second<< std::endl;
            int newRow = cell.first + dr[i];
            int newCol = cell.second + dc[i];
            // std::cout << "checking neighbors...\n";
            if (newRow >= 0 && newRow < gridSize.first && newCol >= 0 && newCol < gridSize.second) {
                if(waveGrid[newRow][newCol] == freeSpace){
                    // std::cout << "valid index\n";
                    waveGrid[newRow][newCol] = cell;
                    std::pair<size_t,size_t> newCell(newRow,newCol);
                    // std::cout << "waveGrid(i,j): " << waveGrid[newRow][newCol].first << ", "<<waveGrid[newRow][newCol].second << std::endl;


                    //check if the current cell is the goal, leave loops if so
                    if(newCell == cellInit){
                        // std::cout << "cellInit found\n";
                        goto endPropogate;
                    }
                    cellToCheck.push(newCell);
                }
                // if(testGrid[newRow][newCol] == 0){
                //     testGrid[newRow][newCol] = testGrid[cell.first][cell.second] + 1;
                // }
            }
            // std::cout << "neighbor out of bounds!\ncell: " << cell.first << ", " << cell.second << std::endl;
        }

        //pop checked cell
        cellToCheck.pop();
        // std::cout << "popped!\nqueue size: " << cellToCheck.size() << "queue empty: " << cellToCheck.empty() << std::endl;
    }
    
    // std::cout << "loop done\n";
    endPropogate:
    // testGrid[cellGoal.first][cellGoal.second] = 99;
    // for(int i = 0; i < gridSize.second;i++){
    //     for(int j=0; j < gridSize.first;j++){
    //         std::cout <<std::setw (2) << testGrid[j][gridSize.second-i-1] << " ";
    //     }
    //     std::cout<<std::endl;
    // }
    std::cout << "creating path...\n";
    path.waypoints.push_back(q_init);
    Eigen::Vector2d centerpoint(m_cell_size_x * (cellInit.first + 0.5) + m_x_min,  m_cell_size_y * (cellInit.second + 0.5) + m_y_min);
    path.waypoints.push_back(centerpoint);
    std::pair<size_t,size_t> cell(cellInit), nextCell;

    do{
        // std::cout << "finding next cell\ncell: " << cell.first << ", " << cell.second << std::endl;
        nextCell = waveGrid[cell.first][cell.second];

        // std::cout << "before centerpoint calc....\n";
        centerpoint << m_cell_size_x * (nextCell.first + 0.5) + m_x_min, m_cell_size_y * (nextCell.second + 0.5) + m_y_min;
        // std::cout << "centerpoint inserted\n";
        path.waypoints.push_back(centerpoint);
        cell = nextCell;
        // std::cout << "nextCell assigned\n";

    }while(cell != cellGoal);
    // std::cout << "path done\n";
    
    path.waypoints.push_back(q_goal);

    return path;
}

// manipulator 2 link arm implementation
amp::Path2D myManipulatorWaveFront::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    amp::Path2D path;
    auto gridSize = grid_cspace.size();
    std::pair<std::size_t, std::size_t> cellInit = grid_cspace.getCellFromPoint(q_init(0), q_init(1));
    std::pair<std::size_t, std::size_t> cellGoal = grid_cspace.getCellFromPoint(q_goal(0), q_goal(1));
    // std::cout << "cellinit: " << cellInit.first << ", " << cellInit.second << std::endl;
    // std::cout << "starting pathing...\n";
    std::cout << "start cell: " << cellInit.first << ", " << cellInit.second << "\nend cell: " << cellGoal.first << ", " << cellGoal.second << std::endl;

    std::pair<std::size_t, std::size_t> waveGrid[gridSize.first][gridSize.second]; //grid of pairs, each element points to previous cell in wavefront
    int testGrid[gridSize.first][gridSize.second]; //testing grid for shenanigans

    //initialize for obstacles
    for(std::size_t i = 0; i < gridSize.first; i++){
        for(std::size_t j = 0; j < gridSize.second; j++){
            if(grid_cspace(i,j)){
                waveGrid[i][j] = std::make_pair(gridSize.first+1,0); //prevents the grid from having a open space with same value as q_init or q_goal
                testGrid[i][j] = 1;
            }
            else{
                waveGrid[i][j] = std::make_pair(0,gridSize.second + 1);
                testGrid[i][j] = 0;
            }
        }
    }
    std::pair<size_t,size_t>freeSpace(0,gridSize.second + 1);
    // for(int i = 0; i < gridSize.first;i++){
    //     for(int j=0; j < gridSize.second;j++){
    //         std::cout <<std::setw (2) << testGrid[i][j] << " ";
    //     }
    //     std::cout<<std::endl;
    // }
    // std::cout << "\n\n\n";


    //initialize cell queue
    std::queue<std::pair<std::size_t, std::size_t> > cellToCheck;
    cellToCheck.push(cellGoal);
    waveGrid[cellGoal.first][cellGoal.second] = cellGoal;
    testGrid[cellGoal.first][cellGoal.second] = 2;
    int dr[] = {1,0,-1,0};
    int dc[] = {0,1,0,-1};

    while(!cellToCheck.empty()){
        // std::cout << "beginning neighbor check...\n";
        std::pair<std::size_t, std::size_t> cell = cellToCheck.front(); //initialize current cell to check around for readablility

        for (int i = 0; i <= 4; i++) {
            // std::cout << "gonna subtract from size_t\nrow: " << cell.first << "\ncol: " << cell.second<< std::endl;
            int newRow = cell.first + dr[i];
            int newCol = cell.second + dc[i];
            // std::cout << "checking neighbors...\n";
            if(waveGrid[newRow][newCol] == freeSpace){
                // std::cout << "valid index\n";
                waveGrid[newRow][newCol] = cell;
                std::pair<size_t,size_t> newCell(newRow,newCol);
                // std::cout << "waveGrid(i,j): " << waveGrid[newRow][newCol].first << ", "<<waveGrid[newRow][newCol].second << std::endl;


                //check if the current cell is the goal, leave loops if so
                if(newCell == cellInit){
                    // std::cout << "cellInit found\n";
                    goto endPropogate;
                }
                cellToCheck.push(newCell);
            }
            // if(testGrid[newRow][newCol] == 0){
            //     testGrid[newRow][newCol] = testGrid[cell.first][cell.second] + 1;
            // }
            
            // std::cout << "neighbor out of bounds!\ncell: " << cell.first << ", " << cell.second << std::endl;
        }

        //pop checked cell
        cellToCheck.pop();
        // std::cout << "popped!\nqueue size: " << cellToCheck.size() << "queue empty: " << cellToCheck.empty() << std::endl;
    }
    
    // std::cout << "loop done\n";
    endPropogate:
    // testGrid[cellGoal.first][cellGoal.second] = 99;
    // for(int i = 0; i < gridSize.second;i++){
    //     for(int j=0; j < gridSize.first;j++){
    //         std::cout <<std::setw (2) << testGrid[j][gridSize.second-i-1] << " ";
    //     }
    //     std::cout<<std::endl;
    // }
    // std::cout << "creating path...\n";
    path.waypoints.push_back(q_init);
    Eigen::Vector2d centerpoint(m_cell_size_x * (cellInit.first + 0.5) + m_x_min,  m_cell_size_y * (cellInit.second + 0.5) + m_y_min);
    path.waypoints.push_back(centerpoint);
    std::pair<size_t,size_t> cell(cellInit), nextCell;

    do{
        // std::cout << "finding next cell\ncell: " << cell.first << ", " << cell.second << std::endl;
        nextCell = waveGrid[cell.first][cell.second];

        // std::cout << "before centerpoint calc....\n";
        centerpoint << m_cell_size_x * (nextCell.first + 0.5) + m_x_min, m_cell_size_y * (nextCell.second + 0.5) + m_y_min;
        // std::cout << "centerpoint inserted\n";
        path.waypoints.push_back(centerpoint);
        cell = nextCell;
        // std::cout << "nextCell assigned\n";

    }while(cell != cellGoal);
    // std::cout << "path done\n";
    
    path.waypoints.push_back(q_goal);

    return path;
}

// C space constructors========================

std::unique_ptr<amp::GridCSpace2D> myPointWaveFront::constructDiscretizedWorkspace(const amp::Environment2D& environment){
    std::size_t xSize = ceil((environment.x_max - environment.x_min)/m_cell_size_x);
    std::size_t ySize = ceil((environment.y_max - environment.y_min)/m_cell_size_y);

    myCSpace2D Cspace(xSize, ySize, environment.x_min, environment.x_max, environment.y_min, environment.y_max, m_cell_size_x, m_cell_size_y);

    for (int i= 0; i<xSize; i++){
        for(int j=0; j<ySize; j++){
            Cspace(i,j) = obstacleInCell(environment, i, j);
            if(Cspace(i,j)){
                // std::cout << "obstacle found at: " << i << ", " << j << std::endl;
            }
        }
    }
    return std::make_unique<myCSpace2D>(Cspace);

}

std::unique_ptr<amp::GridCSpace2D> myManipulatorCSpaceConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
    std::vector<double> m_link_lengths = manipulator.getLinkLengths();
    std::vector<amp::Obstacle2D> obstacles = env.obstacles;

    std::size_t x0_cells = 100;
    std::size_t x1_cells = 100;
    double x0_cell_size =  2*M_PI / x0_cells;
    double x1_cell_size =  2*M_PI / x1_cells;
    myCSpace2D Cspace(x0_cells, x1_cells, 0.0, 2*M_PI , 0.0, 2*M_PI, x0_cell_size, x1_cell_size); //create 100x100 grid for cspace
    // myCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max, double xCellSize, double yCellSize)

    ManipulatorState2Link state;
    //check collision of every configuration
    for (int i = 0; i < x0_cells; i++){
        for (int j = 0; j < x1_cells; j++){
            // std::cout << i;
            double x0 = (i+1)*2*M_PI / x0_cells;
            double x1 = (j+1)*2*M_PI / x1_cells;
            state << x0, x1;
            Cspace(i,j) = manipulatorCollision(manipulator, env, state);
        }
    }
    std::unique_ptr<myCSpace2D> CSpacePointer = std::make_unique<myCSpace2D>(Cspace);
    return CSpacePointer;
}

// collision checking============================

bool myPointWaveFront::obstacleInCell(const amp::Environment2D& environment, int row, int col){


    //check centerpoint for obstacle
    Eigen::Vector2d centerpoint(m_cell_size_x * (row + 0.5) + environment.x_min, m_cell_size_y * (col + 0.5) + environment.y_min);
    std::vector<amp::Obstacle2D> obstacles = environment.obstacles;
    for(int i = 0; i<obstacles.size(); i++){
        if(isInObstacle(centerpoint, obstacles[i])){
            // std::cout << "collision detected at: " << centerpoint(0) << ", " << centerpoint(1) << std::endl;
            return true;
        }
    }
    return false;
}

std::pair<std::size_t, std::size_t> myCSpace2D::getCellFromPoint(double x0, double x1) const{
    std::size_t xCell, yCell;
    xCell = floor((x0 - x_min) / m_cell_size_x);
    yCell = floor((x1 - y_min) / m_cell_size_y);

    // std::cout << "xCell: " << xCell << "\nyCell: " << yCell << std::endl;
    std::pair<std::size_t, std::size_t> cell(xCell, yCell);
    return cell;
}

bool myManipulatorCSpaceConstructor::manipulatorCollision(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env, ManipulatorState state){
    // std::cout << "state: " << state[0] << ", " << state[1] << std::endl;

    // check each joint location for collision
    // std::cout << "checking joints\n";
    for(uint32_t i=0; i <= manipulator.nLinks(); i++){
        Eigen::Vector2d joint(manipulator.getJointLocation(state, i));
        // std::cout << "joint location: " << joint(0) << ", " << joint(1) << std::endl;
        for(int j = 0; j < env.obstacles.size(); j++){
            if(isInObstacle(joint, env.obstacles[j])){
                // std::cout << "joint in obstacle!\n";
                // std::cout << "joint location: " << joint(0) << ", " << joint(1) << std::endl;
                return true;
            }
        }
    }
    // check each link for intersection with obstacle
    // std::cout << "checking links\n";
    for(uint32_t i=0; i < manipulator.nLinks(); i++){
        Eigen::Vector2d joint1(manipulator.getJointLocation(state, i));
        Eigen::Vector2d joint2(manipulator.getJointLocation(state, i+1));
        for(int j = 0; j < env.obstacles.size(); j++){
            if(lineIntersectingPolygon(env.obstacles[j], joint1, joint2)){
                // std::cout<< "link in obstacle!\n";
                // std::cout << "joint1 location: " << joint1(0) << ", " << joint1(1) << std::endl;
                // std::cout << "joint2 location: " << joint2(0) << ", " << joint2(1) << std::endl;
                return true;
            }
        }
    }
    // std::cout<< "no collisions found\n";
    return false;
}