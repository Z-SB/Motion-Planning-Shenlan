#include "Astar_searcher.h"
#include  "math.h"
using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0); //(-5,-5,0)
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0); //(5,5,5)
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id; //(50,50,25)
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;  //0.2
    inv_resolution = 1.0 / _resolution;    //5

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    // 障碍物map data=1
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl; //(0,0,0) -> (-2.5,-2.5,2.5)
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt)   //(0,0,1) ->(25,25,5)
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
   //based on the coordinates: traverse the neighbors of currentPtr 
   //check if it is free
   Vector3i current_idx;
   current_idx = currentPtr->index;
   int current_x = current_idx(0);
   int current_y = current_idx(1);
   int current_z = current_idx(2);
   for(int i=1; i>=-1; i--)
   {
       for(int j=1; j>=-1; j--)
       {
           for(int k=1; k>=-1; k--)
           {
               if( i || j || k ) //excluding current node itself
               {
                   Vector3i nb_idx;
                   nb_idx(0) = current_x+i;
                   nb_idx(1) = current_y+j;
                   nb_idx(2) = current_z+k;

                   if(isFree(nb_idx)) // excluding obstacles and insuring index is in boundary
                   {
                        double edgeCost = sqrt(pow(current_x - nb_idx(0), 2) + pow(current_y - nb_idx(1), 2)+ pow(current_z - nb_idx(2), 2));
                        edgeCostSets.push_back(edgeCost);
                        neighborPtrSets.push_back(GridNodeMap[nb_idx(0)][nb_idx(1)][nb_idx(2)]);

                   }
               }
           }
       }
   }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
   // Dijkstra
//    double hn = 0;


   Vector3i start = node1->index;
   Vector3i end = node2->index;
   int dx = abs(start(0)-end(0));
   int dy = abs(start(1)-end(1));
   int dz = abs(start(2)-end(2));
   //Euclidean
//    double hn = sqrt(pow(dx, 2) + pow(dy, 2)+ pow(dz, 2));

   //Manhattan
//    double hn = dx+dy+dz;

//    Diagnal heuristic
   int a[3] = {dx,dy,dz};
   int dmin = dx;
   int dmax = dx;

    // int dmin = min(dx,dy,dz); no such function in math.h
    // int dmax = max(dx,dy,dz);
   for(int i=0; i<3; i++)
   {
       dmin = dmin>min(dmin, a[i])?min(dmin, a[i]):dmin;
       dmax = dmax<max(dmin, a[i])?max(dmin, a[i]):dmax;
   }
    int dmid = dx+dy+dz-dmin-dmax;
    double hn = (sqrt(3)-sqrt(2))*dmin + (sqrt(2)-1)*dmid + dmax;

    //simple tie breaker
    // hn = hn*(1+1/1000);
    return hn;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    
    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt); //index: coordinate in grid map
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;
    ROS_INFO("index of start point: %i, %i, %i", start_idx(0), start_idx(1), start_idx(2));
    ROS_INFO("index of end point: %i, %i, %i", end_idx(0), end_idx(1), end_idx(2));
    //position of start_point and end_point in real word coordinate frame
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);
    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    // ROS_INFO("[A*]{STEP1}  startPtr->fScore =  %f m", startPtr->fScore * resolution );
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
   //Update the startPtr in GridNodeMap
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] = startPtr;
    double tentative_gScore; // store temp gscore to compare with previous gscore of neighborPtr
    
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    // this is the main loop
    while (!openSet.empty()){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        int openSet_empty = 1;
        //set iterator to traverse in multimap: openSet
        multimap<double, GridNodePtr>::iterator nodeMapIt;
        nodeMapIt = openSet.begin();
        // int count = 0;
        while(nodeMapIt!=openSet.end())
        {
            // count++;
            if(nodeMapIt ->second->id == 1)
            {
                //set currentPtr to the node with lowest cost in openList
                currentPtr = nodeMapIt->second;
                //move it from openset to closed set
                currentPtr->id = -1;
                // indicates openset is not empty
                openSet_empty = 0; 
                break;
            }
            nodeMapIt++;
        }

        if(openSet_empty) // if openSet is empty, break
            break;

        // ROS_WARN("[A*]{Step3}  index of currentPtr in open list : %i, current fScore = %f. ",count, currentPtr->fScore* resolution);            
        // ROS_INFO("coord of current point: %f, %f, %f", currentPtr->coord(0), currentPtr->coord(1), currentPtr->coord(2));    

        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost is %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself         
        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            tentative_gScore = currentPtr->gScore+edgeCostSets[i];

            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               // set gScore, fScore, id, parentNode, and put it in openSet
               neighborPtr->gScore =  tentative_gScore;
               neighborPtr->fScore = tentative_gScore+getHeu(neighborPtr,endPtr);   
               neighborPtr-> id = 1;
               neighborPtr-> cameFrom = currentPtr;
               openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) );

            }
            else if(neighborPtr -> id == 1 && tentative_gScore<neighborPtr->gScore){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore =  tentative_gScore+getHeu(neighborPtr,endPtr);   
                neighborPtr-> cameFrom = currentPtr;
                openSet.insert( make_pair(neighborPtr->fScore, neighborPtr) );
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    GridNodePtr tempPtr;
    tempPtr = terminatePtr;
   do
   {
       gridPath.push_back(tempPtr);
       tempPtr = tempPtr->cameFrom;

   }while(tempPtr!=NULL);

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}