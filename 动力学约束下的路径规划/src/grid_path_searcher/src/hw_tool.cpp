#include <hw_tool.h>
#include <ros/ros.h>
using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,Eigen::Vector3d _start_velocity,Eigen::Vector3d _target_position)
{
    double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it 
    /*
                    



    STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
    the solving process has been given in the document

    because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP

    after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory


    */
   VectorXd delta_S(6);
   VectorXd lamda(6); 
   MatrixXd Trans(6,6);
   double T, alpha1, alpha2, alpha3, belta1, belta2, belta3, J_cost, J_dt;
   double Pix, Piy, Piz, Vix, Viy, Viz, Pfx, Pfy, Pfz;
    // 提取出已知量
   Pix = _start_position(0);
   Piy = _start_position(1);
   Piz = _start_position(2);
   Vix = _start_velocity(0);
   Viy = _start_velocity(1);
   Viz = _start_velocity(2);

   Pfx = _target_position(0);
   Pfy = _target_position(1);
   Pfz = _target_position(2);

    T = 0.0001; //初始化时间
    // 将已知量带入J对于T的求导后的方程, 暴力求解T值
    do
    {
        J_dt = (-36*pow(Pfx,2) - 36*pow((Pfz - Piz),2) + pow(T,4) + 24*Pfx*(3*Pix+T*Vix) - 4*pow((3*Pix+T*Vix),2) 
            - 4*pow((-3*Pfy+3*Piy+T*Viy),2) + 24*(Pfz-Piz)*T*Viz - 4*pow(T, 2)*pow(Viz, 2))/pow(T,4);
        if(abs(J_dt)<0.001)   //  求出使得J_dt大概为0时的T值
        {
            // ROS_INFO(" break at time: %f", T);
            break;
        }
        else
            T += 0.001;
    }while(T<20);
    // 将T值带回求得J
    delta_S(0) =  Pfx - Vix*T- Pix;
   delta_S(1) =  Pfy - Viy*T- Piy;
   delta_S(2) =  Pfz - Viz*T- Piz;
   delta_S(3) = -Vix;
   delta_S(4) = -Viy;
   delta_S(5) = -Viz;

    Trans << -12/pow(T,3), 0, 0, 6/pow(T,2), 0, 0,
                     0, -12/pow(T,3),  0, 0, 6/pow(T,2), 0,
                     0, 0, -12/pow(T,3),  0,  0, 6/pow(T,2),
                    6/pow(T,2), 0, 0, -2/T, 0, 0,
                    0, 6/pow(T,2), 0, 0, -2/T, 0,
                    0, 0, 6/pow(T,2), 0, 0, -2/T;
    lamda = Trans*delta_S;

    alpha1 = lamda(0);
    alpha2 = lamda(1);
    alpha3 = lamda(2);
    belta1 = lamda(3);
    belta2 = lamda(4);
    belta3 = lamda(5);

    //计算出J在T时刻的cost(大约极小值)即启发方程-> 考虑运动学 不考虑障碍物的函数
    J_cost = T+(1/3*pow(alpha1,2)*pow(T,3)+ alpha1*belta1*pow(T,2)+ pow(belta1,2)*T)
    + (1/3*pow(alpha2,2)*pow(T,3)+ alpha2*belta2*pow(T,2)+ pow(belta2,2)*T)
    + (1/3*pow(alpha3,2)*pow(T,3)+ alpha3*belta3*pow(T,2)+ pow(belta3,2)*T);
    
    optimal_cost = abs(J_cost);

    return optimal_cost;
}
