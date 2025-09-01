#ifndef GEO_PLANNER_H_
#define GEO_PLANNER_H_

#include <time.h>
#include <math.h>
//#include "Eigen/Core"
//#include "Eigen/Dense"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/bind.hpp>
//#include "ros/ros.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <functional>
#include <vector>
#include <geo_local_planner/obstacle_update.h>


#include <casadi/casadi.hpp>


namespace geo_local_planner
{
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace casadi;

typedef struct _OBSTACLE_STRUCT
{
    double X;
    double Y;
    double R1;
    double R2;
    double Theta;
} OBSTACLES;

class geo_planner
{
public:
    geo_planner();

    bool config_geo_planner();

    void caculate_geo_path(vector<geometry_msgs::PoseStamped>& robot_plan, 
                           std::vector<geometry_msgs::PoseStamped>& leftSidewalk_plan, 
                           std::vector<geometry_msgs::PoseStamped>& rightSidwalk_plan);



    //void geo_path_visualize(std::vector<geometry_msgs::PoseStamped> &geo_plan);
    void geo_path_visualize(std::vector<geometry_msgs::PoseStamped> &geo_plan, std::vector<geometry_msgs::PoseStamped> &polyfit_plan);

    void insertSegmentObs(b_segment seg, double inflation_rate); // 线段的障碍物转换成椭圆
    void insertCircleObs(circle circ);

    bool checkObstacleFootprintIntersection(const std::vector<geometry_msgs::PoseStamped> &plan, 
                                            const std::vector<geometry_msgs::Point> &_footprint_spec, const std::vector<b_segment> &seg_obs_, const std::vector<circle> &circ_obs_);
    
    Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);

    std::vector<double> nlp_params;
    std::vector<double> lbg, ubg;
    std::vector<double> lbu, ubu;
    std::vector<double> u0;

    // Obstacles
    std::vector<OBSTACLES> _obstacles_vec;

    bool obstaclePresent;

    // Solver
    Function solver;

    // NLP parameters
    double X_val, car_width_val, safe_dis_val, x_star_val, y_star_val, k_star_val;
    std::vector<double> global_coeff_val, 
                        lt_coeff_val, 
                        rt_coeff_val, 
                        obs_val; 

    //u_opt
    std::vector<double> u_opt;



protected:

private:

    double ns;
    double n_obs;
    double _seg_inflation_rate;



};
}  // namespace geo_local_planner
#endif 