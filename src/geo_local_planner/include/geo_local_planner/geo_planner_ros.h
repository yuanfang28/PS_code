#ifndef GEO_PLANNER_ROS_H_
#define GEO_PLANNER_ROS_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <mbf_msgs/ExePathResult.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <obstacle_detector/Obstacles.h>

#include <geo_local_planner/geo_planner.h>
#include <geo_local_planner/visualization.h>
#include <geo_local_planner/geo_planner_config.h>

using namespace std;
using namespace obstacle_detector;
// namespace bg = boost::geometry;

namespace geo_local_planner{

class GeoPlannerROS : public nav_core::BaseLocalPlanner{
public:
    GeoPlannerROS();

    GeoPlannerROS(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~GeoPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &geo_plan,
                                  double min_separation_via, 
                                  double min_separation_goal);

    bool isGoalReached();

protected:

    template <typename P1, typename P2> //将来使用的时候，定义点是任何类型都能使用计算举例的方程
    inline double distancePoints2d(const P1 &point1, const P2 &point2) 
    {
        return std::sqrt(std::pow(point2.x - point1.x, 2) + std::pow(point2.y - point1.y, 2));
    }; //用inline，以后使用这个函数的时候，直接作为源代码插入。如果不用，就是找到这个代码位置，然后使用。提高计算性能。小的函数可以使用


    bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                    std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=0.1);

    //bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
     //               std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=0.1, double dist_ahead_robot=4);


    /**
     * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
     *
     * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
     * such that the index of the current goal pose is returned as well as
     * the transformation between the global plan and the planning frame.
     * @param tf A reference to a tf buffer
     * @param global_plan The plan to be transformed
     * @param global_pose The global pose of the robot
     * @param costmap A reference to the costmap being used so the window size for transforming can be computed
     * @param global_frame The frame to transform the plan to
     * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also
     * bounded by the local costmap size!]
     * @param[out] transformed_plan Populated with the transformed plan
     * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
     * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
     * @return \c true if the global plan is transformed, \c false otherwise
     */
    /*bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, const std::string& robot_frame,
                             double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<geometry_msgs::PoseStamped>& robot_plan, 
                             int* current_goal_idx = NULL, double* path_length = NULL, geometry_msgs::TransformStamped* tf_plan_to_base = NULL) const;*/

    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, const std::string& robot_frame,
                             double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<geometry_msgs::PoseStamped>& robot_plan, 
                             std::vector<geometry_msgs::PoseStamped>& leftsidewalk_plan_,
                             std::vector<geometry_msgs::PoseStamped>& rightsidewalk_plan_,
                             int* current_goal_idx = NULL, double* path_length = NULL, 
                             geometry_msgs::TransformStamped* tf_plan_to_base = NULL) const;

    void obstacles_callback(const obstacle_detector::Obstacles::ConstPtr new_obstacles);

    double estimateRefPathOrientation(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, int moving_average_length) const;

    //void leftSidewalk_pts_callback (const sensor_msgs::PointCloud2ConstPtr& cloud);
    //void rightSidewalk_pts_callback (const sensor_msgs::PointCloud2ConstPtr& cloud);



    void puresuitControl(double rho, double alpha, double phi, double &v, double &omega);
    double clip(double value, double lower, double upper);
    void cart2Pol(double x, double y, double &deg, double &dist);
    double angDiff(double alpha, double beta);
    void poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi);


private:

    void reconfigureCB(GeoLocalPlannerReconfigureConfig &config, uint32_t level);

    //pointer
    costmap_2d::Costmap2DROS *_costmap_ros;
    costmap_2d::Costmap2D *_costmap;
    tf2_ros::Buffer *tf_;



    tf::TransformListener _tf_listener;

    double control_duration_;
    //flag
    bool initialized_;
    bool reached_goal_;
    bool last_back_;
    

    //odom_helper
    base_local_planner::OdometryHelperRos odom_helper_;

    
    // geo_planner
    geo_planner _geo_planner;

    //frame
    std::string global_frame_; ///< @brief The frame in which the controller will run
    std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
    std::string camera_frame_;

    //robot
    geometry_msgs::Twist last_cmd_;         //!< Store last velocity command
    geometry_msgs::Twist robot_vel_;        //!< Store current robot translational and angular velocity (vx, vy, omega)
    geometry_msgs::PoseStamped robot_pose_; //!< Store current robot pose
    
    
    //viapoint
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> via_points_;

    //path
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> leftsidewalk_plan_, rightsidewalk_plan_;
    std::vector<geometry_msgs::PoseStamped> local_plan_;

    //footprint
    std::vector<geometry_msgs::Point> _footprint_spec;  

    //Obstacles
    std::vector<b_segment> seg_obs_;
    std::vector<circle> circ_obs_;
    

    //reconfiguretion
    std::mutex obslock;
    boost::shared_ptr<dynamic_reconfigure::Server<GeoLocalPlannerReconfigureConfig>> dynamic_recfg_;
    GeoLocalConfig cfg_;

    //subscriber
    ros::Subscriber left_side_sub, right_side_sub, joy_sub_, obstacles_sub_;

    //pub
    ros::Publisher geo_path_pub, pose_pub, obs_center_pub, obs_radius_pub;



    //Visualisation
    PlannerVisualizationPtr visualization_;
    PlannerVisualizationPtr visualization_local;

    
    geometry_msgs::Point robot_position;
    std::vector<geometry_msgs::Point> robot_position_all;



};
}
#endif