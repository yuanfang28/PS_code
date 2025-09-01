#ifndef GEO_PLANNER_CONFIG_H_
#define GEO_PLANNER_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <geo_local_planner/GeoLocalPlannerReconfigureConfig.h>

namespace geo_local_planner
{
class GeoLocalConfig
{
    public:

        std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
        std::string map_frame; //!< Global planning frame
        //! Robot related parameters
        struct Robot
        {
            double max_vel_x;           //!< Maximum translational velocity of the robot
            double max_vel_x_backwards; //!< Maximum translational velocity of the robot for driving backwards
            double max_vel_theta;       //!< Maximum angular velocity of the robot
            double acc_lim_x;           //!< Maximum translational acceleration of the robot
            double acc_lim_theta;       //!< Maximum angular acceleration of the robot
            double min_turn_radius;     //!< Minimum turning radius of the robot
            bool turn_around_priority;  //!< If true, the robot preferentially adjusts the orientation to fit the direction of the path
            double stop_dist;           //!< When the Euclidean distance between the nearst lethal point on planned path and the robot frame origin is less than this distance, the robot stops
            double dec_dist;            //!< When the Euclidean distance between the nearst lethal point on planned path and the robot frame origin is less than this distance, the robot slows down
        } robot;

        //! Trajectory related parameters
        struct Trajectory
        {
            double max_local_plan_lookahead_dist; //!< Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if <=0: disabled; the length is also bounded by the local costmap size!]
            double local_plan_viapoint_sep;       //!< Min. separation between each two consecutive via-points extracted from the global plan (if negative: disabled)
            double local_plan_goal_sep;           //!< Min. separation between the last via-point and goal pose
            double local_plan_prune_distance;     //!< Distance between robot and via_points of global plan which is used for pruning
        } trajectory;


        //! Goal tolerance related parameters
        struct GoalTolerance
        {
            double yaw_goal_tolerance; //!< Allowed final orientation error
            double xy_goal_tolerance;  //!< Allowed final euclidean distance to the goal position
        } goal_tolerance;              //!< Goal tolerance related parameters


        //! Optimization related parameters
        struct Optimization
        {
            double k_rho;   //!< Proportional parameter for linear velocity adjustment based on the Euclidean distance of the robot position to the current target
            double k_alpha; //!< Proportional parameter for angular velocity adjustment based on the tangential angle of the target position in the robot's frame of reference
            double k_phi;   //!< Proportional parameter for angular velocity adjustment based on the difference between the robot's orientation(yaw) and the current target orientation(yaw)
        } optimization;

        GeoLocalConfig()
        {
            odom_topic = "odom";
            map_frame = "map";

            // Robot
            robot.max_vel_x = 0.3;
            robot.max_vel_x_backwards = 0.2;
            robot.max_vel_theta = 0.5;
            robot.acc_lim_x = 0.5;
            robot.acc_lim_theta = 0.52;
            robot.min_turn_radius = 0.5;
            robot.turn_around_priority = false;
            robot.stop_dist = 0.5;
            robot.dec_dist = 1.0;

            // GoalTolerance
            goal_tolerance.xy_goal_tolerance = 1;
            goal_tolerance.yaw_goal_tolerance = 0.1;

            // Trajectory
            trajectory.max_local_plan_lookahead_dist = 5.0;
            trajectory.local_plan_viapoint_sep = 0.5;
            trajectory.local_plan_goal_sep = 0.5;
            trajectory.local_plan_prune_distance = 1.0;

            
            // Optimization
            optimization.k_rho = 1.0;
            optimization.k_alpha = -3.0;
            optimization.k_phi = -1.0;

        }

        void loadRosParamFromNodeHandle(const ros::NodeHandle &nh);
        void reconfigure(GeoLocalPlannerReconfigureConfig &cfg);
        /**
         * @brief Return the internal config mutex
         */      
        boost::mutex& configMutex() {return config_mutex_;}

    private:
        boost::mutex config_mutex_; //!< Mutex for config accesses and changes

};
};

#endif