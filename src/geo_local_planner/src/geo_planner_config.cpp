#include <geo_local_planner/geo_planner_config.h>

namespace geo_local_planner
{
    void GeoLocalConfig::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
    {
        nh.param("odom_topic", odom_topic, odom_topic); //在参数服务器里寻找‘’，然后如果有，复赋值给第二个变量，如果没有，创建，并以第三个变量赋值      
        // Robot
        nh.param("max_vel_x", robot.max_vel_x, robot.max_vel_x);
        nh.param("max_vel_x_backwards", robot.max_vel_x_backwards, robot.max_vel_x_backwards);
        nh.param("max_vel_theta", robot.max_vel_theta, robot.max_vel_theta);
        nh.param("acc_lim_x", robot.acc_lim_x, robot.acc_lim_x);
        nh.param("acc_lim_theta", robot.acc_lim_theta, robot.acc_lim_theta);
        nh.param("min_turn_radius", robot.min_turn_radius, robot.min_turn_radius);
        nh.param("turn_around_priority", robot.turn_around_priority, robot.turn_around_priority);
        nh.param("stop_dist", robot.stop_dist, robot.stop_dist);
        nh.param("dec_dist", robot.dec_dist, robot.dec_dist);

        // GoalTolerance
        nh.param("xy_goal_tolerance", goal_tolerance.xy_goal_tolerance, goal_tolerance.xy_goal_tolerance);
        nh.param("yaw_goal_tolerance", goal_tolerance.yaw_goal_tolerance, goal_tolerance.yaw_goal_tolerance);

        // Trajectory
        nh.param("max_local_plan_lookahead_dist", trajectory.max_local_plan_lookahead_dist, trajectory.max_local_plan_lookahead_dist);
        nh.param("local_plan_viapoint_sep", trajectory.local_plan_viapoint_sep, trajectory.local_plan_viapoint_sep);
        nh.param("local_plan_goal_sep", trajectory.local_plan_goal_sep, trajectory.local_plan_goal_sep);
        nh.param("local_plan_prune_distance", trajectory.local_plan_prune_distance, trajectory.local_plan_prune_distance);
        
        // Optimization
        nh.param("k_rho", optimization.k_rho, optimization.k_rho);
        nh.param("k_alpha", optimization.k_alpha, optimization.k_alpha);
        nh.param("k_phi", optimization.k_phi, optimization.k_phi);
    }

    void GeoLocalConfig::reconfigure(GeoLocalPlannerReconfigureConfig &cfg)
    {
        boost::mutex::scoped_lock l(config_mutex_);
        // Robot
        robot.max_vel_x = cfg.max_vel_x;
        robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
        robot.max_vel_theta = cfg.max_vel_theta;
        robot.acc_lim_x = cfg.acc_lim_x;
        robot.acc_lim_theta = cfg.acc_lim_theta;
        robot.min_turn_radius = cfg.min_turn_radius;
        robot.turn_around_priority = cfg.turn_around_priority;
        robot.stop_dist = cfg.stop_dist;
        robot.dec_dist = cfg.dec_dist;

        // GoalTolerance
        goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
        goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;

        // Trajectory
        trajectory.max_local_plan_lookahead_dist = cfg.max_local_plan_lookahead_dist;
        trajectory.local_plan_viapoint_sep = cfg.local_plan_viapoint_sep;
        trajectory.local_plan_goal_sep = cfg.local_plan_goal_sep;
        trajectory.local_plan_prune_distance = cfg.local_plan_prune_distance;

        // Optimization
        optimization.k_rho = cfg.k_rho;
        optimization.k_alpha = cfg.k_alpha;
        optimization.k_phi = cfg.k_phi;
    }

}