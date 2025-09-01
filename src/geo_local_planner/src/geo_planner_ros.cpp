#include <geo_local_planner/geo_planner_ros.h>

#include "Eigen/Core"

#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.hpp>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_core/parameter_magic.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>

PLUGINLIB_EXPORT_CLASS(geo_local_planner::GeoPlannerROS, nav_core::BaseLocalPlanner)

namespace geo_local_planner
{
    GeoPlannerROS::GeoPlannerROS() : _costmap_ros(NULL), tf_(NULL), initialized_(false){}

    GeoPlannerROS::GeoPlannerROS(std::string name, tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmap_ros)
        : dynamic_recfg_(NULL), _costmap_ros(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    GeoPlannerROS::~GeoPlannerROS() {}

    void GeoPlannerROS::reconfigureCB(GeoLocalPlannerReconfigureConfig &config, uint32_t level)
    {
        cfg_.reconfigure(config);
        
    }
  
    void GeoPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf,
                                        costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {      
            //create the planner instance
            _geo_planner.config_geo_planner();
            
            //load params
            ros::NodeHandle nh("~/" + name);
            cfg_.loadRosParamFromNodeHandle(nh);
            
            //frame
            tf_ = tf;
            _costmap_ros = costmap_ros;
            _costmap = _costmap_ros->getCostmap();//调用 getCostmap() 函数，并将返回值赋值给 _costmap
            global_frame_ = _costmap_ros->getGlobalFrameID();
            robot_base_frame_ = _costmap_ros->getBaseFrameID();

            //footprint
            _footprint_spec = _costmap_ros->getRobotFootprint();


            //visualisation
            visualization_ = PlannerVisualizationPtr(new PlannerVisualization(nh, global_frame_));
            visualization_local = PlannerVisualizationPtr(new PlannerVisualization(nh, robot_base_frame_));
            pose_pub = nh.advertise<geometry_msgs::Point>("robot_pose", 100);
            obs_center_pub = nh.advertise<geometry_msgs::PoseArray>("obs_center", 10);
            obs_radius_pub = nh.advertise<std_msgs::Float64MultiArray>("obs_radius", 10);


            //subcirber
            //left_side_sub = nh.subscribe<sensor_msgs::PointCloud2>("/left_point_cloud",1,&GeoPlannerROS::leftSidewalk_pts_callback, this);
            //right_side_sub = nh.subscribe<sensor_msgs::PointCloud2>("/right_point_cloud",1,&GeoPlannerROS::rightSidewalk_pts_callback, this);      
            obstacles_sub_ = nh.subscribe("/raw_obstacles", 3, &GeoPlannerROS::obstacles_callback, this);


            // setup dynamic reconfigure
            dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<GeoLocalPlannerReconfigureConfig> >(nh);
            dynamic_reconfigure::Server<GeoLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&GeoPlannerROS::reconfigureCB, this, _1, _2);
            dynamic_recfg_->setCallback(cb);

            // init the odom helper to receive the robot's velocity from odom messages
            odom_helper_.setOdomTopic(cfg_.odom_topic);


            ros::NodeHandle nh_move_base("~");
            double controller_frequency = 5;
            nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
            control_duration_ = 1.0 / controller_frequency;

            //flag init
            last_back_ = false;
            reached_goal_ = false; 
            initialized_ = true;

            robot_position_all.clear();

        }
    }


    bool GeoPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

    // To-do
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        //visualization_->publishGlobalPlan(global_plan_);

        reached_goal_ = false;

        return true;

    }
    bool GeoPlannerROS::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if (reached_goal_)
        {
            ROS_INFO("GOAL Reached!");
            return true;
        }
        return false;
    }


    bool GeoPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // check if plugin initialized
        if(!initialized_)
        {
            ROS_ERROR("geo_local_planner has not been initialized, please call initialize() before using this planner");
            return mbf_msgs::ExePathResult::NOT_INITIALIZED;
        }


        // Get robot pose
        if (!_costmap_ros->getRobotPose(robot_pose_)) 
        {
            return false;
        }
        geometry_msgs::TransformStamped odom_to_global_transform = tf_->lookupTransform(
            global_plan_[0].header.frame_id,  // 目标坐标系
            ros::Time(),
            robot_pose_.header.frame_id,  // 源坐标系
            robot_pose_.header.stamp,  // 时间戳
            robot_pose_.header.frame_id,
            ros::Duration(0.5)  // 缓冲时间
        );
        geometry_msgs::PoseStamped robot_pose_map;
        tf2::doTransform(robot_pose_, robot_pose_map, odom_to_global_transform);
        robot_position.x = robot_pose_map.pose.position.x;
        robot_position.y = robot_pose_map.pose.position.y;
        pose_pub.publish(robot_position);
        

        // Get robot velocity (in robot base frame)
        geometry_msgs::PoseStamped robot_vel_tf;
        odom_helper_.getRobotVel(robot_vel_tf);
        robot_vel_.linear.x = robot_vel_tf.pose.position.x;
        robot_vel_.linear.y = robot_vel_tf.pose.position.y;
        robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

        // prune global plan to remove parts of the past. robot_pose->odom_frame, global_plan_->map_frame

        pruneGlobalPlan(*tf_, robot_pose_, global_plan_, cfg_.trajectory.local_plan_prune_distance);

        //visualization_->publishGlobalPlan(global_plan_);

        // Transform global plan to the frame of interest (w.r.t. the local costmap)
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        std::vector<geometry_msgs::PoseStamped> robot_plan;
        int goal_idx;
        double path_length;
        geometry_msgs::TransformStamped tf_plan_to_base;
        //                              map_frame    odom_frame             odom_frame     base_link
        /*if (!transformGlobalPlan(*tf_, global_plan_, robot_pose_, *_costmap, global_frame_, robot_base_frame_, cfg_.trajectory.max_local_plan_lookahead_dist, 
                                transformed_plan, robot_plan, &goal_idx, &path_length, &tf_plan_to_base))*/

        if (!transformGlobalPlan(*tf_, global_plan_, robot_pose_, *_costmap, global_frame_, robot_base_frame_, cfg_.trajectory.max_local_plan_lookahead_dist, 
                                transformed_plan, robot_plan, leftsidewalk_plan_, rightsidewalk_plan_, &goal_idx, &path_length, &tf_plan_to_base))                       
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return mbf_msgs::ExePathResult::INTERNAL_ERROR;
        }

        // return false if the transformed global plan is empty
        if (transformed_plan.empty())
        {
            ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
            return mbf_msgs::ExePathResult::INVALID_PATH;
        }

        // check if global is reached
        geometry_msgs::PoseStamped base_goal;
        tf2::doTransform(global_plan_.back(), base_goal, tf_plan_to_base);
        bg::model::d2::point_xy<double> p1(base_goal.pose.position.x,base_goal.pose.position.y), p2(0,0);
        double goal_dist = bg::distance(p1,p2);
        if(goal_dist < cfg_.goal_tolerance.xy_goal_tolerance)
        {
            ROS_INFO("distance to goal %f", goal_dist );
            reached_goal_ = true;
            //aver_time = calc_time / iter;
            //ROS_INFO("nmpc average calculation time %.4f", aver_time);
            return mbf_msgs::ExePathResult::SUCCESS;
        }
        if (goal_dist < cfg_.goal_tolerance.xy_goal_tolerance + 1.0)
            path_length *= 0.7;


        // check if obstacle intersected with footprint on local reference path
        _geo_planner.obstaclePresent = _geo_planner.checkObstacleFootprintIntersection(transformed_plan, _footprint_spec, seg_obs_, circ_obs_);


        // ref_path_theta
        double ref_path_theta;
        ref_path_theta = estimateRefPathOrientation(transformed_plan,10);

        std::vector<geometry_msgs::PoseStamped> geo_path_plan;
        std::vector<geometry_msgs::PoseStamped> geo_path;
        std::vector<geometry_msgs::PoseStamped> polyfit_path;

        visualization_local->publish_left_side_Plan(leftsidewalk_plan_);
        visualization_local->publish_right_side_Plan(rightsidewalk_plan_);


        if (ref_path_theta < M_PI/2  && ref_path_theta > -M_PI/2) 
        {
            //if (transformed_plan.size()<5){}
            //else{
            //    _geo_planner.caculate_geo_path(transformed_plan, leftsidewalk_plan_, rightsidewalk_plan_);
            //}
            //ros::Time begin2 = ros::Time::now();
            //visualization_local->publishGlobalPlan(transformed_plan);
            _geo_planner.caculate_geo_path(transformed_plan, leftsidewalk_plan_, rightsidewalk_plan_);
            //_geo_planner.caculate_geo_path(global_plan_, leftsidewalk_plan_, rightsidewalk_plan_);
            
            //_geo_planner.geo_path_visualize(geo_path);
            _geo_planner.geo_path_visualize(geo_path, polyfit_path);
            visualization_local->publishGlobalPlan(polyfit_path);
            
           
            updateViaPointsContainer(geo_path,
                                     cfg_.trajectory.local_plan_viapoint_sep, 
                                     cfg_.trajectory.local_plan_goal_sep); 
            
            const double goal_x = base_goal.pose.position.x;
            const double goal_y = base_goal.pose.position.y;
            const double goal_th = tf2::getYaw(base_goal.pose.orientation); 
           
            via_points_.push_back(Eigen::Vector3d(goal_x, goal_y, goal_th)); //std::pusch_back 用于向容器的末尾添加一个新元素, 再加入终点坐标
            //local_plan_.push_back(global_plan_.back()); 

            //cauculate v and omega        
            double dx1 = via_points_[0][0];
            double dy1 = via_points_[0][1];
            double dyaw1 = via_points_[0][2];
            double rho, alpha, phi, v, omega;
            poseError(dx1, dy1, dyaw1, rho, alpha, phi);
            puresuitControl(rho, alpha, phi, v, omega);




            const double min_feasible_angular_speed = robot_vel_.angular.z - cfg_.robot.acc_lim_theta * control_duration_;
            const double max_feasible_angular_speed = robot_vel_.angular.z + cfg_.robot.acc_lim_theta * control_duration_;
            omega = clip(omega, min_feasible_angular_speed, max_feasible_angular_speed);

            const double min_feasible_linear_speed = robot_vel_.linear.x - cfg_.robot.acc_lim_x * control_duration_;
            const double max_feasible_linear_speed = robot_vel_.linear.x + cfg_.robot.acc_lim_x * control_duration_;
            v = clip(v, min_feasible_linear_speed, max_feasible_linear_speed);

            omega = 2*v / (pow(dx1,2)+pow(dy1,2)) * dy1;
            //v = 0;
            //omega = 0;

            cmd_vel.linear.x = v;
            cmd_vel.angular.z = omega;

            //ros::Duration elapsed_time2 = ros::Time::now() - begin2;
            //calc_time += elapsed_time2.toSec();
            //iter++;
        }
        else
        {
            ROS_INFO("Large angle error, rotate to find forwarding path");
            cmd_vel.linear.x  = 0;
            if (ref_path_theta > 0.0)
                cmd_vel.angular.z = 2.0;
            else
                cmd_vel.angular.z = -2.0;
        }

        // get obstacles (base-map-transform)
        geometry_msgs::TransformStamped base_to_global_transform = tf_->lookupTransform(
            global_plan_[0].header.frame_id,  // 目标坐标系
            ros::Time(),
            geo_path[0].header.frame_id,      // 源坐标系
            geo_path[0].header.stamp,         // 时间戳
            geo_path[0].header.frame_id,
            ros::Duration(0.5)                // 缓冲时间
        );
        // 定义变量
        geometry_msgs::Point center_transform;
        geometry_msgs::Point center;
        double radius;
        geometry_msgs::PoseArray center_all; 
        std_msgs::Float64MultiArray radius_msg;  // 新建消息


        center_all.header.frame_id = global_plan_[0].header.frame_id;
        center_all.header.stamp = ros::Time::now();
        for (size_t i = 0; i < circ_obs_.size(); ++i) {
            center.x = boost::geometry::get<0>(circ_obs_[i].center);
            center.y = boost::geometry::get<1>(circ_obs_[i].center);
            center.z = 0.0;  

            // 使用 tf2::doTransform 将圆心转换到目标坐标系
            tf2::doTransform(center, center_transform, base_to_global_transform);

            // 提取当前障碍物的半径
            radius = circ_obs_[i].radius;

            // 存储转换后的中心点和半径
            geometry_msgs::Pose pose;
            pose.position = center_transform;  // 设置位置
            pose.orientation.w = 1.0;          // 单位四元数（方向无关）
            center_all.poses.push_back(pose);
            radius_msg.data.push_back(radius);
        }

        obs_center_pub.publish(center_all);
        obs_radius_pub.publish(radius_msg);

        
        //visualisation
        //base_local_planner::publishPlan(geo_path, geo_path_pub);
        visualization_local->publishLocalPlan(geo_path);
        visualization_local->publishViaPoints(via_points_);
        
        //visualization_->publishLocalPlan(local_plan_);
        return true;
    }



    void GeoPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &geo_plan, double min_separation_via,double min_separation_goal)
    {
        via_points_.clear();
        local_plan_.clear();

        //存储机器人当前的点
        /*geometry_msgs::PoseStamped pose;
        pose.header.frame_id = robot_base_frame_;
        pose.header.stamp = ros::Time::now();
        pose.pose = robot_pose.pose;
        local_plan_.push_back(pose);*/

        if (min_separation_via <= 0)
            return;

        std::size_t prev_idx = 0; //std::size_t：是一个无符号整数类型，通常用于表示对象的大小、数组索引、容器大小等
        double via_point_yaw;
        for (std::size_t i = 1; i < geo_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
        {
            bg::model::d2::point_xy<double> p1(geo_plan[i].pose.position.x, geo_plan[i].pose.position.y), p2(0,0);
            double robot_target_dist = bg::distance(p1,p2);
            // check separation to the previous via-point inserted
            if ((distancePoints2d(geo_plan[prev_idx].pose.position, geo_plan[i].pose.position) < min_separation_via) ||
                (distancePoints2d(geo_plan.back().pose.position, geo_plan[i].pose.position) < min_separation_goal) ||
                (robot_target_dist < min_separation_via))
                continue; //满足其中一条就跳过当前循环


            // add via-point
            via_point_yaw = std::atan2(geo_plan[i].pose.position.y - geo_plan[prev_idx].pose.position.y,
                                       geo_plan[i].pose.position.x - geo_plan[prev_idx].pose.position.x);
            via_points_.push_back(Eigen::Vector3d(geo_plan[i].pose.position.x, geo_plan[i].pose.position.y, via_point_yaw));
            //geometry_msgs::PoseStamped local_plan_pose = geo_plan[i];
            //local_plan_pose.pose.orientation = tf::createQuaternionMsgFromYaw(via_point_yaw);
            //local_plan_.push_back(local_plan_pose);
            prev_idx = i;
        }
    }

    void GeoPlannerROS::puresuitControl(double rho, double alpha, double phi, double &v, double &omega)
    {
        v = cfg_.optimization.k_rho * rho;
        /*if ((fabs(alpha) > M_PI * 0.5 + 0.1) or ((fabs(alpha) > M_PI * 0.5 - 0.1) and last_back_))
        {
            v = v * (-1.0);
            alpha = angDiff(alpha, M_PI);
            last_back_ = true;
        }
        else
            last_back_ = false;*/
        //v = clip(v, cfg_.robot.max_vel_x * (-1.0) * dec_ratio_, cfg_.robot.max_vel_x * dec_ratio_);

        omega = cfg_.optimization.k_alpha * alpha + cfg_.optimization.k_phi * phi;
        
        //omega = clip(omega, cfg_.robot.max_vel_theta * (-1.0) * dec_ratio_, cfg_.robot.max_vel_theta * dec_ratio_);
    }


    void GeoPlannerROS::cart2Pol(double x, double y, double &deg, double &dist)
    {
        dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        deg = std::atan2(y, x);
    }

    double GeoPlannerROS::angDiff(double alpha, double beta)
    {
        double delta;
        delta = fmod((alpha - beta), 2.0 * M_PI);
        if (delta > M_PI)
            delta = delta - 2 * M_PI;
        else if (delta < M_PI * (-1.0))
            delta = delta + 2 * M_PI;
        return delta;
    }

    void GeoPlannerROS::poseError(double dx, double dy, double dyaw, double &rho, double &alpha, double &phi)
    {
        cart2Pol(dx, dy, alpha, rho);
        alpha = angDiff(0, alpha);
        phi = dyaw;
    }

    double GeoPlannerROS::clip(double value, double lower, double upper)
    {
        if (value < lower)
            return lower;
        else if (value > upper)
            return upper;
        else
            return value;
    }



    void GeoPlannerROS::obstacles_callback(const obstacle_detector::Obstacles::ConstPtr new_obstacles)
    {
        // register circle obstacles
        circ_obs_.clear();
        for (auto &circ : new_obstacles->circles)
        {
            circle new_circ_obs;
            b_point center(circ.center.x,circ.center.y);
            new_circ_obs.center = center;
            new_circ_obs.radius = (circ.radius + circ.true_radius)/2;


            bg::strategy::buffer::distance_symmetric<double> distance_strategy(new_circ_obs.radius);
            bg::strategy::buffer::end_round end_strategy;
            bg::strategy::buffer::join_round join_strategy;
            bg::strategy::buffer::point_circle point_strategy(60);
            bg::strategy::buffer::side_straight side_strategy;
            bg::buffer(new_circ_obs.center, new_circ_obs.circ, distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);    
            circ_obs_.push_back(new_circ_obs);

            //ROS_INFO("num of circles: %ld", new_obstacles->circles.size());
        }
        // obstacles_.circles.clear();
        // obstacles_.circles.assign(new_obstacles->circles.begin(), new_obstacles->circles.end());
        
        // register segment obstacles
        seg_obs_.clear();
        for (auto &seg : new_obstacles->segments)
        {
            b_segment new_seg(b_point(seg.first_point.x, seg.first_point.y), b_point(seg.last_point.x, seg.last_point.y));
            seg_obs_.push_back(new_seg);
            //ROS_INFO("num of segments: %ld", new_obstacles->segments.size());
        }
        // obstacles_.segments.clear();
        // obstacles_.segments.assign(new_obstacles->segments.begin(), new_obstacles->segments.end());

    }


    bool GeoPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                    std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
    {
        if (global_plan.empty())
            return true;
        
        try
        {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform); // global_to_plan_transform->transform odom to map
            
            double dist_thresh_sq = dist_behind_robot*dist_behind_robot; 
            
            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end())
            {
            double dx = robot.pose.position.x - it->pose.position.x;
            double dy = robot.pose.position.y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < dist_thresh_sq)
            {
                erase_end = it;
                break;
            }
            ++it;
            }
            if (erase_end == global_plan.end())
                return false;
            
            if (erase_end != global_plan.begin())
            global_plan.erase(global_plan.begin(), erase_end);
        }
        catch (const tf::TransformException& ex)
        {
            // ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }
        return true;
    }
    
    /*bool GeoPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                    std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot,  double dist_ahead_robot)
    {
        if (global_plan.empty())
            return true;
        
        try
        {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform); // global_to_plan_transform->transform odom to map
            
            double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
            double dist_ahead_thresh_sq = dist_ahead_robot * dist_ahead_robot;  
            
            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end())
            {
                double dx = robot.pose.position.x - it->pose.position.x;
                double dy = robot.pose.position.y - it->pose.position.y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq < dist_thresh_sq)
                {
                    erase_end = it;
                    break;
                }
                ++it;
            }
            if (erase_end == global_plan.end())
                return false;
            
            if (erase_end != global_plan.begin())
            global_plan.erase(global_plan.begin(), erase_end);


            // Step 2: Reverse prune - Remove points ahead of the robot beyond `dist_ahead_robot`
            auto rit = global_plan.rbegin();
            auto erase_start = rit;
            while (rit != global_plan.rend())
            {
                double dx = robot.pose.position.x - rit->pose.position.x;
                double dy = robot.pose.position.y - rit->pose.position.y;
                double dist_sq = dx * dx + dy * dy;
                if (dist_sq <= dist_ahead_robot)
                {
                    erase_start = rit;
                    break;
                }
                ++rit;
            }
            // Convert reverse iterator to normal iterator for erase
            if (erase_start == global_plan.rend())
                return false;
            if (erase_start != global_plan.rbegin())
                global_plan.erase(erase_start.base(), global_plan.end());
        }
        catch (const tf::TransformException& ex)
        {
            // ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }
        return true;
    }*/

    /*bool GeoPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap,
                                                const std::string& global_frame, const std::string& robot_base_frame, double max_plan_length,
                                                std::vector<geometry_msgs::PoseStamped>& transformed_plan, 
                                                std::vector<geometry_msgs::PoseStamped>& robot_plan,
                                                int* current_goal_idx, double* plan_length,
                                                // geometry_msgs::TransformStamped* tf_plan_to_global,
                                                geometry_msgs::TransformStamped* tf_plan_to_base) const
    {
        // this method is a slightly modified version of base_local_planner/goal_functions.h
        const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

        transformed_plan.clear();
        // robot_plan.clear();
        try
        {
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            // target_frame, target_time, source_frame, source_time, fixed_frame, ros::Duration timeout) const
            geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(
                global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // target_frame, source_frame, time
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                // robot_base_frame, global_frame, ros::Time(0));
            geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(0.5));

            // get plan_to_global_transform from plan frame to base_link_frame
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
            //     robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));


            // we'll discard points on the plan that are outside the local costmap
            double dist_threshold =
                std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85;  // just consider 85% of the costmap size to better incorporate point obstacle that are
                                    // located on the border of the local costmap
            int i                    = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist           = 1e10;

            // we need to loop to a point on the plan that is within a certain distance of the robot

            geometry_msgs::PoseStamped newer_pose;

            *plan_length = 0;  // check cumulative Euclidean distance along the plan

            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || *plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped& pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist       = x_diff * x_diff + y_diff * y_diff;

                // caclulate distance to previous pose 
                if (i > 0 && max_plan_length > 0)
                    *plan_length += std::sqrt( std::pow(global_plan[i].pose.position.x-global_plan[i - 1].pose.position.x,2) + std::pow(global_plan[i].pose.position.y-global_plan[i - 1].pose.position.y,2) );
                    // teb_local_planner::distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

                ++i;
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = i - 1;  // subtract 1, since i was increased once before leaving the loop
            }

            // int k = 0;
            // // transform to robot frame
            // while (k < (int)transformed_plan.size())
            // {
            //     const geometry_msgs::PoseStamped& pose = transformed_plan[k];
            //     tf2::doTransform(pose, newer_pose, plan_to_robot_transform);

            //     robot_plan.push_back(newer_pose);

            //     ++k;
            // }            

            // Return the transformation from the global plan to the global planning frame if desired
            // if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
            if (tf_plan_to_base) *tf_plan_to_base = plan_to_robot_transform;
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(),
                        global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }*/

    bool GeoPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap,
                                                const std::string& global_frame, const std::string& robot_base_frame, double max_plan_length,
                                                std::vector<geometry_msgs::PoseStamped>& transformed_plan, 
                                                std::vector<geometry_msgs::PoseStamped>& robot_plan,
                                                std::vector<geometry_msgs::PoseStamped>& leftsidewalk_plan_,
                                                std::vector<geometry_msgs::PoseStamped>& rightsidewalk_plan_,
                                                int* current_goal_idx, double* plan_length,
                                                // geometry_msgs::TransformStamped* tf_plan_to_global,
                                                geometry_msgs::TransformStamped* tf_plan_to_base) const
    {
        // this method is a slightly modified version of base_local_planner/goal_functions.h
        const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

        transformed_plan.clear();
        leftsidewalk_plan_.clear();
        rightsidewalk_plan_.clear();
        // robot_plan.clear();
        try
        {
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            // target_frame, target_time, source_frame, source_time, fixed_frame, ros::Duration timeout) const
            geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(
                global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // target_frame, source_frame, time
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                // robot_base_frame, global_frame, ros::Time(0));
            geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(0.5));

            // get plan_to_global_transform from plan frame to base_link_frame
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
            //     robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));


            // we'll discard points on the plan that are outside the local costmap
            /*double dist_threshold =
                std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85;  // just consider 85% of the costmap size to better incorporate point obstacle that are
                                    // located on the border of the local costmap
            int i                    = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist           = 1e10;*/

            // we need to loop to a point on the plan that is within a certain distance of the robot

            geometry_msgs::PoseStamped newer_pose;

            *plan_length = 0;  // check cumulative Euclidean distance along the plan
            int i = 0;
            double sq_dist;
            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() /*&& sq_dist <= sq_dist_threshold*/ && (max_plan_length <= 0 || *plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped& pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist       = x_diff * x_diff + y_diff * y_diff;

                // caclulate distance to previous pose 
                if (i > 0 && max_plan_length > 0)
                    *plan_length += std::sqrt( std::pow(global_plan[i].pose.position.x-global_plan[i - 1].pose.position.x,2) + std::pow(global_plan[i].pose.position.y-global_plan[i - 1].pose.position.y,2) );
                    // teb_local_planner::distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

                ++i;
            }
                    // side
            geometry_msgs::PoseStamped left_plan_;
            geometry_msgs::PoseStamped right_plan_;

            for (double x = -20.0; x <= 50; x += 1)
            {
            ros::Time current_time = ros::Time::now(); 
            geometry_msgs::PoseStamped pose_left;
            geometry_msgs::PoseStamped pose_right;
            pose_left.header.stamp = current_time;   
            pose_left.header.frame_id = "map"; 
            pose_left.pose.position.x = x; 
            pose_left.pose.position.y = 3; 
            pose_left.pose.position.z = 0.0; 
            pose_left.pose.orientation.w = 1.0; 

            tf2::doTransform(pose_left, left_plan_, plan_to_robot_transform);
            leftsidewalk_plan_.push_back(left_plan_);
        

            pose_right.header.stamp = current_time;  
            pose_right.header.frame_id = "map"; 
            pose_right.pose.position.x = x; 
            pose_right.pose.position.y = -3; 
            pose_right.pose.position.z = 0.0; 
            pose_right.pose.orientation.w = 1.0;

            tf2::doTransform(pose_right, right_plan_, plan_to_robot_transform);
            rightsidewalk_plan_.push_back(right_plan_);
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = i - 1;  // subtract 1, since i was increased once before leaving the loop
            }

            if (tf_plan_to_base) *tf_plan_to_base = plan_to_robot_transform;
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(),
                        global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }

    /*void GeoPlannerROS::leftSidewalk_pts_callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        sensor_msgs::PointCloud2 cloud_out;
        camera_frame_ = cloud->header.frame_id.c_str();
        if(transformPCL(*tf_, camera_frame_, robot_base_frame_, cloud, cloud_out))
        {
            pcl::fromROSMsg(cloud_out, *temp_cloud);
        }
            
        //do stuff
        leftsidewalk_plan_.clear();
        double left_path_length = 0;
        std::reverse(temp_cloud->points.begin(),temp_cloud->points.end());
        for (int i = 0; i < temp_cloud->points.size(); ++i)
        {
            geometry_msgs::PoseStamped newer_pose;
            newer_pose.header.frame_id = robot_base_frame_;
            newer_pose.pose.position.x = temp_cloud->points[i].x;
            newer_pose.pose.position.y = temp_cloud->points[i].y;
            leftsidewalk_plan_.push_back(newer_pose);
    
            left_path_length = std::sqrt( std::pow(temp_cloud->points[i].x,2) 
            + std::pow(temp_cloud->points[i].y,2) );
            // ROS_INFO("points %d at x: %f, y: %f, l %f", (int)leftsidewalk_plan_.size(), temp_cloud->points[i].x,temp_cloud->points[i].y,left_path_length);

            if (left_path_length > 4)
                break;
            // ROS_INFO("points %d at x: %f, y: %f, z %f", (int)leftsidewalk_plan_.size(), temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z);
        }

        //visualization_->publishLocalPlan(leftsidewalk_plan_);
        
    }
    
    void GeoPlannerROS::rightSidewalk_pts_callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        sensor_msgs::PointCloud2 cloud_out;
        camera_frame_ = cloud->header.frame_id.c_str();
        if(transformPCL(*tf_, camera_frame_, robot_base_frame_, cloud, cloud_out))
        {
            pcl::fromROSMsg(cloud_out, *temp_cloud);
        }
            
        //do stuff
        rightsidewalk_plan_.clear();
        double right_path_length = 0;
        for (int i = 0; i < temp_cloud->points.size(); ++i)
        {
            geometry_msgs::PoseStamped newer_pose;
            newer_pose.header.frame_id = robot_base_frame_;
            newer_pose.pose.position.x = temp_cloud->points[i].x;
            newer_pose.pose.position.y = temp_cloud->points[i].y;
            rightsidewalk_plan_.push_back(newer_pose);
            right_path_length = std::sqrt( std::pow(temp_cloud->points[i].x,2) 
            + std::pow(temp_cloud->points[i].y,2) );
            if (right_path_length > 4)
                break;
            // ROS_INFO("points %d at x: %f, y: %f, z %f", (int)rightsidewalk_plan_.size(), temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z);
        }

        //visualization_->publishLocalPlan(rightsidewalk_plan_);
        
    }*/

    double GeoPlannerROS::estimateRefPathOrientation(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, int moving_average_length) const
    {
        int n = (int)transformed_plan.size();

        moving_average_length = std::min(moving_average_length, n-1 ); // maybe redundant, since we have checked the vicinity of the goal before
        double accum_angles = 0.0;
        if (moving_average_length == 0)
            moving_average_length = 1;
        for (int i = 0; i < moving_average_length; ++i)
        {
            // Transform pose of the global plan to the planning frame
            tf2::Quaternion q;
            tf2::convert(transformed_plan.at(i).pose.orientation,q);
            accum_angles+=tf2::getYaw(q);
            // calculate yaw angle  
            // candidates.push_back( std::atan2(transformed_plan.at(i).pose.position.y - transformed_plan.at(i).pose.position.y,
            //     transformed_plan.at(i).pose.position.x - transformed_plan.at(i).pose.position.x ) );
            
        }
        return accum_angles/double(moving_average_length);
    }




}