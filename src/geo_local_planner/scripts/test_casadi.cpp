#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geo_local_planner/geo_planner.h>  // 包含 geo_planner 头文件

int main(int argc, char** argv)
{
    ros::init(argc, argv, "geo_planner_test_node");
    ros::NodeHandle nh;

    // 初始化 geo_planner
    geo_local_planner::geo_planner planner;
    planner.config_geo_planner();

    // 创建测试输入数据
    std::vector<geometry_msgs::PoseStamped> robot_plan;
    for (int i = 0; i < 10; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        robot_plan.push_back(pose);
    }

    std::vector<geometry_msgs::PoseStamped> leftSidewalk_plan;
    for (int i = 0; i < 10; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 3;
        leftSidewalk_plan.push_back(pose);
    }

    std::vector<geometry_msgs::PoseStamped> rightSidwalk_plan;
    for (int i = 0; i < 10; ++i) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = -3;
        rightSidwalk_plan.push_back(pose);
    }
    planner.caculate_geo_path(robot_plan, leftSidewalk_plan, rightSidwalk_plan);







    // 打印优化结果
// For u_opt
ROS_INFO("Optimization Result u_opt: ");
for (size_t i = 0; i < planner.u_opt.size(); ++i) {
    ROS_INFO("%f", planner.u_opt[i]);
}

// For rt_coeff_val
ROS_INFO("rt_coeff_val: ");
for (size_t i = 0; i < planner.rt_coeff_val.size(); ++i) {
    ROS_INFO("%f", planner.rt_coeff_val[i]);
}
ROS_INFO("lt_coeff_val: ");
for (size_t i = 0; i < planner.lt_coeff_val.size(); ++i) {
    ROS_INFO("%f", planner.lt_coeff_val[i]);
}

// For global_coeff_val
ROS_INFO("global_coeff_val: ");
for (size_t i = 0; i < planner.global_coeff_val.size(); ++i) {
    ROS_INFO("%f", planner.global_coeff_val[i]);
}

// For obs_val
ROS_INFO("obs: ");
for (size_t i = 0; i < planner.obs_val.size(); ++i) {
    ROS_INFO("%f", planner.obs_val[i]);
}


    
    return 0;
}
