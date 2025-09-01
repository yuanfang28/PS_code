#include <geo_local_planner/visualization.h>

namespace geo_local_planner
{
    PlannerVisualization::PlannerVisualization() : initialized_(false)
    {
    }

    PlannerVisualization::PlannerVisualization(ros::NodeHandle &nh, std::string frame_id) : initialized_(false), frame_id_(frame_id)
    {
        initialize(nh);
    }
    void PlannerVisualization::initialize(ros::NodeHandle &nh)
    {
        if (initialized_)
            ROS_WARN("PlannerVisualization already initialized. Reinitalizing...");

        via_point_pub = nh.advertise<visualization_msgs::Marker>("via_point", 100);
        local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
        global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
        bl_plan_pub_ = nh.advertise<nav_msgs::Path>("sidewalk_left_transform", 1);
        br_plan_pub_ = nh.advertise<nav_msgs::Path>("sidewalk_right_transform", 1);
        initialized_ = true;
    }
    void PlannerVisualization::publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                                               const std::string &ns, const std_msgs::ColorRGBA &color) const
    {
        if (via_points.empty() || printErrorWhenNotInitialized())
            return;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(2.0);

        for (std::size_t i = 0; i < via_points.size(); ++i)
        {
            geometry_msgs::Point point;
            point.x = via_points[i][0];
            point.y = via_points[i][1];
            point.z = 0;
            marker.points.push_back(point);
        }

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color = color;

        via_point_pub.publish(marker);
    }

    void PlannerVisualization::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &local_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(local_plan, local_plan_pub_);
    }

    void PlannerVisualization::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(global_plan, global_plan_pub_);
    }

    void PlannerVisualization::publish_left_side_Plan(const std::vector<geometry_msgs::PoseStamped> &leftsidewalk_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(leftsidewalk_plan, bl_plan_pub_);
    }
    
    void PlannerVisualization::publish_right_side_Plan(const std::vector<geometry_msgs::PoseStamped> &rightsidewalk_plan) const
    {
        if (printErrorWhenNotInitialized())
            return;
        base_local_planner::publishPlan(rightsidewalk_plan, br_plan_pub_);
    }  

    bool PlannerVisualization::printErrorWhenNotInitialized() const
    {
        if (!initialized_)
        {
            ROS_ERROR("PlannerVisualization class not initialized. You must call initialize or an appropriate constructor");
            return true;
        }
        return false;
    }

    std_msgs::ColorRGBA PlannerVisualization::toColorMsg(double a, double r, double g, double b)
    {
        std_msgs::ColorRGBA color;
        color.a = a;
        color.r = r;
        color.g = g;
        color.b = b;
        return color;
    }

}