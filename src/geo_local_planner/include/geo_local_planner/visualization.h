
#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include "Eigen/Core"
#include <string>
// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <nav_msgs/Path.h>
#include <base_local_planner/goal_functions.h>

namespace geo_local_planner
{
    class PlannerVisualization
    {
    public:
        PlannerVisualization();
        PlannerVisualization(ros::NodeHandle &nh, std::string frame_id);
        void initialize(ros::NodeHandle &nh);
        void publishViaPoints(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &via_points,
                              const std::string &ns = "ViaPoints", const std_msgs::ColorRGBA &color = toColorMsg(1.0, 0.0, 0.0, 1.0)) const;
        void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &local_plan) const;
        void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan) const;
        void publish_left_side_Plan(const std::vector<geometry_msgs::PoseStamped> &leftsidewalk_plan) const;
        void publish_right_side_Plan(const std::vector<geometry_msgs::PoseStamped> &rightsidewalk_plan) const;
        


        /**
         * @brief Helper function to generate a color message from single values
         * @param a Alpha value
         * @param r Red value
         * @param g Green value
         * @param b Blue value
         * @return Color message
         */
        static std_msgs::ColorRGBA toColorMsg(double a, double r, double g, double b);

    protected:
        bool printErrorWhenNotInitialized() const;
        ros::Publisher via_point_pub; //!< Publisher for visualization markers
        ros::Publisher local_plan_pub_;    //!< Publisher for the local plan
        ros::Publisher global_plan_pub_;
        ros::Publisher bl_plan_pub_;    //!< Publisher for the local plan
        ros::Publisher br_plan_pub_;

        bool initialized_;
        std::string frame_id_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    //! Abbrev. for shared instances of the PlannerVisualization
    typedef boost::shared_ptr<PlannerVisualization> PlannerVisualizationPtr;
}
#endif /* VISUALIZATION_H_ */

