#include <pluginlib/class_loader.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "plugin_loader_test");

    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> loader("nav_core", "nav_core::BaseLocalPlanner");

    try {
        boost::shared_ptr<nav_core::BaseLocalPlanner> plugin = loader.createInstance("geo_local_planner/GeoPlannerROS");
        ROS_INFO("Plugin loaded successfully!");
    } catch (pluginlib::PluginlibException& ex) {
        ROS_ERROR("Failed to load the plugin. Error: %s", ex.what());
    }

    return 0;
}
