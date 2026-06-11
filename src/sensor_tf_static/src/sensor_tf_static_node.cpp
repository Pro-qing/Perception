#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include <sys/inotify.h>
#include <unistd.h>
#include <libgen.h>
#include <pthread.h>

// Global variables
static tf2_ros::StaticTransformBroadcaster static_broadcaster;
static std::string g_caliration_file;
static ros::NodeHandle* g_nh = nullptr;

bool loadAndPublish()
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(g_caliration_file);
    }
    catch(const YAML::Exception& e)
    {
        ROS_ERROR("Failed to load calibration file '%s': %s", g_calibration_file.c_str(), e.what());
        return false;
    }
    
    if(!config["calibration"] || !config["calibration"]["main"])
    {
        ROS_ERROR("Missing 'calibration.main' in calibration file!");
        return false;
    }

    YAML::Node main_node = config["calibration"]["main"];
    double x = main_node["x"].as<double>(0.0);
    double y = main_node["y"].as<double>(0.0);
    double z = main_node["z"].as<double>(0.0);
    double roll = main_node["roll"].as<double>(0.0);
    double pitch = main_node["pitch"].as<double>(0.0);
    double yaw = main_node["yaw"].as<double>(0.0);

    ROS_INFO("Loaded calibration: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f", 
             x, y, z, roll, pitch, yaw);

    geometry_msgs::TransformStamped static_transform;
    
}