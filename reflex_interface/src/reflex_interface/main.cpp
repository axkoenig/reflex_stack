
#include <ros/ros.h>

#include "reflex_interface/reflex_interface.hpp"

std::string node_name = "reflex_interface_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ROS_INFO("Launched %s.", node_name.c_str());

    ros::NodeHandle nh;
    bool use_sim_data_hand = true; 
    bool use_sim_data_obj = true; 
    ReflexInterface ri = ReflexInterface(&nh, use_sim_data_hand, use_sim_data_obj);
    ros::spin();
}