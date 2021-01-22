#ifndef INTERFACE_H
#define INTERFACE_H

#include "hand_command.hpp"
#include "hand_state.hpp"

class ReflexInterface
{
private:
    ros::NodeHandle nh;

public:
    HandState state = HandState(&nh);
    HandCommand command = HandCommand(&nh);
};
#endif