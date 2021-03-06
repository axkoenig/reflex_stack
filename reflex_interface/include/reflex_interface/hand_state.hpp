#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <vector>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <tf2_ros/transform_broadcaster.h>
#include <reflex_interface/HandStateStamped.h>

#include "reflex_interface/finger_state.hpp"
#include "reflex_interface/motor_state.hpp"
#include "reflex_interface/hand_state_variables.hpp"
#include "reflex_interface/grasp_quality.hpp"
#include "reflex_interface/task_polytope.hpp"
#include "sensor_listener/ContactFrames.h"

class HandState
{
public:
    const int num_fingers = 3;
    const int num_motors = 4;
    const int num_sensors_per_finger = 9;
    enum ContactState
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };

    HandState(ros::NodeHandle *nh, bool use_sim_data_hand, bool use_sim_data_obj);
    FingerState *finger_states[3];
    MotorState *motor_states[4] = {new MotorState(1), new MotorState(2), new MotorState(3), new MotorState(4)};

    ContactState getContactState();
    int getNumFingersInContact();
    int getFingerIdSingleContact();
    bool allFingersInContact();
    HandStateVariables getVars();

private:
    ros::NodeHandle *nh;
    ros::Subscriber reflex_state_sub;
    ros::Subscriber sim_state_sub;
    ros::Publisher hand_state_pub;
    tf2_ros::TransformBroadcaster br_reflex_measured;
    tf2_ros::TransformBroadcaster br_obj_measured;
    std::string object_name;
    tf2::Transform obj_measured;
    bool use_sim_data_hand;
    bool use_sim_data_obj;
    boost::mutex mtx;

    GraspQuality grasp_quality = GraspQuality();
    HandStateVariables vars = HandStateVariables();
    ContactState cur_state;

    void sim_state_callback(const sensor_listener::ContactFrames &msg);
    void reflex_state_callback(const reflex_msgs::Hand &msg);
    void updateHandStateReal();
    void updateHandStateSim();
    void broadcastModelState(tf2::Transform tf, std::string source_frame, std::string target_frame, tf2_ros::TransformBroadcaster *br);
    void updateQualityMetrics();
    TaskPolytope getTaskWrenches();
    reflex_interface::HandStateStamped getHandStateMsg();
    tf2::Vector3 create_vec_from_msg(const geometry_msgs::Vector3 &msg);
};

#endif