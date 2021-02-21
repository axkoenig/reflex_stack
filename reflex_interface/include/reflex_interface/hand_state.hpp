#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <vector>

#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <tf2_ros/transform_broadcaster.h>

#include "reflex_interface/finger_state.hpp"
#include "reflex_interface/grasp_quality.hpp"

class HandState
{
public:
    HandState(ros::NodeHandle *nh, bool use_sim_data_hand, bool use_sim_data_obj);
    ros::NodeHandle *nh;
    ros::Subscriber state_sub;
    ros::Publisher epsilon_pub;
    ros::Publisher num_contacts_pub;
    FingerState *finger_states[3];
    GraspQuality grasp_quality = GraspQuality();

    enum ContactState
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };

    ContactState getContactState();
    int getNumFingersInContact();
    int getFingerIdSingleContact();
    float getEpsilon(tf2::Vector3 object_com_world);
    std::vector<tf2::Transform> getContactFramesWorld() { return contact_frames_world; };
    std::vector<int> getNumSensorsInContactPerFinger() { return num_sensors_in_contact_per_finger; };
    std::vector<bool> getFingersInContact() { return fingers_in_contact; };
    bool allFingersInContact();

private:
    std::string object_name;
    ContactState cur_state;
    bool use_sim_data_hand;
    bool use_sim_data_obj;
    const int num_fingers = 3;
    const int num_motors = 4;
    const int num_sensors_per_finger = 9;
    std::vector<tf2::Transform> contact_frames_world;
    std::vector<int> num_sensors_in_contact_per_finger; // example: two sensors in contact on finger 1 and one on finger 2: {2, 1, 0}
    std::vector<bool> fingers_in_contact;               // example: fingers 1 and 3 in contact: {1, 0, 1}
    void callback(const reflex_msgs::Hand &msg);
    void updateContactFramesWorldSim();
    void updateContactFramesWorldReal();
    void broadcastModelState(tf2::Transform tf, std::string source_frame, std::string target_frame, tf2_ros::TransformBroadcaster *br);
    tf2_ros::TransformBroadcaster br_reflex_measured;
    tf2_ros::TransformBroadcaster br_obj_measured;
};

#endif