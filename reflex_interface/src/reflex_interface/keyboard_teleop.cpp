#include <iostream>
#include <map>
#include <termios.h>
#include <math.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>

#include "reflex_interface/GraspPrimitive.h"
#include "reflex_interface/PosIncrement.h"

std::string node_name = "finger_teleop_node";
std::string source_frame = "world";
std::string target_frame = "reflex";

std::string open_srv_name = "reflex_interface/open";
std::string close_srv_name = "reflex_interface/close";
std::string pinch_srv_name = "reflex_interface/pinch";
std::string sph_open_srv_name = "reflex_interface/spherical_open";
std::string sph_close_srv_name = "reflex_interface/spherical_close";
std::string pos_incr_srv_name = "reflex_interface/position_increment";
std::string close_until_contact_srv_name = "reflex_interface/close_until_contact";
std::string tighten_grip_srv_name = "reflex_interface/tighten_grip";

float trans_scaling = 0.01;
float rot_scaling = 0.2;
float finger_scaling = 0.1;

// format: {x, y, z, r, p ,y} in "reflex" frame
std::array<float, 6> init_pose = {0, 0, 0.1, -M_PI / 2, 0, 0};

tf2::Transform getTcpToWristFrame()
{
    float z_offset = -0.09228;
    float x_offset = -0.02;

    tf2::Transform translate_to_wrist = tf2::Transform();
    translate_to_wrist.setIdentity();
    translate_to_wrist.setOrigin(tf2::Vector3{x_offset, 0, z_offset});

    return translate_to_wrist;
}

// keys for wrist teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> wrist_bindings{

    // format: {x, y, z, r, p ,y}
    {'u', {1, 0, 0, 0, 0, 0}},
    {'i', {0, 1, 0, 0, 0, 0}},
    {'o', {0, 0, 1, 0, 0, 0}},
    {'j', {0, 0, 0, 1, 0, 0}},
    {'k', {0, 0, 0, 0, 1, 0}},
    {'l', {0, 0, 0, 0, 0, 1}},
    {'U', {-1, 0, 0, 0, 0, 0}},
    {'I', {0, -1, 0, 0, 0, 0}},
    {'O', {0, 0, -1, 0, 0, 0}},
    {'J', {0, 0, 0, -1, 0, 0}},
    {'K', {0, 0, 0, 0, -1, 0}},
    {'L', {0, 0, 0, 0, 0, -1}},
};

// keys for reflex finger teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> finger_bindings{

    // format: {f1, f2, f3, preshape}
    {'q', {1, 0, 0, 0}},
    {'w', {0, 1, 0, 0}},
    {'e', {0, 0, 1, 0}},
    {'r', {0, 0, 0, 1}},
    {'Q', {-1, 0, 0, 0}},
    {'W', {0, -1, 0, 0}},
    {'E', {0, 0, -1, 0}},
    {'R', {0, 0, 0, -1}},
};

// BEGIN CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

// for non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // get the current character
    ch = getchar();

    // reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

// END CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

tf2::Transform calcTransformFromEuler(std::array<float, 6> pose)
{
    tf2::Vector3 t = {pose[0], pose[1], pose[2]};
    tf2::Quaternion q;
    q.setRPY(pose[3], pose[4], pose[5]);
    tf2::Transform transform(q, t);

    return transform;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(100);
    ROS_INFO("Launched %s node.", node_name.c_str());

    // service clients for hand_command_node
    ros::ServiceClient open_client = nh.serviceClient<reflex_interface::GraspPrimitive>(open_srv_name);
    ros::ServiceClient close_client = nh.serviceClient<reflex_interface::GraspPrimitive>(close_srv_name);
    ros::ServiceClient pinch_client = nh.serviceClient<reflex_interface::GraspPrimitive>(pinch_srv_name);
    ros::ServiceClient sph_open_client = nh.serviceClient<reflex_interface::GraspPrimitive>(sph_open_srv_name);
    ros::ServiceClient sph_close_client = nh.serviceClient<reflex_interface::GraspPrimitive>(sph_close_srv_name);
    ros::ServiceClient pos_incr_client = nh.serviceClient<reflex_interface::PosIncrement>(pos_incr_srv_name);
    ros::ServiceClient close_until_contact_client = nh.serviceClient<std_srvs::Trigger>(close_until_contact_srv_name);
    ros::ServiceClient tighten_grip_client = nh.serviceClient<std_srvs::Trigger>(tighten_grip_srv_name);

    // service messages
    std_srvs::Trigger trigger;
    reflex_interface::PosIncrement pos_incr;
    reflex_interface::GraspPrimitive gp;

    // init transform broadcaster
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Transform transform;

    // populate initial wrist transform (we are controlling TCP)
    std::array<float, 6> cur_pose = init_pose;
    transform = calcTransformFromEuler(init_pose) * getTcpToWristFrame();
    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;
    ts.transform = tf2::toMsg(transform);

    // TODO find another, more elegant solution for this
    // wait before publishing first transform to fix warning from wrist_controller_node
    // ""reflex" passed to lookupTransform argument target_frame does not exist."
    // this also waits for services of reflex interface to spawn
    ros::Duration(3).sleep();

    // send initial wrist transform
    ts.header.stamp = ros::Time::now();
    br.sendTransform(ts);
    open_client.call(gp);

    ROS_INFO("Listening to keyboard input...");
    char key(' ');

    while (ros::ok())
    {
        key = getch();

        // WRIST CONTROL --------------------------------------------------------
        if (wrist_bindings.count(key) == 1)
        {
            // x, y, z
            cur_pose[0] += trans_scaling * wrist_bindings[key][0];
            cur_pose[1] += trans_scaling * wrist_bindings[key][1];
            cur_pose[2] += trans_scaling * wrist_bindings[key][2];

            // r, p, y
            cur_pose[3] += trans_scaling * wrist_bindings[key][3];
            cur_pose[4] += trans_scaling * wrist_bindings[key][4];
            cur_pose[5] += trans_scaling * wrist_bindings[key][5];

            ROS_INFO("Wrist transform updated.");
        }
        else if (key == 'm')
        {
            // reset wrist to init frame
            cur_pose = init_pose;
            ROS_INFO("Wrist transform reset.");
        }

        transform = calcTransformFromEuler(cur_pose) * getTcpToWristFrame();
        ts.header.stamp = ros::Time::now();
        ts.transform = tf2::toMsg(transform);
        br.sendTransform(ts);

        // FINGER CONTROL (TELEOP) ----------------------------------------------
        if (finger_bindings.count(key) == 1)
        {
            pos_incr.request.f1 = finger_scaling * finger_bindings[key][0];
            pos_incr.request.f2 = finger_scaling * finger_bindings[key][1];
            pos_incr.request.f3 = finger_scaling * finger_bindings[key][2];
            pos_incr.request.preshape = finger_scaling * finger_bindings[key][3];
            pos_incr_client.call(pos_incr);
            ROS_INFO_STREAM(pos_incr.response.message);
        }

        // FINGER CONTROL (PRIMITIVES) ------------------------------------------
        switch (key)
        {
        case 'y':
        {
            pinch_client.call(gp);
            ROS_INFO_STREAM(gp.response.message);
            break;
        }
        case 'x':
        {
            open_client.call(gp);
            ROS_INFO_STREAM(gp.response.message);
            break;
        }
        case 'c':
        {
            close_client.call(gp);
            ROS_INFO_STREAM(gp.response.message);
            break;
        }
        case 'v':
        {
            sph_open_client.call(gp);
            ROS_INFO_STREAM(gp.response.message);
            break;
        }
        case 'b':
        {
            sph_close_client.call(gp);
            ROS_INFO_STREAM(gp.response.message);
            break;
        }
        case 'n':
        {
            close_until_contact_client.call(trigger);
            ROS_INFO_STREAM(trigger.response.message);
            break;
        }
        case 't':
        {
            tighten_grip_client.call(trigger);
            ROS_INFO_STREAM(trigger.response.message);
            break;
        }
        }
        // EXIT ------------------------------------------
        if (key == '\x03')
        {
            ROS_INFO("Pressed Ctrl-C. Shutting down...");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}