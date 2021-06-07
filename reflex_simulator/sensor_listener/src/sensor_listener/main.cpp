#include <ros/ros.h>
#include <reflex_msgs/Hand.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_listener/ContactFrames.h"
#include "sensor_listener/reflex_hand.hpp"
#include "gazebo_interface/gazebo_interface.hpp"

std::string node_name = "sensor_listener";
std::string hand_state_topic = "reflex_takktile/hand_state";
std::string contact_frames_topic = "reflex_takktile/sim_contact_frames";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Publisher hand_pub = nh.advertise<reflex_msgs::Hand>(hand_state_topic, 1);
    ros::Publisher cf_pub = nh.advertise<sensor_listener::ContactFrames>(contact_frames_topic, 1);
    ROS_INFO("Launched %s node.", node_name.c_str());

    ReflexHand hand;
    reflex_msgs::Hand hand_msg;

    float reflex_pub_rate;
    getParam(&nh, &reflex_pub_rate, "reflex_pub_rate");
    ros::Rate rate(reflex_pub_rate);

    ROS_INFO("Starting to listen to Gazebo sensor values.");
    ROS_INFO_STREAM("Publishing to " << hand_state_topic << " and " << contact_frames_topic);

    while (ros::ok())
    {
        sensor_listener::ContactFrames cfs_msg;

        for (int i = 0; i < hand.num_fingers; i++)
        {
            // fill hand message
            hand_msg.finger[i].proximal = hand.fingers[i].getProximalAngle();
            hand_msg.finger[i].distal_approx = hand.fingers[i].getDistalAngle();

            for (int j = 0; j < hand.num_sensors; j++)
            {
                hand_msg.finger[i].pressure[j] = hand.fingers[i].sensors[j]->getPressure();
                hand_msg.finger[i].contact[j] = hand.fingers[i].sensors[j]->getContact();
            }

            // fill contact frames message with prox and distal contacts
            cfs_msg.contact_frames_world.insert(cfs_msg.contact_frames_world.end(), hand.fingers[i].prox_contact_frames.begin(), hand.fingers[i].prox_contact_frames.end());
            cfs_msg.contact_frames_world.insert(cfs_msg.contact_frames_world.end(), hand.fingers[i].dist_contact_frames.begin(), hand.fingers[i].dist_contact_frames.end());
        }

        // iterate over motors
        for (int i = 0; i < hand.num_motors; i++)
        {
            hand_msg.motor[i].joint_angle = hand.motors[i]->getAngle();
            hand_msg.motor[i].velocity = hand.motors[i]->getVelocity();
            hand_msg.motor[i].load = hand.motors[i]->getLoad();
        }

        // add palm info to contact frames message
        cfs_msg.contact_frames_world.insert(cfs_msg.contact_frames_world.end(), hand.palm.contact_frames.begin(), hand.palm.contact_frames.end());
        cfs_msg.num_contact_frames = cfs_msg.contact_frames_world.size();

        // copy info over and transform to shell frame
        cfs_msg.contact_frames_shell = cfs_msg.contact_frames_world;
        tf2::Transform world_to_shell = getLinkPoseSim(&nh, "shell", "world", false);
        tf2::Transform shell_to_world = world_to_shell.inverse();

        for (int i = 0; i < cfs_msg.num_contact_frames; i++)
        {
            // this info must be transformed (i.e. translation and rotation )
            tf2::Vector3 vec;
            tf2::Transform frame;
            tf2::fromMsg(cfs_msg.contact_frames_shell[i].contact_position, vec);
            tf2::fromMsg(cfs_msg.contact_frames_shell[i].contact_frame, frame);
            cfs_msg.contact_frames_shell[i].contact_position = tf2::toMsg(shell_to_world * vec);
            cfs_msg.contact_frames_shell[i].contact_frame = tf2::toMsg(shell_to_world * frame);

            // this info must only be rotated
            shell_to_world.setOrigin(tf2::Vector3(0, 0, 0));
            tf2::fromMsg(cfs_msg.contact_frames_shell[i].contact_normal, vec);
            cfs_msg.contact_frames_shell[i].contact_normal = tf2::toMsg(shell_to_world * vec);
            tf2::fromMsg(cfs_msg.contact_frames_shell[i].contact_wrench.force, vec);
            cfs_msg.contact_frames_shell[i].contact_wrench.force = tf2::toMsg(shell_to_world * vec);
            tf2::fromMsg(cfs_msg.contact_frames_shell[i].contact_wrench.torque, vec);
            cfs_msg.contact_frames_shell[i].contact_wrench.torque = tf2::toMsg(shell_to_world * vec);
        }

        cfs_msg.header.stamp = ros::Time::now();

        hand_pub.publish(hand_msg);
        cf_pub.publish(cfs_msg);
        ros::spinOnce();
        rate.sleep();
    }
}