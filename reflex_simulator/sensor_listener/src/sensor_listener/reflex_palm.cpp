#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gazebo_interface/gazebo_interface.hpp"
#include "sensor_listener/reflex_palm.hpp"
#include "sensor_listener/contact_point_buffer.hpp"

ReflexPalm::ReflexPalm()
{
    std::string topic_name = "gazebo/" + sensor_link_name + "_sensor_bumper";
    sensor_link_sub = nh.subscribe(topic_name, 1, &ReflexPalm::contacts_callback, this);
}

tf2::Vector3 ReflexPalm::create_vec_from_msg(const geometry_msgs::Vector3 &msg)
{
    return tf2::Vector3{msg.x, msg.y, msg.z};
}

void ReflexPalm::contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    contact_frames.clear();
    int num_contact_states = msg.states.size();

    // no contacts
    if (num_contact_states == 0)
    {
        return;
    }

    tf2::Transform world_to_pad = getLinkPoseSim(&nh, sensor_link_name, "world", false);
    tf2::Vector3 link_y = world_to_pad * tf2::Vector3(0, 1, 0);
    tf2::Transform pad_to_world = world_to_pad.inverse();

    ContactPointBuffer cpb_over_time;

    for (int j = 0; j < num_contact_states - 1; j++)
    {
        int num_contacts = msg.states[j].wrenches.size();

        ContactPointBuffer cpb_over_link;

        for (int i = 0; i < num_contacts; i++)
        {
            SimContactResult scr;

            scr.force = create_vec_from_msg(msg.states[j].wrenches[i].force);
            float contact_force_magnitude = scr.force.length();

            // stop if force is smaller than thresh (we are doing this since
            // Gazebo returns many forces which are 0 and we want to ignore them)
            if (contact_force_magnitude < ignore_force_thresh)
            {
                continue;
            }

            // transform contact position from world frame to pad origin
            scr.position = create_vec_from_msg(msg.states[j].contact_positions[i]);
            tf2::Vector3 contact_position_on_pad = pad_to_world * scr.position;

            // stop if contact on back of palm
            if (contact_position_on_pad[1] < 0.0)
            {
                continue;
            }

            scr.normal = create_vec_from_msg(msg.states[j].contact_normals[i]);
            scr.torque = create_vec_from_msg(msg.states[j].wrenches[i].torque);

            // this is our dirty fix for the Gazebo bug that flips contact normals (see https://github.com/dartsim/dart/issues/1425)
            float angle = acos(link_y.dot(scr.normal) / (scr.normal.length() * link_y.length()));
            if (abs(angle) > M_PI / 2)
            {
                scr.normal *= -1;
                scr.force *= -1;
                scr.torque *= -1;
            }
            // all is well, add contact frame to vector
            cpb_over_link.results.push_back(scr);
        }

        if (!cpb_over_link.results.empty())
        {
            // compute weighted average to reduce result to one contact point
            cpb_over_time.results.push_back(cpb_over_link.get_force_weighted_averaged_results());
        }
    }

    // if we did not have any valid contacts we're done
    if (cpb_over_time.results.empty())
    {
        return;
    }

    SimContactResult avg_scr = cpb_over_time.get_averaged_results();

    sensor_listener::ContactFrame cf_msg;

    // calculate rotation of contact frame (x must align with contact normal)
    tf2::Vector3 world_x = tf2::Vector3{1, 0, 0};
    tf2::Quaternion rot_x_to_normal = tf2::shortestArcQuatNormalize2(world_x, avg_scr.normal);
    tf2::Transform contact_frame = tf2::Transform(rot_x_to_normal, avg_scr.position);

    // extract basis vectors of transform
    tf2::Vector3 cf_x = tf2::quatRotate(rot_x_to_normal, tf2::Vector3{1, 0, 0});
    tf2::Vector3 cf_y = tf2::quatRotate(rot_x_to_normal, tf2::Vector3{0, 1, 0});
    tf2::Vector3 cf_z = tf2::quatRotate(rot_x_to_normal, tf2::Vector3{0, 0, 1});

    // fill remaining message
    cf_msg.sensor_id = 0;
    cf_msg.hand_part_id = 0;
    cf_msg.palm_contact = true;
    cf_msg.prox_contact = false;
    cf_msg.dist_contact = false;
    cf_msg.contact_torque_magnitude = avg_scr.torque.length();
    cf_msg.contact_force_magnitude = avg_scr.force.length();
    cf_msg.contact_wrench.force = tf2::toMsg(avg_scr.force);
    cf_msg.contact_wrench.torque = tf2::toMsg(avg_scr.torque);
    cf_msg.contact_frame = tf2::toMsg(contact_frame);
    cf_msg.contact_position = tf2::toMsg(avg_scr.position);
    cf_msg.contact_normal = tf2::toMsg(avg_scr.normal);
    cf_msg.normal_force = avg_scr.force.dot(cf_x);
    cf_msg.tang_force_y = avg_scr.force.dot(cf_y);
    cf_msg.tang_force_z = avg_scr.force.dot(cf_z);

    contact_frames.push_back(cf_msg);
}
