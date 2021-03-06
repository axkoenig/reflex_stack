#ifndef CONTACT_POINT_BUFFER_H
#define CONTACT_POINT_BUFFER_H

#include <vector>

#include <tf2/LinearMath/Vector3.h>

class SimContactResult
{
public:
    tf2::Vector3 position = tf2::Vector3(0, 0, 0);
    tf2::Vector3 normal = tf2::Vector3(0, 0, 0);
    tf2::Vector3 force = tf2::Vector3(0, 0, 0);
    tf2::Vector3 torque = tf2::Vector3(0, 0, 0);
};

class ContactPointBuffer
{
public:
    std::vector<SimContactResult> results;

    SimContactResult get_averaged_results();
    SimContactResult get_force_weighted_averaged_results();
};

#endif