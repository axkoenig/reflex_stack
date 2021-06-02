#include "sensor_listener/contact_point_buffer.hpp"

SimContactResult ContactPointBuffer::get_averaged_results()
{
    int num_contacts = results.size();

    // nothing to average with
    if (num_contacts == 1)
    {
        return results[0];
    }

    SimContactResult scr;

    for (int i = 0; i < num_contacts; i++)
    {
        scr.position += results[i].position;
        scr.normal += results[i].normal;
        scr.force += results[i].force;
        scr.torque += results[i].torque;
    }
    scr.position /= num_contacts;
    scr.normal /= num_contacts;
    scr.force /= num_contacts;
    scr.torque /= num_contacts;

    return scr;
}

SimContactResult ContactPointBuffer::get_force_weighted_averaged_results()
{
    int num_contacts = results.size();

    // nothing to average with
    if (num_contacts == 1)
    {
        return results[0];
    }

    SimContactResult scr;
    float total_force_mag = 0.0;

    for (int i = 0; i < num_contacts; i++)
    {
        float force_mag = results[i].force.length();
        total_force_mag += force_mag;

        // weigh sum with force magnitude
        scr.position += force_mag * results[i].position;
        scr.normal += force_mag * results[i].normal;

        // we want total wrench on link, hence only sum it up
        scr.torque += results[i].torque;
        scr.force += results[i].force;
    }
    scr.position /= total_force_mag;
    scr.normal /= total_force_mag;

    return scr;
}