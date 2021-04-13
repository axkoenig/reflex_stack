#ifndef REFLEX_SENSOR_H
#define REFLEX_SENSOR_H

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>

class ReflexSensor
{
private:
    double pressure = 0.0;
    bool contact = false;

    // use rolling buffer to filter noisy sensing from simulation
    int buf_size = 5;
    std::vector<bool> contact_buffer = {0, 0, 0, 0, 0};
    std::vector<float> pressure_buffer = {0, 0, 0, 0, 0};
    int default_contact_threshold = 20; // this default value comes from reflex driver
    int max_pressure_val = 255;
    float pressure_at_max_val = 40;     // 40 Newtons on sensor in simulation corresponds to maximum pressure reading

public:
    int getPressure();
    bool getContact();
    void addContactToBuffer(const bool contact);
    void addPressureToBuffer(const float pressure);
};

#endif