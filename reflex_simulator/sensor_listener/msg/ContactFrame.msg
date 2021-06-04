# message for contact frames and wrenches that we measure from the simulation
# all data in world frame by default!

# contact frame x axis points along contact normal and (y,z) form a right-handed coosy 
geometry_msgs/Transform contact_frame

# this is already included in contact_frame, but still publishing for completeness
geometry_msgs/Vector3 contact_position 
geometry_msgs/Vector3 contact_normal

# on which Reflex hand part the contact is (ranges from 0 to 3, 0 means palm contact, fingers are 1,2,3) 
int32 hand_part_id

# to which Reflex sensor would this contact belong (ranges from 0 to 9, 0 means palm contact) 
int32 sensor_id

# only one of these can be true at a time
bool palm_contact
bool prox_contact
bool dist_contact

# measured contact wrench in world frame
geometry_msgs/Wrench contact_wrench

# magnitudes of contact wrench
float64 contact_force_magnitude
float64 contact_torque_magnitude