# contact frame array of variable size (because we don't know number of contacts beforehand)
std_msgs/Header header
ContactFrame[] contact_frames_world
# same info but in the shell frame
ContactFrame[] contact_frames_shell
int32 num_contact_frames