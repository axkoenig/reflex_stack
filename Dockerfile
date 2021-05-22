FROM osrf/ros:noetic-desktop-focal

# update package managers
RUN apt-get -y update && apt-get -y upgrade \
    && apt -y update && apt -y upgrade 

# install Gazebo dependencies
RUN apt-get -y install wget gnupg2 lsb-release \
    && apt-get -y remove ".*gazebo.*" ".*sdformat.*" ".*ignition-math.*" ".*ignition-msgs.*" ".*ignition-transport.*" \
    && sh -c "echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main' > /etc/apt/sources.list.d/gazebo-stable.list" \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && apt-get -y update \
    && wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh \
    && bash -c 'GAZEBO_MAJOR_VERSION=11 ROS_DISTRO=noetic . /tmp/dependencies.sh && echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d "\\" | xargs apt-get -y install'

# install DART 6
RUN apt-get -y install software-properties-common \
    && apt-add-repository ppa:dartsim \
    && apt-get -y update \
    && apt-get -y install libdart6-dev libdart6-utils-urdf-dev

# compile Gazebo 11 from source (DART physics engine requires installation from source)
RUN apt -y install git \
    && git clone https://github.com/osrf/gazebo /tmp/gazebo \
    && cd /tmp/gazebo \
    && mkdir build && cd build \
    && cmake ../ \
    && make -j4 \
    && make install \
    && rm -r /tmp/gazebo

# install ROS project dependencies
RUN apt-get -y install ros-noetic-ros-control \ 
    ros-noetic-ros-controllers  \ 
    ros-noetic-gazebo-ros-pkgs  \ 
    ros-noetic-gazebo-ros-control

# init catkin workspace, copy Reflex Stack over and build it
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir ${CATKIN_WS}/src -p \
    && cd ${CATKIN_WS}/src \
    && /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace'
COPY . ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# add workspace sourcing in bashrc (can be sourced with bash option '-l')
RUN /bin/bash -c 'echo "source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc'

# source workspace and execute simulation
CMD ["/bin/bash", "-c", "source ${CATKIN_WS}/devel/setup.bash; roslaunch description reflex.launch gui:=false" ]