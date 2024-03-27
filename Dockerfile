from ros:noetic

# Install ROS extra packages
RUN apt-get update &&\ 
    apt-get install -y python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools \
        python3-lxml \
        build-essential \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean
RUN apt-get update \
    && apt-get install -y ros-${ROS_DISTRO}-ros-control \ 
        ros-${ROS_DISTRO}-joint-state-controller \
        ros-${ROS_DISTRO}-velocity-controllers \
        ros-${ROS_DISTRO}-effort-controllers \ 
        ros-${ROS_DISTRO}-position-controllers \ 
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/* && apt autoremove && apt clean

# Range sensor deps install
RUN pip3 install git+https://github.com/pimoroni/VL53L0X-python.git

RUN mkdir -p /ros_ws/src && cd /ros_ws \ 
    && catkin config --extend /opt/ros/${ROS_DISTRO} \
    && catkin build && echo "source /ros_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /ros_ws

COPY ./ /ros_ws/src/omnicar_ros_driver

RUN catkin build

HEALTHCHECK --interval=20s --timeout=1s --retries=3 --start-period=20s CMD if [[ $(rostopic list 2> /dev/null | wc -c) > 0 ]]; then exit 0; fi;
CMD ["/bin/bash", "-ci", "roslaunch omnicar_ros_driver omnicar_driver.launch"]

