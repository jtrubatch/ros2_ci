FROM osrf/ros:galactic-desktop-focal

RUN apt-get update && apt-get install -y \
gazebo11 \
ros-galactic-gazebo-ros-pkgs \
ros-galactic-gazebo-ros2-control \
ros-galactic-ros2-control \
ros-galactic-ros2-controllers \
ros-galactic-joint-state-publisher \
ros-galactic-joint-state-broadcaster \
ros-galactic-robot-state-publisher \
ros-galactic-xacro \
ros-galactic-tf2-ros \
ros-galactic-tf2-tools \
python3-colcon-common-extensions \
&& rm -rf /var/lib/apt/lists/*

WORKDIR /
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

COPY ./tortoisebot_gazebo /ros2_ws/src/tortoisebot_gazebo
COPY ./tortoisebot_description /ros2_ws/src/tortoisebot_description
COPY ./tortoisebot_waypoints /ros2_ws/src/tortoisebot_waypoints
COPY ./tortoisebot_action /ros2_ws/src/tortoisebot_action
COPY ./tortoisebot_bringup /ros2_ws/src/tortoisebot_bringup

WORKDIR /ros2_ws
RUN ["/bin/bash", "-c", "source /opt/ros/galactic/setup.bash && cd /ros2_ws && colcon build --packages-select tortoisebot_action tortoisebot_description tortoisebot_gazebo tortoisebot_bringup && source install/setup.bash && colcon build --packages-select tortoisebot_waypoints"]
#RUN ["/bin/bash", "-c", "cd /ros2_ws "]
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

RUN sed -i 's|source "/opt/ros/\$ROS_DISTRO/setup.bash"|source "/ros2_ws/install/setup.bash"|g' /ros_entrypoint.sh

CMD ["/bin/bash", "-c", "ros2 launch tortoisebot_waypoints tortoisebot.launch.py"]
# COMMIT CHANGE