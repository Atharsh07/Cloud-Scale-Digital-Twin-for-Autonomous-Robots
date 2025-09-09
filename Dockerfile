# Base ROS2 Humble + Gazebo
FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    numpy \
    websockets

# Copy workspace
COPY ./ros2_packages /ros2_ws/src
COPY ./gazebo_worlds /ros2_ws/gazebo_worlds

# Build ROS2 workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Source ROS2 setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]
