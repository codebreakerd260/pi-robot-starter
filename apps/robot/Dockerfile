# Use the official ROS 2 Humble image with desktop and Gazebo support
FROM osrf/ros:humble-desktop

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# Set timezone to prevent interactive prompts during package installation
ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install necessary ROS 2 packages and other dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create and set the working directory for the ROS 2 workspace
WORKDIR /ros2_ws

# Copy the source space
COPY ./src ./src

# Build the ROS 2 workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Create an entrypoint script to source the ROS 2 workspace
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
