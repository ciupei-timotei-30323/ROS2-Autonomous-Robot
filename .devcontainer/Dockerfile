FROM osrf/ros:jazzy-desktop

# Install colcon build tools, git, and other essentials
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Automatically source ROS 2 when you open a terminal in the container
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
