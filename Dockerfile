FROM osrf/ros:jazzy-desktop

# Install essential dependencies
RUN apt-get update && apt-get install -y \
    git make cmake python3-pip python3-colcon-common-extensions \
    wget nano tmux wget \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# 1. Download a clean PX4-Autopilot directly from the source
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# 2. Copy ONLY your lightweight raw source code
COPY ./px4_ros_ws_src /root/px4_ros_ws/src
COPY ./clearpath_ws_src /root/clearpath_ws/src
COPY ./autonomous_gripper_HYBRID_GRIP_VERIFY_MICROLIFT.py /root/

# 3. Build the workspaces inside the container
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /root/px4_ros_ws && colcon build && \
    cd /root/clearpath_ws && colcon build"

# 4. Auto-source everything for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/px4_ros_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/clearpath_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
