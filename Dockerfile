FROM osrf/ros:jazzy-desktop

# 1. Install essential dependencies, ROS-GZ bridge, RQT, and QGC GUI requirements
RUN apt-get update && apt-get install -y \
    git make cmake python3-pip python3-colcon-common-extensions \
    wget nano tmux ros-jazzy-ros-gz ros-jazzy-rqt-image-view \
    libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# 2. Download and Build MicroXRCEAgent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && make && make install && ldconfig

# 3. Download a clean PX4-Autopilot directly from the source
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# Install PX4 dependencies (this takes a few minutes during build)
RUN bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# 4. Download QGroundControl
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O /root/QGroundControl.AppImage && \
    chmod +x /root/QGroundControl.AppImage

# 5. Copy your lightweight raw source code AND the payload box
COPY ./px4_ros_ws_src /root/px4_ros_ws/src
COPY ./clearpath_ws_src /root/clearpath_ws/src
COPY ./autonomous_gripper_HYBRID_GRIP_VERIFY_MICROLIFT.py /root/
COPY ./payload_box.sdf /root/

# 6. Build the workspaces inside the container
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /root/px4_ros_ws && colcon build && \
    cd /root/clearpath_ws && colcon build"

# 7. Auto-source everything for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/px4_ros_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/clearpath_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
