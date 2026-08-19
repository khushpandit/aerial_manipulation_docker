FROM osrf/ros:jazzy-desktop

# 1. Install dependencies
RUN apt-get update && apt-get install -y \
    git make cmake python3-pip python3-colcon-common-extensions \
    wget nano tmux ros-jazzy-ros-gz ros-jazzy-rqt-image-view \
    libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

# 2. EXACT MIRROR: Create the specific folders you use
RUN mkdir -p /root/Downloads
RUN mkdir -p /root/px4_ros_ws/src
RUN mkdir -p /root/clearpath_ws/src

# 3. Put QGroundControl EXACTLY in the Downloads folder
RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage -O /root/Downloads/QGroundControl.AppImage && \
    chmod +x /root/Downloads/QGroundControl.AppImage

# 4. Install Micro-XRCE-DDS-Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && make && make install && ldconfig

# 5. Clone PX4 AND the exact Gazebo models you use for Terminal 1
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN git clone https://github.com/PX4/PX4-gazebo-models.git
RUN bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# 6. Copy ONLY the needed custom models, worlds, and code from your laptop
COPY ./px4_ros_ws_src /root/px4_ros_ws/src
COPY ./clearpath_ws_src /root/clearpath_ws/src
COPY ./payload_box.sdf /root/

# 7. Put your Python script EXACTLY in the Downloads folder
COPY ./autonomous_gripper_HYBRID_GRIP_VERIFY_MICROLIFT.py /root/Downloads/

# 8. Compile the setup
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /root/px4_ros_ws && colcon build && \
    cd /root/clearpath_ws && colcon build"

# 9. Auto-source all environments
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/px4_ros_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /root/clearpath_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
