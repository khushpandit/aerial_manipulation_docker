How to Build
To build the simulation environment, run these commands:

git clone https://github.com/khushpandit/aerial_manipulation_docker.git
cd aerial_manipulation_docker
xhost +local:docker
docker compose build
docker compose run drone_sim

How to Run (Inside the Container)
Once you are inside the Docker container, open 8 separate terminal tabs. Run these exact commands in order:

Terminal 1: Launch Gazebo Simulation World
killall -9 ruby gz px4 MicroXRCEAgent px4_sitl 2>/dev/null
cd ~
source /opt/ros/jazzy/setup.bash
source ~/clearpath_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-gazebo-models/models:$GZ_SIM_RESOURCE_PATH
ros2 launch clearpath_gz simulation.launch.py world:=pipeline

Terminal 2: XRCE Agent
cd ~
MicroXRCEAgent udp4 -p 8888

Terminal 3: QGroundControl
cd ~/Downloads
./QGroundControl.AppImage

Terminal 4: Spawn the Payload Box
gz service -s /world/pipeline/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "'$HOME'/payload_box.sdf", name: "free_payload_box", pose: {position: {x: 0, y: 0, z: 0.8}}'

Terminal 5: ROS-GZ Bridge
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /gripper/camera@sensor_msgs/msg/Image@gz.msgs.Image /gripper/depth@sensor_msgs/msg/Image@gz.msgs.Image

Terminal 6: Spawn the Drone
cd ~/PX4-Autopilot
export PX4_GZ_WORLD="pipeline"
export PX4_GZ_MODEL_POSE="5,0,0.1,0,0,0"
make px4_sitl gz_x500

Terminal 7: Python Execution
cd ~/Downloads
source /opt/ros/jazzy/setup.bash
source ~/px4_ros_ws/install/setup.bash
python3 autonomous_gripper_HYBRID_GRIP_VERIFY_MICROLIFT.py

Terminal 8: Visualizer
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
