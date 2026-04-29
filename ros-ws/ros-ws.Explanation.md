First install ROS2 and MoveIt2 from the given link before starting.

To avoid sourcing ros2 and it's required components everytime in the terminal individually and also not wanting to to also upload in the "~/.bashrc" file. 
I used the "ros2.sh" file to source everytime when it needed.

But before using the "ros2.sh" file make sure to change the location of the ros-ws folder as per the location that you saved. 
This file is movable so, if you move this file to home you can source it directly without any hassle. 

The robotic arm's model is given as "roboassem.urdf" and the gripper's models (that also includes both connector and the end effector) is given as "holdassem.urdf".

To simulate both model at same time, i used full.xacro to combine both the robotic arm (roboassem.urdf.xacro) and gripper (holdassem.urdf.xacro).
So, to run each file, i made seperate launch.py file to simulate each urdf files in Rviz2. One launch.py file and one controller.yaml for each urdf.xacro file.

To make the already existing file compatable with MoveIt2, put the following command in terminal after sourcing to get a setup file make this process simple.
    
    "ros2 launch moveit_setup_assistant setup_assistant.launch.py" 

To launch the Moveit2 file run the following command in terminal after sourcing.
    
    "ros2 launch panda_moveit_config demo.launch.py" 

To connect MoveIt2 and Arduino UNO R3, first upload "ard_connect.ino" in the ard-connect folder (ros-ws/src/aries/ard_connect).
Next run "ros2_to_arduino_bridge.py" in seperate terminal after the demo.launch.py is launched.
Finally run "run_sequence.py" in seperate terminal if you want to want to run the robotic arm in pre planned sequence. This also work even when Arduino is not connected.

To learn more, visit the official documentations to get clear in-depth knowledge.