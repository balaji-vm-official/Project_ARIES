# Project_ARIES

A team of 3 students including me are doing this project.

In this project we are making a connector that can change to multiple end effectors easily without any human intervention.
This connector doesn't need any electricity to work but also allow the end effector to use the electricity with a seperate section allocated to it.
This system includes 3 main components: host (in this case a robotic arm), end effector (in this case a gripper) and connector which connect both host and end effector.

The connector is placed in a holder, it contains 3 parts: top - (which has a hole so that the host can connect to it), button - (two spring activated mechanism used to lock, connect and hold the connector to the host properly) and bottom - (usually the end effector is fiexed this part)

I am making a robotic arm project using ROS2 Jazzy (installed from this website https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

I am going to control the robot directly using my laptop (instead of the Raspberry Pi to control) with the help of Arduino UNO.

To avoid sourcing ros2 and it required components everytime in the terminal individually and not wanting to to also upload in the "~bashrc" file. i used the ".ros2.sh" file to source everytime when it neeeded

The robotic arm's model is given as "roboassem.urdf" and the gripper's models (that also includes both connector and the end effector) is given as "holdassem.urdf".
To simulate both model at same time, i used full.xacro to combine both the robotic arm (roboassem.urdf.xacro) and gripper (holdassem.urdf.xacro).
Similarly i have seperate launch.py file to simulate eac urdf files in rviz. So total of 3 launch.py file and one for each urdf.xacro file. and i also have cotroller.yaml for all 3 urdf.xacro file.
