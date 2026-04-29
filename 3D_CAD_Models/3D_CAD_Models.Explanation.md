This project is totally made in SolidWorks and some generic parts like motors are taken from GrabCAD. 

- holdassem folder consist of all the parts related to the connector.

- roboassem folder consist of all the parts related to the robotic arm.

- SolidWorks to URDF folder consist of urdf files that are directly converted from the assembly file using "SolidWorks to URDF Exporter" tool. 

- This URDF can work on both ROS1 and ROS2 but the launch files directly works on ROS1 and not on ROS2. ROS2 needs its own launch.py file to show in Rviz2.