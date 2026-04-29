This circuit is made using KiCAD a free open source software. This is a accurate depiction of the circuit connected to the robotic arm for ROS2 control. 

Each servo motor is connected directly to a PWM suppported pin the Arduino UNO R3 that is given by,

   - 03 - rotate1 
   - 05 - link1
   - 06 - link2
   - 09 - rotate2
   - 10 - end
   - 11 - gripper

The Arduino UNO and all servo motors are directly powered by a 5V adapter so that the motors can gets uninterrupted power supply.