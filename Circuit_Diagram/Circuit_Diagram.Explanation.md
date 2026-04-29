This circuit is made using KiCAD a free open source software. This is a accurate depiction of the circuit connected to the robotic arm for ROS2 control. 

Each servo motor is connected directly to a PWM suppported pin the Arduino UNO R3 that is given by,

   1. rotate1  - 3
   2. link1    - 5
   3. link2    - 6
   4. rotate2  - 9
   5. end      - 10
   6. gripper  - 11

The Arduino UNO and all servo motors are directly powered by a 5V adapter so that the motors can gets uninterrupted power supply.