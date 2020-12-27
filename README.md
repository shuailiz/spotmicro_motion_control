# Spotmicro Motion Control
This package contains code that controls the motions of the [spotmicro robot](https://spotmicroai.readthedocs.io/en/latest/). The robot is printed using the [Ender5 Pro](https://www.amazon.com/Upgrade-Creality-Upgraded-Extruder-Capricorn/dp/B081SPJ2VX/ref=sr_1_3?crid=ZD9XOPWWSXG2&dchild=1&keywords=ender+5+pro&qid=1609032821&sprefix=ender5+pro%2Caps%2C246&sr=8-3) printer.
<p align="center">
  <img src="https://raw.githubusercontent.com/shuailiz/spotmicro_motion_control/main/media/First_time_somewhat_stable_walking.gif" />
</p>

This package is intended to be used with ROS melodic. This package also requires this [low level motor controller package](https://github.com/shuailiz/spotmicro_motor_control) to communicate with the servos. The details for each of the sub-packages are:
### spotmicro_planner
Contains the motion planning libraries for controlling the robot's motion. Currently it contains:
* IK solver for the robot legs given either the foot toes' positions in the base link
* Gait generator that generates foot toes' positions given the gait requirements, such as the phase design. The gait generator currently uses [Bezier curve](https://en.wikipedia.org/wiki/B%C3%A9zier_curve#:~:text=A%20B%C3%A9zier%20curve%20(%2F%CB%88b,the%20bodywork%20of%20Renault%20cars.)) for the swing phase for each leg.
* A joint state re-publisher that translate the joint states from the robot to the planner. 

### spotmicro_description
Contains the urdf files for the spotmicro model.

### spotmicro_commander
Contains the robot commander that actually controls the motions of the robot. This package uses the planner to plan robot motions and sends the motion commands to the low level robot control nodes.

### spotmicro_teleop
Contains the tele-operation nodes that controls the motions of the spotmicro robot using a joystick (currently a ps4 controller).