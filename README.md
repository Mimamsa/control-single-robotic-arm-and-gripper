# Control Single Robotic Arm and Gripper in PyBullet

This repository is a minified version for controlling robotic arm and gripper in PyBullet.

Currently we have the following robotic arm and gripper combinations:

- Techman TM5-900 and Robotic Hand-E
- Universal Robots UR5e and Robotic Hand-E


## Limitation

This robotic arm is not able to grasp an object by friction. If you'd like to make any modification to grasp an object, try suggestions here:

[Applying force to grasp an object so that the object does not slip through the gripper #1936](https://github.com/bulletphysics/bullet3/issues/1936)

[Correct way to implement a gripper - pybullet.org](https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=10120)


## Installation

```
pip install -r requirements.txt
```

## Usage

Control robotic arm without on-hand camera attached
```
python3 main.py
```

Control robotic arm with on-hand camera attached
```
python3 main.py -ca
```

Press 'Space' to capture images.

Press 'q' to exit program.


## References

1. [Source of Robotiq URDF files](https://github.com/cambel/robotiq/tree/noetic-devel)

2. [Source of Techman URDF files](https://github.com/TechmanRobotInc/tmr_ros1)

3. [Source of Universal Robots URDF files](https://github.com/ros-industrial/universal_robot)
