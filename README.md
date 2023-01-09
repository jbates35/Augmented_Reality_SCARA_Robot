# Augmented Reality SCARA Robot

[A short youtube video of the project.](https://www.youtube.com/watch?v=OVDXKHhm_M4)

This is a half-embedded linux system (it has the sensors, but not the actuators!). This is the base in which PotBot was created from, which is a *full* embedded system, where we used both the camera sensor and a few different actuators.

The master branch contains all files to make this project work, but there is another branch called "what_i_coded" which specifically contains the files that I made or modified (such as the CMake file).

These sets of labs basically encompass my robotics class. Labs 3-4 explore the idea of virtual cameras and then physical cameras (and detecting coordinate systems within). Afterwards, we put our efforts forth to creating a computer-generated SCARA robot which requires translation and rotation matrices (and the multiplications thereof) to build.

In lab 5, we focus on using forward kinematics which is where we use matrix math to put in joint angles and then draw from one joint to the next to the next.

In lab 6, we focus on inverse kinematics, which is where we say "I want the end effector (the tooltip of the robot) to be at these X and Y and Z coordinates," which we then use systems of equations (and a lot of arctan functions) to figure out what joint angles are required to implement those coordinates.

In lab 7 (final lab), we implement smooth trajectories. In other words, we use a 5th order polynomial (and a 6x6 matrix) to figure out a set of coordinates that accomodates acceleration and deceleration. This would be useful in real life when you look to protect motors, for instance.

This repo is designed to work with Visual Studio with the VisualGDB extension installed.
