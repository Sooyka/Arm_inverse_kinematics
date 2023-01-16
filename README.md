# Arm_inverse_kinematics
A simple numerical inverse kinematics model of the robotic arm with visualisation. Supports arbitrary number of joints of revolut, spherical or prismatic types.
For now only forward kinematics is implemented, but project should be completed soon (estimated to 18.01.2023).  

## Cloning

This repository contains visualisation submodule provided by Langwedocjusz. It should be separately cloned recursively inside arm_inverse_kinematics folder:

    git clone --recursive https://github.com/Sooyka/Arm_inverse_kinematics.git

After this it might be necceseary to run:
    
    git submodule update --init --recursive

## Usage

How to add joints and segments to an arm:

Coordinates are exponential coordinates on se(3) representing how the given joint will bend or how long will be given segment.

Rotations are normalized to -pi, pi. 

First three coordinates are for rotations and should be used only in joints.

For now. its up to the user to provide rotations that make sense regarding the joint type.

Second three of coordinates are for translations and should be used only for segments.

To properly integrate with visualisation segment length should be written as 5th coordinate.

## Binaries
In the build folder there is compiled binary for Linux.
