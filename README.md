# tracked_robot-
# Tracked Mobile Robot Control and Navigation

**Author:** Mykyta Semeniuk
**Field:** Mechanical Engineer
**LinkedIn:** [Mykyta Semeniuk](https://www.linkedin.com/in/mykytasemeniuk/)

## Project Overview

This repository contains the code and documentation for my Master's thesis project, which focused on the development, control, and navigation of a tracked mobile robot. The project involved implementing various robotics concepts to enable autonomous operation.

## Hardware Components
A single-board computer, Raspberry Pi, is used for high-level processing, and an Arduino controller is utilized for real-time control tasks. The sensing system for environment perception includes 2D LiDAR and a camera for image recognition. The connection diagram is illustrated below. 
![HDW](https://github.com/user-attachments/assets/d85cb51d-b9f3-4889-96a7-0808faf127da)


### 1. Map Generation using slam_toolbox

This video demonstrates the robot's capability to generate a 2D map of an unknown environment using the slam_toolbox package in ROS 2. The robot navigates an indoor environment, mapping the space and identifying objects. The robot's movement and image recognition are displayed in real-time, showcasing its ability to perceive and understand its surroundings. However, the  average FPS value is low.


https://github.com/user-attachments/assets/40bd45b4-075f-4438-ad33-17be5efa8ce2



### 2. Navigation using Nav2

This video showcases the robot's autonomous navigation capabilities utilizing the Nav2 stack in ROS.




https://github.com/user-attachments/assets/7e61e73b-3d19-4f81-a4a9-e47d0c081741




### 3. Waypoints Following

This video demonstrates the robot's ability to follow a predefined sequence of waypoints.


        

https://github.com/user-attachments/assets/4de4e4ea-43a5-4119-be20-b0c0731af374



### 4. Image Recognition with YOLO

The robot is also equipped with image recognition capabilities using a YOLO (You Only Look Once) model. This allows the robot to detect and classify objects in its environment.
![image_2025-04-18_14-06-33](https://github.com/user-attachments/assets/b52560e2-8d9a-4e2c-98a6-f6954ec97516)
