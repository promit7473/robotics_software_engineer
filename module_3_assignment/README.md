# Module 3 Assignment: URDF and Robot Creation in ROS 2

## Objective

This assignment focuses on understanding and applying the concepts of URDF (Unified Robot Description Format) to create custom robots in ROS 2. The goal is to design and build a robotic arm with multiple degrees of freedom (DOF), integrate it with a mobile platform, and create an Ackerman drive system.

## Tasks Overview

### Task 1: Create a Custom Transform Tree

- Designed a robotic arm with 3 degrees of freedom (DOF) using URDF.
- Defined the transform tree for the robotic arm without including any visualization tags, focusing solely on creating the correct transforms for the arm's joints.

![alt text](<Screencast from 12-18-2024 02_12_49 PM.gif>)

### Task 2: Add Joints and Visual Elements

- Enhanced the robotic arm by adding joints:
  - **Finger Joints:** Used prismatic joint types for the fingers.
  - **Base Joint:** Set as a continuous type.
  - **All Other Joints:** Defined as revolute joints.
- Added visualization tags to the robot's URDF to create the body, primarily using cylinder shapes for simplicity.

![alt text](<Screencast from 12-18-2024 02_14_04 PM.gif>)

### Task 3: Build a Mobile Manipulator

- Integrated the robotic arm with a mobile robot platform:
  - Placed the robotic arm on top of a differential drive robot.
  - Connected the arm using the `base_link` of the differential drive robot.
- Created an Ackerman Drive System:
  - Designed a car-like robot structure that represents the front axle rotations for turning, simulating an Ackerman steering mechanism.

![alt text](<Screencast from 12-18-2024 02_15_25 PM.gif>)

## URDF Files

- **Task 1:** `task1.urdf`
- **Task 2:** `task2.urdf`
- **Task 3:** `task3.urdf`

## Running the Code

To visualize and test the robot models in ROS 2, follow these steps:

1. **Build the Workspace:**
   ```bash
   cd ~/assignment_ws
   colcon build

2. ## Running the Code



   **Source the Workspace:**
   ```bash
   source install/setup.bash
   ```

   **Check if the URDF perfectly working or not**
   ```bash
   check_urdf /home/mhpromit7473/assignment_ws/install/module_3_assignment/share/module_3_assignment/urdf/task3.urdf
   ```

   **Run the commands**
   ```bash
   ros2 launch module_3_assignment task1_rviz.launch.py
   ros2 launch module_3_assignment task2_rviz.launch.py
   ros2 launch module_3_assignment task3_rviz.launch.py
   ```