
# Creating a Rough 3D Model of Our Robot with URDF

## Overview
This tutorial focuses on designing a differential drive robot using URDF (Unified Robot Description Format) in ROS2. We'll build a simple 3D model of the robot, set up a URDF file for the robot's structure, and prepare it for simulation in Gazebo.

The differential drive robot has two driven wheels, and any additional caster wheels simply provide balance and stability. We'll start by defining the main components of the robot and writing the necessary URDF and Xacro (XML macros) files.

## Prerequisites
To follow along with this tutorial, make sure you've already completed the following setup:
1. Installed ROS2.
2. Created a workspace, built it with `colcon`, and sourced it.
3. Installed the necessary tools (`xacro`, `joint_state_publisher_gui`) with the following commands:
   ```bash
   sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui
   ```

Additionally, you should be familiar with URDF and Xacro files, which describe the structure of the robot. If not, refer to previous tutorials on URDF basics.

## Key Concepts
1. **Differential Drive**: The robot is controlled by two wheels, each driven independently, allowing it to move forward, backward, rotate in-place.
2. **Base Link**: The main reference frame for our robot, located at the center of the two driven wheels.
3. **Chassis**: The body of the robot.
4. **URDF and Xacro**: We'll create modular robot descriptions in URDF format, using Xacro to organize and simplify the files.

## Setting Up the URDF Files

### Creating the Robot Core File
1. Navigate to your ROS2 workspace and locate the package you created.
2. Find the `robot.urdf.xacro` file under the `description` directory. We'll use this file to include separate component files.
3. Delete the existing content and add the following:
   ```xml
   <xacro:include filename="robot_core.xacro"/>
   ```
   This includes the core structure of the robot in a separate file named `robot_core.xacro`.

4. Now, create the `robot_core.xacro` file inside the `description` directory. This file will define the base link, chassis, and wheels of the robot.

### Defining Materials (Colors)
At the beginning of `robot_core.xacro`, we define the materials (colors) for different components of the robot:
```xml
<material name="white">
  <color rgba="1 1 1 1"/> <!-- defines rgb & alpha value -->
</material>
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
<material name="black">
  <color rgba="0 0 0 1"/>
</material>
```
These colors will be used throughout the URDF to color different parts of the robot.

### Base Link and Chassis
Next, define the base link and chassis of the robot. The base link represents the robot's coordinate frame, and the chassis is the body of the robot. 
(Note: xyz axis are defined forward, left, and up)
```xml
<!-- Base Link -->
<link name="base_link"/> <!-- base_link is main coordinate frame of the robot -->

<!-- Chassis -->
<joint name="chassis_joint" type="fixed">
  <parent link="base_link"/>
  <child link="chassis"/>
  <origin xyz="-0.1 0 0"/> <!-- chassis coordinate is 0.1m behind x-axis -->
</joint>

<link name="chassis">
  <visual>
    <geometry>
      <box size="0.3 0.3 0.15"/>
    </geometry>
    <material name="white"/>
    <origin xyz="0.15 0 0.075"/> <!-- when xyz="0 0 0", the origin is set to center of box -->
  </visual>
</link>
```

### Wheels (Left and Right)
The robot uses two driven wheels: one on the left and one on the right. Each wheel is defined as a cylindrical shape in the URDF.
```xml
<!-- Left Wheel -->
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 -0.175 0" rpy="-1.5708 0 0"/>
  <axis xyz="0 0 1"/>
</joint>

<link name="left_wheel">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
    <material name="blue"/>
    <origin xyz="0 0 0"/>
  </visual>
</link>

<!-- Right Wheel (mirror of left) -->
<joint name="right_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="right_wheel"/>
  <origin xyz="0 0.175 0" rpy="1.5708 0 0"/>
  <axis xyz="0 0 -1"/>
</joint>

<link name="right_wheel">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
    <material name="blue"/>
    <origin xyz="0 0 0"/>
  </visual>
</link>
```

### Caster Wheel
We'll add a caster wheel at the front of the robot to provide stability.
```xml
<!-- Caster Wheel -->
<joint name="caster_joint" type="fixed">
  <parent link="chassis"/>
  <child link="caster_wheel"/>
  <origin xyz="0.24 0 0"/>
</joint>

<link name="caster_wheel">
  <visual>
    <geometry>
      <sphere radius="0.05"/>
    </geometry>
    <material name="black"/>
  </visual>
</link>
```

## Adding Inertial and Collision Properties
For each link, we need to define collision and inertia to simulate physics in Gazebo.

### Example of Collision and Inertial Properties for the Chassis:
```xml
<collision>
  <geometry>
    <box size="0.3 0.3 0.15"/>
  </geometry>
</collision>

<inertial>
  <mass value="0.5"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
</inertial>
```
Repeat this for the wheels and caster.

## Visualizing the Robot in RViz
Once you've written the URDF file, you can launch the robot model in RViz for visualization.

1. Source the workspace:
   ```bash
   source install/setup.bash
   ```

2. Run the launch file to start the robot state publisher:
   ```bash
   ros2 launch <your_package_name> rsp.launch.py
   ```

3. Open RViz and visualize the robot model:
   ```bash
   rviz2
   ```

In RViz, set the "Fixed Frame" to `base_link` and add the "RobotModel" display to visualize the robot's 3D structure.

## Conclusion
By the end of this tutorial, you will have built a rough 3D model of a differential drive robot, described in URDF format. This structure will serve as the foundation for further development, including adding sensors and preparing the robot for simulation in Gazebo.

In the next part, we'll drop the robot into Gazebo, simulate its behavior, and explore how to drive it around. Stay tuned!
