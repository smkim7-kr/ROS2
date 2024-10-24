
# Describing a Robot with URDF

## Overview
When designing robotic systems, a common language is essential for consistency and simplicity. ROS provides the **Unified Robot Description Format (URDF)** to describe a robot's physical characteristics. The robot's **description** is stored in this XML-based format and can be referenced across all the components of the ROS system.

This tutorial will guide you through understanding and creating your own URDF file, which contains information about a robot's **links**, **joints**, and various **properties** like **visual**, **collision**, and **inertial** data.

## Basic Structure of a URDF File

A URDF file is structured in **XML** format. It begins with an XML declaration followed by a **robot** tag that contains all other elements of the robot. The main components are:

- **Links**: Represent different physical parts of the robot. Each link has its own frame of reference (origin).
- **Joints**: Define how two links are connected, including their relative motion.

### XML Structure
Every URDF file must begin with an XML declaration:
```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Other elements go here -->
</robot>
```

### Defining Links
Each **link** represents a physical component of the robot and is defined with a name. It can have three main optional properties:
1. **Visual**: Specifies the appearance in RViz and Gazebo.
2. **Collision**: Defines the shape for physical interactions.
3. **Inertial**: Describes how the link responds to forces.

Example link definition:
```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

### Defining Joints
A **joint** defines the connection between two links and their relative motion. Common joint types include:
- **Revolute**: Rotational motion with fixed limits (e.g., a robotic arm).
- **Continuous**: Rotational motion with no limits (e.g., a wheel).
- **Prismatic**: Linear motion (e.g., a slider).
- **Fixed**: No relative motion.

Example joint definition:
```xml
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <axis xyz="0 1 0"/>
  <limit effort="10" lower="0" upper="1.57" velocity="1.0"/>
</joint>
```

## Detailed Properties

### Visual
The **visual** element defines what the robot looks like in tools like RViz. It can reference basic geometric shapes or 3D meshes.
- **Geometry**: Defines the shape (box, cylinder, sphere, or mesh).
- **Material**: Defines the color of the link, which only affects RViz.

Example:
```xml
<visual>
  <geometry>
    <cylinder radius="0.1" length="0.5"/>
  </geometry>
  <material name="green"/>
</visual>
```

### Collision
The **collision** element is used by the physics engine (e.g., Gazebo) to calculate physical interactions. It often mirrors the **visual** element but can use simpler shapes for efficiency.
Example:
```xml
<collision>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial
The **inertial** element defines the physical properties of a link, including:
- **Mass**: The weight of the link.
- **Inertia**: The distribution of mass affecting how the link rotates.
- **Origin**: The center of mass.

Example:
```xml
<inertial>
  <mass value="2.0"/>
  <origin xyz="0 0 0.05"/>
  <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
</inertial>
```

## Joints
The **joint** element defines a connection between two links and specifies the type of motion allowed.

- **Name**: The unique name of the joint.
- **Type**: Specifies the type of joint, such as `revolute` (rotational), `prismatic` (linear), or `fixed`.

Example:

```xml
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <axis xyz="0 1 0"/>
  <limit effort="10" lower="0" upper="1.57" velocity="1.0"/>
</joint>
```

#### Explanation of Tags:

- **`<joint name="arm_joint" type="revolute">`**:
  - Defines a joint with the name `arm_joint`.
  - The type is set to `revolute`, meaning this joint allows rotational movement around an axis.

- **`<parent link="base_link"/>`**:
  - Specifies the **parent link** of the joint, which is `base_link`. The joint connects this link to the child link.

- **`<child link="arm_link"/>`**:
  - Specifies the **child link** of the joint, which is `arm_link`. This link moves relative to the parent link through the joint.

- **`<axis xyz="0 1 0"/>`**:
  - Defines the **axis of rotation** for the joint. The vector `xyz="0 1 0"` indicates the joint rotates around the Y-axis.

- **`<limit effort="10" lower="0" upper="1.57" velocity="1.0"/>`**:
  - **Effort**: Specifies the maximum force or torque the joint can exert, in this case, `10`.
  - **Lower and Upper Limits**: These values define the range of motion for the joint in radians. `lower="0"` indicates no movement below 0 radians, and `upper="1.57"` means the joint can rotate up to 1.57 radians (90 degrees).
  - **Velocity**: Specifies the maximum velocity of the joint, which is `1.0` radians per second.

## Using Xacro for Macros and File Organization
URDF files can become large and complex. To simplify them, ROS provides **Xacro (XML Macros)**. Xacro allows for breaking URDFs into multiple files and reusing components like links and joints. This avoids repeating code and keeps things manageable.

### Example Xacro File
```xml
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="example_robot">
  <xacro:include filename="base.urdf.xacro"/>
  <xacro:include filename="arm.urdf.xacro"/>
</robot>
```

Xacro also supports:
- **Properties**: Define constants to avoid repetition.
- **Macros**: Define reusable templates for complex components.

## Full Example URDF

Below is an example of a full URDF structure for a simple robot (without xacro here):
```xml
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 0.1"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>

  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="1.0"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Conclusion
URDF files are the backbone of robot descriptions in ROS. By defining links and joints in a clear, structured way, you can create detailed models of robots, which are essential for simulation and visualization in tools like Gazebo and RViz. Additionally, Xacro simplifies the process, making large files more manageable by breaking them into reusable components.

