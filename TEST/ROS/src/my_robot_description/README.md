# 🤖 Custom Parametric 4-Mecanum Robot: Creation & Testing Guide

This guide walks you through building and verifying a custom, fully-parametric 4-Mecanum Wheel Robot in ROS 2 Jazzy and Gazebo Harmonic.

We utilize a distributed split-container environment:

- **Server Container** (`jazzy_server_dev`): computes URDF parsing, frames, and transforms headlessly.
- **GUI Container** (`jazzy_gui_dev`): visualizes the robot in `rviz2` and spawns interactive sliders.
- **Pristine Host Strategy**: only the `./src` folder is shared. No build pollution (`build/`, `install/`, `log/`) is written to your local host disk.

---

## 📂 Step 1: Create the Package Structure

Enter your running Server container:

```bash
docker exec -it jazzy_server_dev bash
```

Navigate to your workspace source partition and create a clean ROS 2 build package:

```bash
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_robot_description
```

Create the directories required to hold your design, configuration, and launch scripts:

```bash
mkdir -p my_robot_description/urdf my_robot_description/launch my_robot_description/config
```

---

## 📐 Step 2: Write the Parametric Xacro Model

Create the master robot definition file. This file uses adjustable variables at the top. Changing these automatically updates the collision dimensions, mass inertia tensors, and wheel mounting distances.

Create the file:

```bash
nano my_robot_description/urdf/robot.urdf.xacro
```

Paste the following complete model:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mecanum_robot">

    <!-- ==================================================== -->
    <!-- MASTER VARIABLES (Tweak these to reshape your robot) -->
    <!-- ==================================================== -->
    
    <!-- Chassis Parameters -->
    <xacro:property name="chassis_length" value="0.60"/>  <!-- Length along X axis (meters) -->
    <xacro:property name="chassis_width"  value="0.40"/>  <!-- Width along Y axis (meters) -->
    <xacro:property name="chassis_height" value="0.15"/>  <!-- Thickness along Z axis (meters) -->
    <xacro:property name="chassis_mass"   value="15.0"/>  <!-- Weight (kg) -->

    <!-- Wheel Parameters -->
    <xacro:property name="wheel_radius"    value="0.10"/> <!-- Radius of the mecanum tires -->
    <xacro:property name="wheel_thickness" value="0.06"/> <!-- Width of the tread -->
    <xacro:property name="wheel_mass"      value="1.5"/>  <!-- Weight per wheel (kg) -->

    <!-- Structural Clearances & Ground Heights -->
    <xacro:property name="ground_clearance" value="0.04"/> 

    <!-- Placement Overrides (Offsets relative to chassis edges) -->
    <xacro:property name="wheel_inset_x" value="0.08"/> 
    <xacro:property name="wheel_inset_y" value="0.03"/> 

    <!-- Lidar Parameters (For future Nav2 scanning) -->
    <xacro:property name="lidar_radius" value="0.05"/>
    <xacro:property name="lidar_height" value="0.04"/>
    <xacro:property name="lidar_mass"   value="0.2"/>

    <!-- ==================================================== -->
    <!-- CALCULATED PROPERTIES (Math derivatives from variables)--> 
    <!-- ==================================================== -->
    <xacro:property name="wheel_pos_x" value="${(chassis_length / 2.0) - wheel_inset_x}"/>
    <xacro:property name="wheel_pos_y" value="${(chassis_width / 2.0) + wheel_inset_y}"/>
    <xacro:property name="chassis_pos_z" value="${ground_clearance + (chassis_height / 2.0)}"/>
    <xacro:property name="wheel_pos_z"   value="${wheel_radius}"/>

    <xacro:property name="wheelbase"        value="${wheel_pos_x * 2.0}"/> 
    <xacro:property name="wheel_separation" value="${wheel_pos_y * 2.0}"/> 

    <!-- ==================================================== -->
    <!-- INERTIAL MATH MACROS (Automatic Weight Physics)      -->
    <!-- ==================================================== -->
    <xacro:macro name="inertial_box" params="mass x y z">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="${mass}"/>
            <inertia ixx="${(mass/12.0) * (y*y + z*z)}" ixy="0.0" ixz="0.0"
                     iyy="${(mass/12.0) * (x*x + z*z)}" iyz="0.0"
                     izz="${(mass/12.0) * (x*x + y*y)}"/>
        </inertial>
    </xacro:macro>

    <xacro:macro name="inertial_cylinder" params="mass radius length">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="${mass}"/>
            <inertia ixx="${(mass/12.0) * (3*radius*radius + length*length)}" ixy="0.0" ixz="0.0"
                     iyy="${(mass/12.0) * (3*radius*radius + length*length)}" iyz="0.0"
                     izz="${(0.5) * mass * (radius*radius)}"/>
        </inertial>
    </xacro:macro>

    <!-- ==================================================== -->
    <!-- STRUCTURAL ROBOT TREE DESIGN                         -->
    <!-- ==================================================== -->

    <!-- BASE_LINK: Footprint coordinate projected directly on the floor -->
    <link name="base_link"/>

    <!-- CHASSIS LINK -->
    <link name="chassis">
        <visual>
            <geometry>
                <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
            </geometry>
            <material name="blue"><color rgba="0.1 0.1 0.8 1.0"/></material>
        </visual>
        <collision>
            <geometry>
                <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
            </geometry>
        </collision>
        <xacro:inertial_box mass="${chassis_mass}" x="${chassis_length}" y="${chassis_width}" z="${chassis_height}"/>
    </link>

    <joint name="chassis_joint" type="fixed">
        <parent link="base_link"/>
        <child link="chassis"/>
        <origin xyz="0 0 ${chassis_pos_z}" rpy="0 0 0"/>
    </joint>

    <!-- PARAMETRIC WHEEL MACRO -->
    <xacro:macro name="mecanum_wheel" params="prefix x_reflect y_reflect">
        <link name="${prefix}_wheel">
            <visual>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_thickness}"/>
                </geometry>
                <material name="black"><color rgba="0.0 0.0 0.0 1.0"/></material>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_thickness}"/>
                </geometry>
            </collision>
            <xacro:inertial_cylinder mass="${wheel_mass}" radius="${wheel_radius}" length="${wheel_thickness}"/>
        </link>

        <joint name="${prefix}_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="${prefix}_wheel"/>
            <origin xyz="${x_reflect * wheel_pos_x} ${y_reflect * wheel_pos_y} ${wheel_pos_z}" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
    </xacro:macro>

    <!-- Instantiate Wheels -->
    <xacro:mecanum_wheel prefix="front_left"  x_reflect="1"  y_reflect="1" />
    <xacro:mecanum_wheel prefix="front_right" x_reflect="1"  y_reflect="-1" />
    <xacro:mecanum_wheel prefix="rear_left"   x_reflect="-1" y_reflect="1" />
    <xacro:mecanum_wheel prefix="rear_right"  x_reflect="-1" y_reflect="-1" />

    <!-- LASER SCANNER FRAME -->
    <link name="laser_frame">
        <visual>
            <geometry>
                <cylinder radius="${lidar_radius}" length="${lidar_height}"/>
            </geometry>
            <material name="red"><color rgba="0.8 0.1 0.1 1.0"/></material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="${lidar_radius}" length="${lidar_height}"/>
            </geometry>
        </collision>
        <xacro:inertial_cylinder mass="${lidar_mass}" radius="${lidar_radius}" length="${lidar_height}"/>
    </link>

    <joint name="laser_joint" type="fixed">
        <parent link="chassis"/>
        <child link="laser_frame"/>
        <origin xyz="${chassis_length/4.0} 0 ${(chassis_height/2.0) + (lidar_height/2.0)}" rpy="0 0 0"/>
    </joint>

    <!-- ==================================================== -->
    <!-- SIMULATION PLUGINS (For Gazebo integration)          -->
    <!-- ==================================================== -->
    <gazebo>
        <plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">
            <front_left_joint>front_left_wheel_joint</front_left_joint>
            <front_right_joint>front_right_wheel_joint</front_right_joint>
            <back_left_joint>rear_left_wheel_joint</back_left_joint>
            <back_right_joint>rear_right_wheel_joint</rear_right_joint>
            
            <wheelbase>${wheelbase}</wheelbase>
            <wheel_separation>${wheel_separation}</wheel_separation>
            <wheel_radius>${wheel_radius}</wheel_radius>
            
            <odom_publish_frequency>50</odom_publish_frequency>
            <topic>/cmd_vel</topic>
            <odom_topic>/odom</odom_topic>
            <tf_topic>/tf</tf_topic>
            <frame_id>odom</frame_id>
            <child_frame_id>base_link</child_frame_id>
        </plugin>
    </gazebo>

    <gazebo>
        <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
            <joint>front_left_wheel_joint</joint>
            <joint>front_right_wheel_joint</joint>
            <joint>rear_left_wheel_joint</joint>
            <joint>rear_right_wheel_joint</joint>
        </plugin>
    </gazebo>

</robot>
```

---

## 📄 Step 3: Configure Build Manifests

Before building, configure package exports and package dependencies.

### 1. Configure `CMakeLists.txt`

Open the file:

```bash
nano my_robot_description/CMakeLists.txt
```

Paste this configuration:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

# Installs asset and configuration folder trees to package path
install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 2. Configure `package.xml`

Open the file:

```bash
nano my_robot_description/package.xml
```

Paste this configuration:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_description</name>
  <version>0.0.0</version>
  <description>Parametric Mecanum Robot Description Package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 🏗️ Step 4: Build Your Workspace (On the Server)

Return to the root of your workspace inside the Server container and compile your modifications:

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚦 Step 5: Test Across the Network

Now run the headless state publisher on the Server and display/manipulate it on the GUI side.

### 1. Launch the Server Publisher (Terminal 1)

Inside your Server container (`jazzy_server_dev`), run:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /root/ros2_ws/src/my_robot_description/urdf/robot.urdf.xacro)"
```

### 2. Launch RViz (Terminal 2)

Open a new terminal tab on your host machine, enter your GUI container, and start RViz:

```bash
docker exec -it jazzy_gui_dev bash
rviz2
```

Configure RViz:

- In the left panel, set **Fixed Frame** to `base_link`
- Click **Add**
- Select **RobotModel** and hit **OK**
- Click **Add** again
- Select **TF (Transforms)** and hit **OK**

Your blue 3D chassis and wheels will instantly populate inside the window.

### 3. Rotate the Wheels with Sliders (Terminal 3)

Open a third terminal tab on your host machine, enter your GUI container, and launch the interactive sliders:

```bash
docker exec -it jazzy_gui_dev bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

Move the sliders in the graphical popup window, and watch your wheels spin inside RViz.

---

## 🔍 Step 6: Quick Syntax Verification (No GUI Required)

To check for typos or incorrect brackets inside your Xacro file without running heavy programs, run these inside the Server container:

```bash
# Move to your urdf directory
cd /root/ros2_ws/src/my_robot_description/urdf

# Compile Xacro manually to a plain XML file
ros2 run xacro xacro robot.urdf.xacro > test.urdf

# Validate the XML tree structures and generate diagram files
urdf_to_graphviz test.urdf mecanum_robot_layout

# Clean up test files
rm test.urdf mecanum_robot_layout.gv mecanum_robot_layout.pdf
```

If this completes without parsing errors, your Xacro syntax is 100% correct!
