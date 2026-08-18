
---

# Mecanum Parcel Acquisition Pipeline: Developer Onboarding Guide

Welcome to the parcel acquisition team. This document covers the architecture, mathematics, and software design for our holonomic docking system.

We are operating a mecanum-wheeled mobile manipulator in a structured warehouse environment. Our goal is to locate, align with, and prepare to grasp 10x10x10 cm parcels using visual servoing, compensating for the inherent odometry drift of mecanum wheels.

---

## 1. Physical Environment & Spatial Geometry

It is critical to understand the difference between the Global Map frame and the Robot's Local frame during this operation.

### The Arena Layout (Global Map Frame)

* **Parcels:** $10 \times 10 \times 10$ cm cubes, spaced 10 cm apart.
* **Arrangement:** Parcels are placed in two straight lines running parallel to the **global Y-axis**.
* **The Aisle:** The two lines are separated by a 2-meter gap (along the global X-axis).
* **Tagging:** ArUco tags are placed on **all 6 faces** of every cube to remove human placement error.

### The Robot Hardware & Alignment (Local `base_link` Frame)

* **Drive Base:** 4-wheel Mecanum (Holonomic kinematics allowing independent X, Y, and Yaw motion).
* **Camera:** Monocular RGB (e.g., Pi Camera V2), mounted on the **LEFT side** of the robot (pointing in the $+Y_{base}$ direction).
* **Manipulator/Gripper:** Mounted on the **FRONT** of the robot (pointing in the $+X_{base}$ direction).
* **The Scanning Path:** Nav2 will drive the robot down the center of the aisle (moving along the global Y-axis). Because the camera is on the left, driving forward scans the **Left Line** of parcels. *(Note: To scan the Right Line, the robot must either drive the aisle in reverse or rotate 180 degrees).*

---

## 2. Core Algorithmic Concepts

Do not attempt to write control loops in the camera's optical frame. All visual servoing math must be executed in `base_link` (the center of the robot chassis) via ROS 2 `tf2`.

### The 6-Face ArUco Problem (Targeting the Cube, not the Tag)

Because the cube has tags on all faces, a high-mounted camera will often see the **Top** tag and the **Front** tag (facing the aisle) simultaneously. If you feed raw ArUco poses to the PID loop, the target will violently jump back and forth by 5 cm (the distance between the top surface and the front surface).

**The Fix:** We intercept the ArUco TF frames and project them to the absolute center of the cube.

1. Transform the detected tag into `base_link`.
2. Look at the tag's Z-axis (normal vector).
3. If it points UP (aligned with global Z), it is the Top tag. Offset the target Z down by 5 cm.
4. If it points HORIZONTALLY (towards the robot), it is the Aisle-facing tag. Offset the target's distance away from the robot by 5 cm.
5. Feed this newly calculated "Cube Center" to the PID controllers.

### Holonomic PID Control

Once the cube center is known in `base_link`, we use three decoupled Proportional (P) controllers to align the robot's left side with the parcel.

* **X-Axis (Forward/Backward):** Centers the robot chassis with the parcel. $Target_X = 0.0$.
* **Y-Axis (Strafe Left/Right):** Maintains a safe standoff distance so the robot doesn't hit the box while scanning. Let's assume a 20cm standoff to the left: $Target_Y = 0.2$ meters.
* **Yaw (Rotation):** Keeps the robot perfectly parallel to the parcel line. $Target_\theta = -90^\circ$ ($-\frac{\pi}{2}$ rad) relative to the tag face.

$$v_x = K_{p,x} \cdot (Target_X - P_x)$$

$$v_y = K_{p,y} \cdot (Target_Y - P_y)$$

$$\omega_z = K_{p,\theta} \cdot (Target_\theta - \theta_{aruco})$$

### Proportional Desaturation (Anti-Clipping)

If the PID loops request more velocity than the motor drivers can output, the motor controller will "clip" the fastest wheel. This destroys the holonomic vector, causing the robot to curve uncontrollably instead of driving straight.

**The Fix:** Calculate $v_x, v_y, \omega_z$. Find the maximum absolute value. If it exceeds our `max_vel` limit (e.g., $0.15$ m/s), scale all three velocities down by the exact same ratio.

---

## 3. Software Architecture (Development Requirements)

To prevent resource conflicts and keep the system modular, we separate the logic into **Three ROS 2 Actions** hosted inside **One Main Node**. This allows the actions to share a single `tf2` Buffer and a single `/cmd_vel` publisher.

### 1. The Action Interfaces (`custom_interfaces/action/`)

You will need to compile three separate Action definitions:

* `SearchParcel.action`: Takes an ArUco ID. Returns success when the tag enters the camera frame.
* `ServoParcel.action`: Takes an ArUco ID. Returns success when X, Y, and Yaw errors are below a target threshold (e.g., $< 1$ cm).
* `RotateBlind.action`: Takes an angle in radians. Returns success after executing the open-loop rotation.

### 2. The Logic Modules (The "Brains")

You will write three separate Python classes, one for each phase. They do not inherit from `rclpy.node.Node`; instead, they are passed the ROS 2 node, the `tf2` buffer, and the velocity publisher as constructor arguments.

* `search_action.py`: Commands a slow positive `linear.x` velocity to drive parallel to the line until `tf_buffer.lookup_transform` succeeds.
* `servo_action.py`: The core holonomic PID loop. Runs at ~20Hz. Stops publishing and fails gracefully if the tag is lost for more than 0.5 seconds.
* `rotate_action.py`: Commands a fixed `angular.z` velocity for a calculated duration to achieve the 90-degree turn toward the parcel.

### 3. The Action Server Node (`acquisition_main.py`)

This is the executable entry point.

1. Initializes `rclpy`.
2. Creates the `AcquisitionManager` node.
3. Instantiates the `tf2_ros.Buffer` and `TransformListener`.
4. Instantiates the `/cmd_vel` publisher.
5. Passes these objects into the three action server classes.
6. Spins using a `MultiThreadedExecutor` to allow concurrent action handling.

### 4. The Action Client (The Sequencer)

Finally, you will write a sequencer script (or configure a Behavior Tree) that acts as the higher-level client. When Nav2 signals that it has arrived in the aisle, this client:

1. Calls `SearchParcel(id=42)` $\rightarrow$ Awaits Success.
2. Calls `ServoParcel(id=42)` $\rightarrow$ Awaits Success.
3. Calls `RotateBlind(angle=-1.57)` $\rightarrow$ Awaits Success (Rotating -90 degrees points the front gripper at the left line).
4. Triggers the gripping/manipulator sequence.

---

## 4. Hardware Warnings & Best Practices

* **Rolling Shutter Blur:** If using a cheap monocular camera (like a Pi Cam), mecanum wheel vibrations will cause "jello" motion blur. You MUST run the camera at a low resolution (e.g., 640x480), force a fast shutter speed ($< 1/100$s), and keep robot search/servo velocities under $0.15$ m/s.
* **Marker Size Calibration:** Measure the printed black square of the ArUco tag with digital calipers. If your ROS 2 configuration file says the tag is 50mm, but the printer scaled it to 48mm, your robot's distance calculations (Y-axis standoff) will be permanently incorrect and the robot will crash into the parcel.
* **TF Tree Integrity:** The transformation from `base_link` to `camera_link` must be perfectly calibrated using a `static_transform_publisher`. Do not guess the pitch angle or translation offsets of your camera mount.