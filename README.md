# Robot arm

## Hardware & Environment
- **OS / Middleware:** Ubuntu 20.04 LTS, ROS 2 Foxy
- **Actuators:** 1× Dynamixel XM540-W150-T (shoulder), 4× Dynamixel XM430-W350-T (IDs `1–5`), connected via U2D2 USB-to-TTL (`/dev/ttyUSB0`, `57600` baud, Protocol 2.0)
- **Sensor:** Intel RealSense D435i mounted horizontally inside the robot base shell (`640×480` aligned RGB-D)

---

## Repository Structure

```text
ada526_robot_arm/
├── ada526_robot_arm/
│   ├── img_processor.py        # CIELAB red/green blob segmentation, depth lookup, 3D deprojection & T_R,C
│   ├── kinematics_core.py      # 5-DOF DH model, ikine_LM solver, table-clearance check & Challenge 2 FSM
│   ├── robot_controller.py     # Dynamixel SDK Protocol 2.0 driver using GroupSyncWrite & motion profiles
│   └── cordinate.py            # CLI node converting baseplate grid labels (e.g. A,1) to /goal_point
├── launch/
│   ├── cdr_bringup.launch.py           # Full closed-loop stack (camera + TF + vision + IK + controller)
│   ├── cdr_camera_only.launch.py       # Camera, static TF, and vision node only (no motor power)
│   └── cdr_bringup_no_camera.launch.py # Kinematics and motor controller only (Challenges 1 & 3)
├── debugCodes/
│   ├── vizGoal.py              # Live 3D matplotlib plotter for /goal_point coordinates
│   └── vizRobot.py             # Live 3D kinematic pose visualizer subscribed to /joint_states
├── package.xml
├── setup.py
└── setup.cfg
