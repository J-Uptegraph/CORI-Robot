## 🤖 CORI Robotics — Core Package Structure (MVP)

This document outlines the foundational ROS 2 packages that power CORI’s MVP: a laundry-sorting, household assistant robot built for real autonomy in constrained domestic spaces.

---

## 1. `cori_interfaces/`  
Custom messages, services, and actions for FSM coordination and robot communication.

```bash
cori_interfaces/
├── msg/
│   ├── SystemState.msg
│   ├── LaundryItem.msg
│   ├── BinStatus.msg
│   └── TaskStatus.msg
├── srv/
│   ├── StartTask.srv
│   ├── EmergencyStop.srv
│   └── GetSystemHealth.srv
├── action/
│   ├── NavigateToGoal.action
│   ├── PickObject.action
│   ├── SortLaundry.action
│   └── LoadWasher.action
├── CMakeLists.txt
└── package.xml
```

---

## 2. `cori_state_machine/`  
Main task logic controller. Implements CORI’s finite state machine and transitions.

```bash
cori_state_machine/
├── cori_state_machine/
│   ├── fsm_node.py
│   ├── states/
│   │   ├── idle_state.py
│   │   ├── wake_state.py
│   │   ├── navigation_states.py
│   │   ├── manipulation_states.py
│   │   ├── perception_states.py
│   │   └── wash_states.py
│   ├── transitions/
│   │   └── state_transitions.py
│   └── utils/
│       └── fsm_utils.py
├── config/
│   └── fsm_config.yaml
├── launch/
│   └── state_machine.launch.py
├── test/
│   ├── test_fsm.py
│   └── test_states.py
├── CMakeLists.txt
└── package.xml
```

---

## 3. `cori_perception/`  
Handles real-time object detection and color classification using OpenCV and pretrained models.

```bash
cori_perception/
├── cori_perception/
│   ├── object_detection/
│   │   ├── hamper_detector.py
│   │   ├── clothes_detector.py
│   │   └── color_classifier.py
│   ├── segmentation/
│   │   └── instance_segmentation.py
│   ├── pose_estimation/
│   │   └── object_pose_estimator.py
│   └── utils/
│       └── image_utils.py
├── models/
│   ├── yolo_clothes.pt
│   └── color_classifier.pkl
├── config/
│   └── camera_params.yaml
├── launch/
│   └── perception.launch.py
└── test/
    └── test_detection.py
```

---

## 4. `cori_manipulation/`  
Arm and gripper control, including grasping logic for clothing items and hamper handling.

```bash
cori_manipulation/
├── cori_manipulation/
│   ├── arm_controller.py
│   ├── gripper_controller.py
│   └── trajectory_planner.py
├── config/
│   └── moveit_config/
│       ├── joint_limits.yaml
│       └── kinematics.yaml
├── launch/
│   └── manipulation.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## 5. `cori_bringup/`  
Brings all subsystems online for either hardware deployment or Gazebo simulation.

```bash
cori_bringup/
├── launch/
│   ├── cori_robot.launch.py        # Launch real system
│   ├── cori_simulation.launch.py   # Launch Gazebo sim
│   └── full_system.launch.py
├── config/
│   └── robot_config.yaml
├── CMakeLists.txt
└── package.xml
```

---
