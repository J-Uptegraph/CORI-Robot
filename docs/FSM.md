## 🧩 OVERVIEW OF THE FINITE STATE MACHINE FOR C.O.R.I.
> “Built to function, designed to matter.”

CORI isn’t just a task executor—it’s a step toward reclaiming your time. Built in ROS 2 with expandable logic and vision-based sorting, this finite state machine (FSM) serves as the heart of CORI’s decision-making. Every state is a small act of care. Every transition is a handoff between intention and action. Whether it’s identifying laundry colors or loading the washer, CORI's architecture reflects a belief:

Robots should do the chores, so humans can do what actually matters.

These are the first tasks—a starting point for something much bigger. Assistive. Modular. Human-centered.



## 🔄 FSM States & Transitions

| **State**            | **Description**                                                    | **Transition Conditions**                                      |
|----------------------|--------------------------------------------------------------------|----------------------------------------------------------------|
| `IDLE`               | Standby mode, waiting for a task                                   | External signal (e.g. user command or schedule)                |
| `WAKE`               | System check, boot sensors                                         | All diagnostics passed                                         |
| `NAV_TO_HAMPER`      | Navigate to laundry hamper                                         | Successful localization and goal reached                       |
| `ISOLATE_HAMPER`     | Use object segmentation to identify hamper in vision frame         | Hamper confirmed within grasp range                            |
| `PICK_HAMPER`        | Use manipulator to pick up hamper                                  | Grasp success confirmed via feedback                           |
| `NAV_TO_WASH`        | Navigate to washer                                                 | Goal reached, obstacle-free path                               |
| `PLACE_HAMPER`       | Place hamper near washer                                           | Placement validated by pose estimation                         |
| `OPEN_WASHER`        | Open washer door (if robotic manipulation is supported)            | Door state = open                                              |
| `SORT_LAUNDRY`       | Pick clothes from hamper one-by-one                                | Until hamper is empty                                          |
| `SELECT_BIN`         | Run color detection → assign to dark/white bin                     | Confidence threshold met (e.g. >90%)                           |
| `FILL_BIN`           | Place item into appropriate bin                                    | Repeat until full                                              |
| `LOAD_WASHER`        | Load sorted bin (e.g. darks first) into washer                     | All bin contents transferred                                   |
| `ADD_DETERGENT`      | Activate servo or dispenser module for detergent                   | Flow detected or timed                                         |
| `START_WASH`         | Trigger washer via smart relay, I/O, or protocol                   | Wash cycle initiated                                           |
| `WAIT_FOR_WASH`      | Monitor wash cycle completion (via API or timer)                   | Machine complete signal or elapsed time                        |
| `CHECK_COMPLETION`   | Verify clothes are clean (optional camera check or trust system)   | Proceed if dry cycle requested or continue to unload           |
| `UNLOAD_WASH`        | Transfer clothes to drying area or return to user                  | End of cycle                                                   |
| `END`                | FSM completes task → Return to `IDLE`                              | Await new command                                              |

---

## 🧠 Decision Logic Details

### 🎨 Color Sorting Algorithm
- Convert RGB image of cloth to **HSV**
- Mask for **white/light** and **dark** color ranges
- Return `color_label` and `confidence_score`

### 🧭 Navigation Stack
- ROS 2 **Nav2** framework with **LiDAR + IMU** fused via **EKF**
- Apartment map loaded from saved **SLAM**
- Dynamic obstacle avoidance (e.g. **pets**, **furniture**, **humans**)

### 🤖 Arm Control
- Controlled using **MoveIt2** or **joint trajectory controllers**
- Predefined **grasp poses** for hamper and clothing
- Grasp success verified via **tactile or joint feedback**
