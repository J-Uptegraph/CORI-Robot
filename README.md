# Meet CORI The Chore Robot

**C.O.R.I.** stands for **Co-operative Organizational Robotic Intelligence** — a modular, ROS 2–based home assistant robot designed to automate household tasks like laundry sorting. This is a personal robotics R&D project focused on building a foundation for real-world autonomy and home integration.

> _Built to function but designed to matter._

---

## 🤖 Vision Statement

CORI aims to streamline domestic chores—starting with laundry sorting—by leveraging ROS 2 and simulation tools. The goal is to create a flexible platform for learning and prototyping, paving the way for practical home robotics.

I read a quote once by Joanna Maciejewska where she stated,

> “I want AI to do my laundry and dishes so I can do my art and writing, not to do my art and writing so I can do my laundry and dishes.”  

_So I built CORI for exactly that reason._

_— Johnathan Uptegraph_

## 🚀 Project Overview

This initial phase focuses on:

1. **ROS 2 Workspace Setup**  
   - Install ROS 2 Humble  
   - Configure a `colcon` workspace  
   - Install dependencies

2. **Basic Robot Model & Simulation**  
   - Create a simple URDF for a mobile base with a camera  
   - Simulate it in Gazebo

3. **Color-Based Laundry Sorting**  
   - Develop a ROS 2 node using OpenCV (HSV filtering)  
   - Detect and classify laundry colors in simulation

---

## 🧠 Key Features

- **ROS 2 Native**  
  Built using ROS 2 Humble for modularity and scalability.

- **Gazebo Simulation**  
  Simulate a mobile base equipped with a camera in Gazebo + RViz.

- **Color-Based Laundry Sorting**  
  Uses OpenCV (Python) with HSV filtering to classify laundry colors.

- **Foundation for Expansion**  
  Architecture designed to support future modules like navigation, manipulation, and task planning.

---

![CORI prototype](assets/concept-art/cori-main-concept-art.png)

---
stateDiagram-v2
    [*] --> START
    
    START --> GET_HAMPER
    
    GET_HAMPER --> SORT_CLOTHES
    GET_HAMPER --> ERROR : Can't find hamper
    
    SORT_CLOTHES --> LIGHTS_BIN : Light colored clothes
    SORT_CLOTHES --> DARKS_BIN : Dark colored clothes
    SORT_CLOTHES --> MORE_CLOTHES : Check if more to sort
    
    LIGHTS_BIN --> MORE_CLOTHES
    DARKS_BIN --> MORE_CLOTHES
    
    MORE_CLOTHES --> SORT_CLOTHES : Yes, more clothes
    MORE_CLOTHES --> CHECK_MACHINE : No more clothes
    
    CHECK_MACHINE --> LOAD_CLOTHES : Machine is free
    CHECK_MACHINE --> WAIT : Machine is busy
    
    WAIT --> CHECK_MACHINE : Check again later
    
    LOAD_CLOTHES --> ADD_SOAP
    LOAD_CLOTHES --> ERROR : Machine is broken
    
    ADD_SOAP --> START_WASH
    ADD_SOAP --> ERROR : No soap available
    
    START_WASH --> WASHING
    
    WASHING --> DONE : Wash complete
    WASHING --> ERROR : Machine error
    
    DONE --> MORE_LOADS : Check for more bins
    
    MORE_LOADS --> LOAD_CLOTHES : Yes, more to wash
    MORE_LOADS --> FINISHED : All done!
    
    ERROR --> START : Try again
    ERROR --> FINISHED : Give up
    
    FINISHED --> [*]
---

## 🛠️ Tech Stack

| **System**   | **Tools / Frameworks**            |
| ------------ | --------------------------------- |
| **OS**       | Ubuntu 22.04 / ROS 2 Humble       |
| **Simulation** | Gazebo Classic / RViz           |
| **Perception** | OpenCV (Python) for HSV filtering |
| **Control**  | Python ROS 2 nodes & launch files |

---

## 🔄 Development Roadmap

- [x] ROS 2 Humble install & workspace setup  
- [ ] Basic robot URDF & Gazebo model  
- [ ] HSV-based laundry detection node  
- [ ] (Future) Navigation & SLAM integration  
- [ ] (Future) Manipulation & gripper control  
- [ ] (Future) Task planning (BT or FSM)

---

## 📜 License

MIT License © 2025 Johnathan Uptegraph  
See [LICENSE](LICENSE) for full text.

---

> ⚠️ All visual assets, robot designs, and documentation are the intellectual property of Johnathan Uptegraph and may not be reused without permission. This repository is for educational and personal-portfolio use.

---

- **Personal Site:** [juptegraph.dev](https://juptegraph.dev)  
- **Simulation Logs:** _TBD_  
- **Portfolio Project Page:** _Coming Soon_

> _Built to function but designed to matter._
