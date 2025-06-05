# Meet CORI
**C.O.R.I.** stands for **Co-operative Organizational Robotic Intelligence** — a modular, ROS 2–based home assistant robot designed to automate household tasks like laundry sorting. This is a personal robotics R&D project focused on building a foundation for real-world autonomy and home integration.

> *Built to function but designed to matter.*

> **Project Page:** [juptegraph.dev]([https://juptegraph.dev)
---

## 🤖 Vision Statement
CORI aims to streamline domestic chores—starting with laundry sorting—by leveraging ROS 2 and simulation tools. The goal is to create a flexible platform for learning and prototyping, paving the way for practical home robotics.

I read a quote once by Joanna Maciejewska where she stated,
> "I want AI to do my laundry and dishes so I can do my art and writing, not to do my art and writing so I can do my laundry and dishes."  

*So I built CORI for exactly that reason.*

*— Johnathan Uptegraph*

## 🚀 Project Overview
This initial phase focuses on:
1. **ROS 2 Workspace Setup**  
   - Install ROS 2 Jazzy  
   - Configure a `colcon` workspace  
   - Install dependencies

2. **Basic Robot Model & Simulation**  
   - Create a simple URDF for a mobile base with a camera  
   - Simulate it in Gazebo Harmonic

3. **Color-Based Laundry Sorting**  
   - Develop a ROS 2 node using OpenCV (HSV filtering)  
   - Detect and classify laundry colors in simulation

---

## 🧠 Key Features
- **ROS 2 Native**  
  Built using ROS 2 Jazzy for modularity and scalability.

- **Gazebo Simulation**  
  Simulate a mobile base equipped with a camera in Gazebo Harmonic + RViz2.

- **Color-Based Laundry Sorting**  
  Uses OpenCV (Python) with HSV filtering to classify laundry colors.

- **Foundation for Expansion**  
  Architecture designed to support future modules like navigation, manipulation, and task planning.

---

![CORI prototype](assets/concept-art/cori-main-concept-art.png)

---

## 🛠️ Tech Stack
| **System**      | **Tools / Frameworks**           |
| --------------- | --------------------------------- |
| **O.S.**        | Ubuntu 22.04 / ROS 2 Jazzy       |
| **Simulation**  | Gazebo Harmonic / RViz2           |
| **Computer Vision** | OpenCV Python                 |
| **Robot Control** | ROS 2 Nodes / Launch Files     |

---

## 🔄 Development Roadmap
- [x] ROS 2 Jazzy install & workspace setup  
- [ ] Basic robot URDF & Gazebo Harmonic model  
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

- **Simulation Logs:** *TBD*  


> *Built to function but designed to matter.*
