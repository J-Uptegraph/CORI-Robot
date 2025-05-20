# CORI: The Chore Robot

**C.O.R.I.** stands for **Co-operative Organizational Robotic Intelligence** — a modular, ROS 2-based home assistant robot engineered to help automate household tasks like cleaning, sorting, and organization. This is a personal robotics R&D project focused on real-world autonomy, home integration, and emotionally intelligent design.

> Built with ROS 2. Designed for humans. Powered by persistence.

---

## 🚀 Project Overview

**CORI** is a home-focused robotic system under active development by Johnathan Uptegraph. It is designed to:
- Streamline **domestic chores**
- Integrate with **smart home environments**
- Utilize **modular ROS 2 packages** for navigation, manipulation, and task planning
- Serve as a **platform for learning, simulation, and real-world prototyping**

The robot’s first goal is to autonomously **sort laundry by color**, navigating a mapped space and using a simple computer vision pipeline to detect and classify clothing.

---

## 🧠 Key Features

- **ROS 2 Native**  
  Built from the ground up using ROS 2 for modularity, flexibility, and future scalability.

- **Gazebo Simulation Support**  
  Full simulation of sensors, actuators, and apartment navigation in Gazebo.

- **Color-Based Laundry Sorting**  
  Initial task module uses HSV filtering and/or ML-based classification for identifying laundry by color.

- **Mobile Manipulation Architecture**  
  Designed for integration with mobile base platforms and simple two-finger grippers.

- **Expandable Architecture**  
  Future modules planned include:
  - Room cleaning & object pickup
  - Smart shelf sorting
  - Pet monitoring or basic verbal interaction

---

## 🛠️ Tech Stack

| System       | Tools / Frameworks                       |
|--------------|-------------------------------------------|
| OS           | Ubuntu 22.04 / ROS 2 Humble               |
| Simulation   | Gazebo / RViz                             |
| Perception   | OpenCV / depth image processing           |
| Control      | Python ROS 2 nodes, launch files          |
| Navigation   | SLAM Toolbox / Nav2 stack (planned)       |
| Hardware     | TBD (planned: depth cam, LiDAR, Jetson)   |

---

## 📁 Project Structure

cori_chore_robot/
├── cori_description/ # URDF, meshes, and visuals
├── cori_bringup/ # Launch files for sim/hardware
├── cori_navigation/ # Nav2 or SLAM-based navigation
├── cori_manipulation/ # Gripper and task logic
├── cori_vision/ # Color sorting & perception
├── cori_msgs/ # Custom ROS messages & services
└── README.md # This file

yaml
Copy
Edit

---

## 🔄 Development Roadmap

- [x] Initial ROS 2 workspace setup
- [x] Basic robot URDF and Gazebo model
- [ ] HSV-based laundry detection module
- [ ] Mobile navigation in simulation
- [ ] Gripper control (sim)
- [ ] Real-world prototype integration
- [ ] Task planning framework (BT or FSM)

---

## 🤖 Vision Statement

Cori is more than a robot—it’s a step toward real-world, emotionally resonant automation. The goal is not just to build robots that move, but ones that help us live better.

> _“I built this to reclaim my time, reduce my overwhelm, and make space for the life I want to live. Cori is a tool for autonomy—both mine, and hers.”_  
> — Johnathan Uptegraph

---

## 📜 License

MIT License
