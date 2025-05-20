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

## 🔄 Development Roadmap

- [ ] Initial ROS 2 workspace setup
- [ ] Basic robot URDF and Gazebo model
- [ ] HSV-based laundry detection module
- [ ] Mobile navigation in simulation
- [ ] Gripper control (sim)
- [ ] Real-world prototype integration
- [ ] Task planning framework (BT or FSM)

---

## 🤖 Vision Statement

Cori is more than a robot—it’s a step toward real-world, emotionally resonant automation. The goal is not just to build robots that move, but ones that help us live better.

> _“I built this to reclaim my free time, streamline the housekeeping process, and automate the most mundane parts of life.  
I once read a quote: ‘I want AI to do my laundry and dishes so that I can do art and writing, not for AI to do my art and writing so that I can do my laundry and dishes.’  
So I built CORI for exactly that reason.”_  
> — Johnathan Uptegraph

---

## 📜 License

MIT License

MIT License

Copyright (c) 2025 Johnathan Uptegraph

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


> ⚠️ All visual assets, robot designs, and documentation are the intellectual property of Johnathan Uptegraph and may not be reused without permission. This repository is for educational and personal portfolio use.

---


- Personal Site: [juptegraph.dev](https://juptegraph.dev)
- Simulation Logs: _TBD_
- Portfolio Project Page: _Coming Soon_

---

> _Built to function but designed to matter._
