# 🤖 Meet C.O.R.I.  
**Co-Operative Organizational Robotic Intelligence**  

C.O.R.I. is a modular, ROS 2–based home assistant robot designed to automate household tasks like laundry sorting. This personal robotics R&D project lays the groundwork for real-world autonomy and home integration.

[**🌐 Project Page → juptegraph.dev**](https://juptegraph.dev)  
<img src="assets/concept-art/cori_portfolio_view.png" width="500"/>

---

## 🧠 Vision Statement

> _"I want AI to do my laundry and dishes so I can do my art and writing, not to do my art and writing so I can do my laundry and dishes."_

That’s why I built CORI. The goal isn’t just automation—it’s improving quality of life. By offloading tedious domestic tasks to a collaborative personal assistant, the aim is to buy back time.

Built with ROS 2, Gazebo, and simulation tooling, CORI is both a functional robotics platform and a deeply personal project—a place to prototype assistive intelligence that learns, adapts, and helps people reclaim the hours they never meant to spend sorting their socks.

— *Johnathan Uptegraph*

---

## 🚀 Project Overview

This initial phase includes:

1. **ROS 2 Workspace Setup**  
   - Install ROS 2 Jazzy  
   - Configure a `colcon` workspace  
   - Install dependencies

2. **Basic Robot Model & Simulation**  
   - Build a URDF of a mobile base with a camera  
   - Simulate in Gazebo Harmonic + RViz2

3. **Color-Based Laundry Sorting**  
   - ROS 2 node using OpenCV with HSV filtering  
   - Detect and classify laundry colors in simulation

---

## ✨ Key Features

- **ROS 2 Native**  
  Modular architecture built with ROS 2 Jazzy for flexibility and scalability.

- **Gazebo Simulation**  
  Simulates a mobile robot with an RGB camera in Gazebo Harmonic and RViz2.

- **Color-Based Laundry Sorting**  
  Uses OpenCV (Python) and HSV filtering for real-time color classification.

- **Scalable Design**  
  Structured to support future modules for navigation, manipulation, and behavior planning.

---

## 📢 Latest Update – v1.0 (June 9, 2025)

<table>
<tr>
<td><img src="assets/gifs/openCV_Color_Sort_Test.gif" width="320"/></td>
<td>

Finished the first version of **CORI’s laundry color detection system**:

- Reads live webcam feed at ~60 FPS  
- Detects clothing items in-frame  
- Classifies by color:  
  - LIGHTS (e.g., white)  
  - DARKS (e.g., black)  
  - COLORS (e.g., red, blue, green)  
- Draws bounding boxes using the assigned color  
- Displays a live summary in the top-right showing:  
  - Total items  
  - Count per category

Built in Python with OpenCV and ROS 2.

**Next steps:** Enable scanning only when a new bin is presented or an item is picked. Vision is currently sourced from a basic webcam, but the long-term plan is for CORI to scan items using palm-mounted cameras before grasping.

Progress is solid. 📈  

> Considering a shift to Moondream for advanced clothing-type detection and low-level logic.

</td>
</tr>
</table>

---

## 🛠️ Tech Stack

| **System Layer**     | **Tools / Frameworks**           |
|----------------------|----------------------------------|
| **Operating System** | Ubuntu 22.04 + ROS 2 Jazzy       |
| **Simulation**       | Gazebo Harmonic, RViz2           |
| **Computer Vision**  | OpenCV (Python)                  |
| **Robot Control**    | rclpy, Launch Files              |

---

## 🔄 Development Roadmap

- [x] ROS 2 Jazzy install & workspace setup  
- [x] HSV-based laundry detection node  
- [ ] Mobile base URDF + Gazebo Harmonic model  
- [ ] (Future) Navigation + SLAM integration  
- [ ] (Future) Arm manipulation + gripper control  
- [ ] (Future) Task planning (Behavior Tree or FSM)

---

## 📜 License

MIT License © 2025 Johnathan Uptegraph  
See [LICENSE](LICENSE) for full terms.

> ⚠️ All visual assets, robot designs, and documentation are the intellectual property of Johnathan Uptegraph. This repository is for educational and personal portfolio use only.

---

- **Simulation Logs:** *Coming soon*

---

> **Not just a robot. A promise in motion.**
