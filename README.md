# 🤖 Robot Waiter – TurtleBot4 | ROS2 Project

*A full Human–Robot Interaction + Navigation + Perception robot waiter system built on TurtleBot4.*

---

## 📌 Overview

This project implements an **autonomous robot waiter** capable of:

* Navigating a restaurant using **Nav2**
* Detecting tables using **AprilTags & Perception**
* Taking customer orders using **LLM-based HRI**
* Handling wake-word → STT → NLU → Order confirmation
* Communicating with the kitchen through a **Kitchen DB Agent**
* Managing safety, docking, thermal, and power systems

This repository is organized using **GitHub Project boards**, **Milestones**, and a structured **Issue → Branch → PR** workflow.

---

## 🚀 Features

### 🟣 Human–Robot Interaction (HRI)

* Wake-word detection
* Speech-to-text using OpenAI/Vosk
* LLM-based NLU & Order extraction
* Dialog management
* OLED / LED / RGB user feedback

### 🔵 Perception

* Camera calibration
* AprilTag-based table localization
* Optional person detection
* TF alignment camera ↔ base ↔ map

### 🟢 Navigation

* SLAM map creation
* Nav2 configuration + tuning
* Waypoint database
* Safe movement with obstacle avoidance

### 🔴 Safety & Health Monitoring

* Hazard watchdog
* Cliff/edge protection
* Battery monitoring
* Autonomous docking
* Thermal management

### 🟣 Core Orchestration

* Central FSM to run the full waiter workflow
* Order manager
* Kitchen agent + DB bridge
* Mission status + event logging

---

## 🏗 System Architecture

*(Insert your architecture diagram or Mermaid graph here)*

```
<architecture diagram placeholder>
```

---

## 🛠 Tech Stack

| Component     | Technology                               |
| ------------- | ---------------------------------------- |
| Robot         | TurtleBot4 / Raspberry Pi 4              |
| OS            | Ubuntu 22.04                             |
| Framework     | ROS2 Humble                              |
| Navigation    | Nav2                                     |
| Perception    | OpenCV, AprilTags                        |
| HRI           | OpenAI APIs, Audio pipeline              |
| Orchestration | Custom FSM                               |
| Safety        | TurtleBot4 onboard sensors               |
| Deployment    | GitHub, Project Boards, Feature Branches |

---

# 🔧 Installation & Setup

### 1. Clone the repository

```
git clone https://github.com/SenithuDampegama/Robot_waiter_project.git
cd Robot_waiter_project
```

### 2. Install dependencies

```
rosdep install --from-paths src -y --ignore-src
```

### 3. Build

```
colcon build --symlink-install
source install/setup.bash
```

### 4. Run the Orchestrator

```
ros2 launch waiter_bringup orchestrator.launch.py
```

---

# 🗂 Repository Structure

```
Robot_waiter_project/
│── perception/
│── navigation/
│── hri/
│── safety/
│── core/
│── bringup/
│── docs/
└── README.md
```

---

# 🧩 Team Roles

### 🟣 Senithu

Perception, LIDAR, Sensor Fusion, Core FSM, Architecture

### 🟢 Fazeel

Navigation, Waypoints DB, SLAM, Nav2 tuning

### 🟠 Abby

HRI, Speech pipeline, LLM integration, Dialog Manager, UI

### 🔴 Saleh

Safety, Docking, Battery Manager, Thermal systems

---

# 🏷 Project Management

We use:

* **Milestones** (Perception, Navigation, HRI, Safety, Core, Submission)
* **Issues** (30+ tasks)
* **GitHub Project Board** (Roadmap + Iterations)
* **Branching Strategy**

---

# 🔒 Contribution Rules (MANDATORY)

**Do NOT push to `main`.** Only merge through PRs.

### Branch naming:

```
feature/<desc>
fix/<desc>
docs/<desc>
test/<desc>
```

### Each Pull Request must include:

* Clear description
* Linked Issue
* Testing proof
* Screenshots/logs if needed

Full rules here: `/Workflow Rules`

---

# ✔ Testing

Each module includes:

* Bag file validation
* Navigation test videos
* Audio pipeline tests
* Safety interrupt logs

---

# 📄 Documentation

Final submission includes:

* Full Engineering Report
* Architecture diagrams
* Test results
* Source code
* Demo videos

---

# 📆 Deadline

📌 **Final project submission: 11 December 2025**

---

# 🧠 Contact

Team Robot Waiter:

* Senithu
* Fazeel
* Abby
* Saleh

