# Micromouse Project
MTRN3100 – UNSW School of Mechanical and Manufacturing Engineering

## 📌 Overview
This project is our team’s solution for the **Micromouse Challenge** in MTRN3100.  
The objective is to design, build, and program an autonomous mobile robot that navigates through a 10x10 maze in the shortest time, using computer vision for path planning.

---



## 🎯 Objectives
- Build a robot that autonomously solves a maze.
- Demonstrate advanced control, sensing, and navigation.
- Use computer vision for maze mapping and path planning.
- Complete lab milestone tasks for assessment.

---

## ⚙️ Robot Specifications
- Powered by provided battery pack with appropriate fuse.
- Controlled by an **Arduino Nano** (all processing onboard, except computer vision for path generation).
- Must fit within a cylinder of **150mm diameter × 150mm height**.
- No permanent adhesives (e.g. epoxy, superglue).
- Modular design enabling easy component replacement.

---

## 🗺️ Maze Representation
- Grid: 10 × 10 cells, each 180mm × 180mm.
- Enclosed walls.
- Maze must be represented programmatically based on computer vision mapping.
- Start position: (row, column, direction) e.g. (1,6,N).
- Goal position: (row, column).

---

## ✅ Assessment Milestones

### 🚀 1. Barebones Movement (Week 4) – 3%
- Move autonomously ~200mm in a straight line on any platform.
- Goal: Demonstrate basic locomotion control.

---

### 🚗 2. Simple Driving (Week 8) – 7%
#### 2.1 Driving – 3%
- Use front LIDAR and a distance-based controller.
- Challenges:
  - Drive to 100mm ±5mm from a static wall.
  - Follow wall retreat/advance to maintain 100mm distance.
- One continuous program for all challenges.

#### 2.2 Turning – 3%
- Perform accurate 90° turns with LIDAR/IMU.
- Challenges:
  - Static 90° turn on start.
  - Reorient after being lifted and rotated.
- Accuracy: ±3° tolerance.

#### 2.3 Chaining Movements – 1%
- Execute a given 8-character movement string in a maze (e.g. "lfrfflfr").
  - `l` = left 90°
  - `r` = right 90°
  - `f` = forward one cell
- Must complete at least 4 moves for partial credit.

✅ **Branch requirement:**  
All code for this milestone must be pushed to the `simple driving` branch.

---

### 🧭 3. Micromouse Challenge (Week 12) – 20%
#### 3.1 Micromouse Race – 10%
- Path Planning (2%):
  - Generate a command sequence from a maze image using computer vision.
  - Must prove it’s not hard-coded.
- Maze Completion (8%):
  - Navigate autonomously from start to goal.
  - Graded on % completion and speed relative to other teams.

#### 3.2 Continuous Planning – 5%
- Navigate a 5×5 obstacle course within the maze.
- Generate occupancy map from image.
- Plan and follow a collision-free path.

#### 3.3 Autonomous Mapping – 5%
- Fully
