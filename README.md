# InMind Final Project – Autonomous TurtleBot3 with Behavior Trees

This repository contains the ROS 2 workspace and code used to build an autonomous TurtleBot3 navigation system using Behavior Trees and Navigation2.

---

## Setup Instructions

### 1. Clone the Repository

Clone this repository into the `src` directory of a new ROS 2 workspace:

```bash
mkdir -p ~/nav_tb3_ws/src
cd ~/nav_tb3_ws/src
git clone https://github.com/Tannous987/nav_tb3_ws.git
```

---

### 2. Import TurtleBot3 Dependencies via `vcs`

Use `vcs` (version control system) to pull all required TurtleBot3 packages as listed in `turtlebot3.repos`:

```bash
vcs import . < turtlebot3.repos
```

> **What is this?**  
> `vcs` is a command-line tool for downloading multiple repositories from a `.repos` file. It simplifies pulling all dependencies needed for a ROS 2 workspace.

---

### 3. Build the Workspace

Navigate to the root of your workspace and build all packages:

```bash
cd ~/nav_tb3_ws
colcon build --symlink-install
```

>**Note:** You might see some `stderr` output or warnings can be ignore they will not affect anything

---

### 4. Source the Workspace

After building, source the setup script to overlay the workspace:

```bash
source install/setup.bash
```

To make it persistent across terminals, add this to your `~/.bashrc`:

```bash
echo "source ~/nav_tb3_ws/install/setup.bash" >> ~/.bashrc
```

---

### 5. Export TurtleBot3 Model (for simulation)

Set the model and simulation paths:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/nav_tb3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle_pi
```

To make this permanent, add both lines to your `~/.bashrc`.

---

## Running the System

### 6. Launch the TurtleBot3 Simulation

```bash
ros2 launch tb3_sim turtlebot3_world.launch.py
```

---

### 7. Launch Navigation2 Stack (with AMCL + initial pose)

```bash
source install/setup.bash
ros2 launch tb3_sim nav2.launch.py
```

---

### 8. Launch the Autonomy Behavior Tree

```bash
source install/setup.bash
ros2 launch tb3_autonomy autonomy.launch.py
```

---

## About

This project was developed as part of the **InMind Final Project** using:
- ROS 2 Humble
- BehaviorTree.CPP
- TurtleBot3 Waffle Pi
- Gazebo simulation
