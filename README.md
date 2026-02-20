# EconomicalPlanner

This repository contains the source code for the paper:

**"Economical Motion Planning for On-Road Autonomous Driving with Distance-Sensitive Spatio-Temporal Resolutions"**

## Dependencies

- ROS (tested on ROS Noetic)
- CasADi
- IPOPT
- HSL

## Installation

1. Create a catkin workspace:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Clone this repository:

```bash
git clone https://github.com/YueshuoSun/EconomicalPlanner.git
```

3. Copy the packages to the workspace:

```bash
cp -r EconomicalPlanner/Planner ~/catkin_ws/src/
cp -r EconomicalPlanner/Util/* ~/catkin_ws/src/
```

4. Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
```

5. Source the workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

1. Launch the planner with RViz visualization:

```bash
roslaunch planner_node planning.launch
```

2. In RViz, use the **2D Nav Goal** tool (green arrow) to set a goal pose on the track in the **counter-clockwise direction**. The vehicle will start moving from its current position towards the goal.

## License

GPL-3.0


## Citation

Copyright (C) 2026 Bai Li and Yueshuo Sun. This source code is associated with the following publication. If you use this code in your research or build upon it in any new publications, please cite our paper:

> Yueshuo Sun, Bai Li, "Economical Motion Planning for On-Road Autonomous Driving with Distance-Sensitive Spatio-Temporal Resolutions," *Machines*, vol. 14, no. 2, article 200, 2026. DOI: [https://doi.org/10.3390/machines14020200](https://doi.org/10.3390/machines14020200)
