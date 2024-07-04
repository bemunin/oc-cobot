# OmniCraft Object Detection Cobot

A pre-configured simulation scene for experimenting with the robot arm using various object detection algorithms, powered by NVIDIA Isaac Sim.

[ TODO: Add video of current version v0.1.0 ]

The goals of this project are to:

- Enable developers to quickly experiment with object detection algorithms without the need to develop a simulation scene.
- Provide an example of directory structure and how to create a NVIDIA Isaac Sim scene from scratch

## Getting Started

## Project Directory Guide
- `sim.py`: Main entry point. We use standalone workflow script to launch the NVIDIA Isaac Sim app and run our `oc.ex.yolo_cobot` extension, which contains the application logic of this example.  
- `assets/`: Contains all USD models composed in the scene.
- `docs/`: Stores the project's files and documentation.
- `exts/`: Project's Isaac Sim extension. You can add new extensions to suit your work here.
- `launch/`: Contains sub-functions used in `sim.py` to launch the simulation application.
- `utils/`: Shared utility functions that are used frequently in the project.


## Release Notes

### v0.1.0
- **Workcell Simulation Scene**: Created a workcell simulation scene in Nvidia Isaac Sim, consisting of the Franka Research 3 Robot, a conveyor, a workcell table, and baskets.
- **Conveyor Module**: Implemented the conveyor API and its logic to control the conveyor and simulate a sensor detecting devices.
- **Spawner Module**: Implemented object spawning logic supporting both geometric and real-life objects. Added logic to track whether object items are on or off the belt. User can random object item, object's spawn angle and position along y-axis of the belt. 
- **Control Panel UI**: Added a control panel to interact with the workcell.
- **Simplify Standalone Workflow Script**: Refactored the standalone workflow script into easy-to-read functions.