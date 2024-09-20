# Changelog

## [v0.2.1] - 2024-09-20
- **oc.cobot.simple_scene**: Add this extension to demonstate pick and place cube.usd and cylinder.usd objects with PickPlaceController and RMPFlowController

## [v0.2.0] - 2024-09-13
- **Simulation**: Migrated isaac sim to version 4.1.0. Refactored sim.py and move all scene setup logic to extension oc.cobot.conveyor. Added Omnigraph to communicate between ROS2 and isaac sim scene workcell.usda.
- **Assets**: Resized conveyor length in workcell.usda. Changed basket assets to nvidia bin crate. 
- **ROS2**: Added ros_ws workspace and moveit task constructor to perform pick and place geometric in conveyor belt. 
- **oc.cobot.conveyor extension**: Renamed oc.cobot.yolo to oc.cobot.conveyor. Added omni.kit.tool.measure to sim.py required extensions.
- **Conveyor Module**: Fixed a conveyor belt randomly feeds object into opposite direction at extension startup. Added Light beam sensor (new feature in isaac sim v.4.1.0).
- **Launch Configuration**: Added config `spawn_pos_x_offset` in `config.toml` to move spawn object point in conveyor belt closer to robot.

## [v0.1.1] - 2024-07-25

### Added
- **Spawner Module**: Add reset button to remove all spawned item.
- **Code Snippets**: Add task vscode snippet to create task boiler plate code. 

## [v0.1.0] - 2024-07-05

### Added

- **Workcell Simulation Scene**: Created a workcell simulation scene in Nvidia Isaac Sim, consisting of the Franka Research 3 Robot, a conveyor, a workcell table, and baskets.
- **Conveyor Module**: Implemented the conveyor API and its logic to control the conveyor and simulate a sensor detecting devices.
- **Spawner Module**: Implemented object spawning logic supporting both geometric and real-life objects. Added logic to track whether object items are on or off the belt. User can random object item, object's spawn angle and position along y-axis of the belt. 
- **Control Panel UI**: Added a control panel to interact with the workcell.
- **Simplify Standalone Workflow Script**: Refactored the standalone workflow script into easy-to-read functions.
- **Launch Configuration**: Allows setting the spawner module configuration at launch time using `config.toml` or `config.local.toml` in your local environment.