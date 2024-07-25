# Changelog

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