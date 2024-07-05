# OmniCraft Cobot Project

> 📌 This project is being developed in an Ubuntu environment and is not yet guaranteed to work with Windows.

A simulation scene built with NVIDIA Isaac Sim for experimenting with a robot arm (cobot) and the YOLO object detection model.

![Scene from extension oc.cobot.yolo](docs/images/oc_cobot_yolo_scene.png "Scene from extension oc.cobot.yolo")

The goal of this project is to provide an example of how to create an NVIDIA Isaac Sim scene from scratch, control a robot manipulator, and use it for experimenting with computer vision algorithms.


## Roadmap
- ✅ Create the entire scene, implement the object spawner module, and simulate conveyor functionality.
- Implement a robot pick-and-place skill using the NVIDIA Isaac Sim Core API.
- Apply YOLO object detection.

## Changelog
For detailed information on our current features, updates and changes, see the [Changelog](./CHANGELOG.md).


## Project Files and Directories
- `sim.py`: Main entry point, a standalone workflow script to launch the NVIDIA Isaac Sim app and run the `oc.cobot.yolo` extension.  
- `assets/`: Contains all USD models composed in the scene.
- `docs/`: Stores the project's files and documentation.
- `exts/`: Project's Isaac Sim extension. You can add new extensions to suit your work here.
  - `oc.cobot.yolo`: An extension that contains the application logic of this example.  
- `launch/`: Contains sub-functions used in `sim.py` to launch the simulation application.
- `utils/`: Shared utility functions that are used frequently in the project.

## Getting Started

1. Install Conda by following the [official Conda installation guide for Linux](https://conda.io/projects/conda/en/latest/user-guide/install/linux.html). Use the **Anaconda Distribution installer for Linux**.

2. Clone the project and change the directory to the `oc-cobot` root:

    ```sh
    git clone https://github.com/bemunin/oc-cobot.git
    cd oc-cobot
    ```

3. Make a copy of `environment.yml` and rename it to `environment.local.yml`.

4. Inside the `environment.local.yml` file, change `<user>` in the `prefix:` key to match your Anaconda path.

5. At the root of the project directory, run this command to create the `oc_cobot` Conda environment:

    ```sh
    conda env create -f environment.local.yml
    ```

6. Activate the `oc_cobot` Conda environment:

    ```sh
    conda activate oc_cobot
    ```

7. Run the `sim.py` script to start the simulation:

    ```sh
    python sim.py
    ```

    ⚠️ **Note:**
    - The first time it runs, it might take some time to load the program and scene as NVIDIA Isaac Sim has not yet cached the data.
    - There will be a prompt from NVIDIA asking you to agree with the NVIDIA License. Type `Y` and press Enter to accept. You only need to do this once.
      ![NVIDIA license agreement](docs/images/eula_license_prompt.png "NVIDIA license agreement")

8. After the simulation starts, you can interact with the scene via the control panel on the top left.

   ![Control panel image](docs/images/control_panel.png "Control panel image")
