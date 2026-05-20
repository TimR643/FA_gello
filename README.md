# GELLO Panda LeRobot Teleoperation

This repository contains my modified GELLO setup for teleoperating a Franka Emika Panda robot and recording demonstrations for imitation learning with LeRobot.

The project is based on the original GELLO software and was extended for my own Franka Panda, Polymetis, camera recording, and LeRobot dataset conversion pipeline.

## License and Attribution

This repository is based on the original GELLO software by Philipp Wu.

Original project: https://github.com/wuphilipp/gello_software

The original copyright and license notice are preserved in the `LICENSE` file.

This version contains project-specific modifications for Franka Emika Panda teleoperation, Polymetis-based robot control, RealSense/FRAMOS camera integration, and LeRobot dataset conversion.



## Overview

The goal of this project is to use a GELLO teleoperation device to control a Franka Emika Panda robot and record demonstration data that can later be used for imitation learning.

This repository includes modifications for:

- Franka Emika Panda teleoperation
- Polymetis-based robot control
- GELLO joint calibration and control
- RealSense / FRAMOS camera integration
- multi-camera recording support
- conversion of recorded GELLO episodes into the LeRobot dataset format
- custom start and stop scripts for the local setup

## System Setup

The current setup is mainly built around:

- Franka Emika Panda robot
- GELLO teleoperation device
- Polymetis robot interface
- RealSense / FRAMOS camera
- LeRobot dataset format
- Python 3.8 / 3.10 environments depending on the used part of the pipeline
- Ubuntu 22.04 based development machine 

##  Usage of the repository 

  ## Setup of the dependencies
  
  1. Install Gello dependencies 
  Original project: [GELLO software](https://github.com/wuphilipp/gello_software)

  2. Install polymetis dependencies 
  [Polymetis installation guide](https://facebookresearch.github.io/fairo/polymetis/installation.html)

  3. Install Intel dependencies
  Note: This is not necessarily based on a specific model of a Intel camera, but in case of different cameras, the models in the files have to be changed!

  4. Install lerobot dependencies:
  [LeRobot installation guide](https://huggingface.co/docs/lerobot/en/installation)
  Note: Lerobot training is not included in this repository. The training is executed seperate. 
  This code just contains the converter to get your pkl data from the gello into the Lerobot_v3 dataformat.



  ## Getting the gello ready 

  ## GELLO Initialization

Before starting teleoperation, the GELLO device has to be initialized so that the teleoperation setup can correctly read the GELLO joint positions and use them for controlling the Franka Emika Panda.

For the initialization pose, look at the official gello repository. 

For this project, the initialization is handled by the custom calibration script script:

```bash
cd gello_software
./calibrate_gello.sh
```

If there are problems with the initialization, open the gello_agent file and edit the offsets for the panda manually (decimals are allowed):Ö
The main GELLO agent logic is implemented in [`gello/agents/Gello_agent.py`](gello/agents/gello_agent.py).



  ## Starting the Gello 
  The Gello start requires the Polymetis robot server, the polymetis gripper server, as well as servers for the cameras. 
  The Polymetis servers are used for local communications and connect the physical panda roboter and the Gello.
  The final window starts the env, which enables the teleoperation and prepares the mode for recordings. 
  
  In the initial setup all the automatically inserted commands are fine for the Framos d435E as a wrist camera and the D455 as base camera, placed in front. Both cameras will be saved in the pkl files from each episode. 
  Attentoion: For the first start, unplug the USB D455 and start the server for the Framos. Afterwards replug the D455 and also start the second server. 



Important files and folders in this modified version:

```text
gello/
  robots/
    panda.py

experiments/
  run_env.py
  launch_two_cameras_nodes.py

convert_gello_pkl_to_lerobot_v3.py
start_gello_panda.sh
stop_gello_panda.sh