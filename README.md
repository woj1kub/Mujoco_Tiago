# Tiago MuJoCo Demo — Repository

English README to reproduce the Tiago + MuJoCo simulation and perception pipeline used for the project.

**Important — First step (required)**
Before anything else, follow the ROS/Gazebo setup instructions from the original TIAGo Gazebo repository `hri-manipulacion`. That repository provides the ROS packages, URDF/Xacro files and MoveIt configuration required to generate the robot URDF and to populate your ROS2 workspace. Only after completing those steps should you proceed to the MuJoCo port in this repository.

Clone and follow its README:

```bash
git clone https://github.com/luispri2001/hri-manipulacion.git ~/ros2_ws/src/hri-manipulacion
# then read and follow the instructions in hri-manipulacion/README.md
```

Typical tasks performed from `hri-manipulacion` README (do these first):

- generate the URDF/Xacro for TIAGo
- build and source the ROS2 workspace
- (optionally) test the Gazebo launch that confirms the ROS topics and TF frames

After verifying the ROS/Gazebo setup from `hri-manipulacion`, return here and continue with the MuJoCo-specific steps in this README.

**Overview**
- This repository contains a MuJoCo scene and a Python node (`tiago_v3.py`) that simulates a TIAGo robot, publishes RGB+depth images, listens for YOLO 3D detections and forwards goals to MoveIt for planning. The repo also includes a YOLO model `yolov8m.pt` used by the ROS2 YOLO bringup.

**Repository contents**
- `scalony.xml` — MuJoCo scene (auxiliary)
- `scene_cup.xml` — primary MuJoCo scene used by `tiago_v3.py`
- `tiago_v3.py` — main simulation + ROS2 node (publisher, MoveIt integration)
- `README.md` — this file

**Prerequisites (high level)**
- Linux (Ubuntu recommended)
- ROS 2 Humble (or compatible ROS2 distribution)
- Python 3.8+ and pip
- MuJoCo (DeepMind MuJoCo) and Python bindings (`mujoco` and viewer)
- CUDA and a GPU (optional, only if running YOLO on GPU)

**External repositories you should clone**
Clone the following repositories into convenient locations (examples use `~/code` and `~/ros2_ws/src`):

- object_sim (optional dataset / preview utilities):

```bash
git clone https://github.com/vikashplus/object_sim.git ~/code/object_sim
```

- YOLO ROS wrapper (provides `yolo_bringup` and `yolo_msgs`):

```bash
git clone https://github.com/mgonzs13/yolo_ros.git ~/ros2_ws/src/yolo_ros
```

- MuJoCo menagerie (reference scenes and assets). If you prefer the full dataset:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git ~/code/mujoco_menagerie
```

- TIAGo description / MoveIt config (needed for URDF and MoveIt):

You will need a TIAGo robot description package and a MoveIt config (these may be provided by your course or lab). Example names used in the notes:

- `tiago_robot` (contains `tiago_description/robots/tiago.urdf.xacro`)
- `tiago_moveit_config` (MoveIt configuration for the robot)

Clone or copy those packages into your ROS2 workspace `~/ros2_ws/src/`.

**System setup and build**
1. Prepare ROS2 workspace (example):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
# copy or clone necessary ROS packages into ~/ros2_ws/src
colcon build --symlink-install
```

2. Source ROS2 and your workspace in every terminal before running ROS nodes:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

3. Install Python dependencies (system-wide or in a virtualenv):

```bash
python3 -m pip install --user numpy opencv-python mujoco
# For MoveIt / ROS Python clients you rely on ROS packages (rclpy, cv_bridge) which are provided by ROS2
```

4. MuJoCo runtime (license & library):
- Install MuJoCo (follow DeepMind MuJoCo instructions). Ensure the `mujoco` Python package points to your MuJoCo installation. You may need to set `LD_LIBRARY_PATH` or `MUJOCO_GL` depending on your system.

5. Install MuJoCo binary (recommended)

To run the simulation you need the MuJoCo native runtime. Download and extract the official release into `~/.mujoco` (example below uses version 3.9.0):

```bash
mkdir -p ~/.mujoco
cd ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.9.0/mujoco-3.9.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.9.0-linux-x86_64.tar.gz
# (optional) remove the archive after extraction
rm mujoco-3.9.0-linux-x86_64.tar.gz
```

Then set environment variables (add to `~/.bashrc` or `~/.profile`):

```bash
export MUJOCO_PATH="$HOME/.mujoco/mujoco-3.9.0"
export LD_LIBRARY_PATH="$MUJOCO_PATH/bin:$LD_LIBRARY_PATH"
# choose an OpenGL backend if needed (try 'glfw' or 'egl')
export MUJOCO_GL=glfw
```

Install the Python bindings (use the same Python interpreter that will run the ROS node):

```bash
python3 -m pip install --user mujoco mujoco-viewer
```

Notes:
- If you have problems with rendering, try changing `MUJOCO_GL` to `egl` or `osmesa` depending on your system.
- Some systems require additional GPU / OpenGL packages (e.g. `libgl1-mesa-glx`, `libgl1-mesa-dev`) — install them via your package manager.
- If your simulation references model files (e.g. `mug.obj`), ensure they are present inside the extracted `mujoco-3.9.0/model/` subfolders or update scene paths accordingly.

**Model files**
- `yolov8m.pt` (YOLOv8 medium model) is included in this repository root. If you use a different model, update the YOLO launch command accordingly.

**Run order and commands**
Follow this order to bring up the whole system.

1. In each ROS terminal, source ROS and workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. Launch YOLO ROS bringup (example using model file placed in repo):

```bash
ros2 launch yolo_bringup yolov8.launch.py model_type:=YOLO model:=yolov8m.pt device:=cuda:0 use_3d:=True target_frame:=head_front_camera_link
```

If you do not have a GPU or want CPU-only, change `device:=cpu`.

3. Start the robot_state_publisher with the compiled URDF (in one terminal):

```bash
# generate URDF if needed (example xacro command):
ros2 run xacro xacro tiago_robot/tiago_description/robots/tiago.urdf.xacro arm:=True end_effector:=pal-gripper > tiago_compiled.urdf

# publish the robot_state
ros2 run robot_state_publisher robot_state_publisher tiago_compiled.urdf
```

4. Launch MoveIt (in another terminal):

```bash
ros2 launch tiago_moveit_config move_group.launch.py use_sim_time:=True
```

5. Start the MuJoCo simulation + ROS node. From this repository run:

```bash
python3 tiago_v3.py
# or if you keep an older script name:
# python3 tiago_mujoco.py
```

This node publishes RGB and depth images on `/camera/rgb/image_raw` and `/camera/depth/image_raw`, publishes camera info, and advertises `FollowJointTrajectory` action to accept trajectories from MoveIt.

6. Optional: view images with rqt_image_view

```bash
ros2 run rqt_image_view rqt_image_view
```

**Notes and troubleshooting**
- Make sure topic namespaces and frame IDs match across YOLO, MoveIt and the simulation (the example uses `head_front_camera_link` and `base_link`).
- If the MoveIt action server name differs, update the `MoveGroup` action client/topic in `tiago_v3.py`.
- If MuJoCo rendering fails or OpenGL issues appear, try setting `MUJOCO_GL` environment variable (e.g. `glfw`, `egl`), or run in headless mode.
- Ensure `cv_bridge` and the ROS2 Python bindings are available for the Python interpreter running `tiago_v3.py`. Usually sourcing ROS2 workspace provides the required Python paths.

**Credits & references**
- MuJoCo menagerie: https://github.com/google-deepmind/mujoco_menagerie
- YOLO ROS wrapper: https://github.com/mgonzs13/yolo_ros
- Object sim preview: https://github.com/vikashplus/object_sim
- HRI manipulation utils: https://github.com/luispri2001/hri-manipulacion

---
If you want, I can: (a) add a minimal `requirements.txt` and `setup` script, (b) generate a sample `launch` file that runs the main components, or (c) adapt README paths to relative repo layout. Let me know which you prefer.

New helper files added in this repository:

- `requirements.txt` — minimal Python packages used by the simulation node (non-ROS deps).
- `setup.sh` — small script that creates a virtualenv and installs `requirements.txt` into it.
- `start_all.sh` — helper that prints the recommended commands to run in separate terminals to start robot_state_publisher, MoveIt, YOLO bringup and the MuJoCo node.

To use them:

```bash
# create virtualenv and install python deps
./setup.sh

# run the sample launcher helper (prints commands to run in separate terminals)
./start_all.sh
```

Notes:
- `tiago_v3.py` is not packaged as a ROS package here — the `start_all.sh` assumes you will run it directly from this repository after sourcing ROS2 and your workspace.
- `requirements.txt` contains only non-ROS Python packages; ROS packages must be provided by your ROS2 installation and workspace.