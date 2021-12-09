# ros2_shm_vision_demo

Demonstrate how to use shared memory with image processing algorithms.

Note this is highly experimental and requires much clean up. It may still serve as an inspiration how to set up a computer vision app in ROS 2 with shared memory for high transmission efficiency.

## Requirements

Only works with ROS 2 Rolling.
ROS 2 Galactic is not supported due to some cmake command changes.

Also require yolo3 object detection parameters found here.

## Installation

TODO: more detail

1. Install ROS 2 Rolling

2. Clone the ros2_shm_vision_demo repo into the src folder of the ROS 2 Rolling installation.

3. Build using `colcon build --packages-up-to ros2_shm_vision_demo`

4. Create a folder vision_config in your ROS 2 Rolling workspace (parallel to src). Copy the cyclonedds.xml and roudi_config.toml to this folder. (This should be automatically installed properly later but currently it is not).

5. Obtain the yolo3 config files from https://pjreddie.com/darknet/yolo/ and copy them to the same folder (The application currently uses hardcoded paths, to be changed later). The following files are needed coco.names (config folder), yolov3.cfg and yolov3.weights (YOLOv3-320 for the right image size). The weights file is fairly large so it is not included in the repository.

6. Adapt the `ros2_ws` and `video` variables as in the of the tmux scripts (scripts folder) as needed (TODO install scripts, relative paths). `ros2_ws` needs to point to the ROS 2 rolling workspace and `video` to some full hd video.

## Running the applications

### Without Shared Memory

Run the `demo.sh` script, it should start all applications and show the corresponding outputs.
Alternatively all (or a subset of) the applications can be started on their own (cf. parameterization in the script).

'stop_demo.sh` stops all applications and the tmux session.

### With Shared Memory

Run the `demo.sh` script, it should start all applications and show the corresponding outputs. This will automatically load the `cyclonedds.xml` configuration. In addition the RouDi shared memory daemon will be started with the configuration `roudi_config.toml`-

'stop_demo.sh` stops all applications and the tmux session.

## TODO

1. more detailed explanation and system description
2. clean up code and add more documentation
3. proper installation (scripts, config, yolo etc.)
4. extend to image pipeline, use ROS 2 executor (?)

