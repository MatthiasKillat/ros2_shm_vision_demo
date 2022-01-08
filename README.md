# shm_vision_demo

Demonstrate how to use shared memory with image processing algorithms.

Note this is highly experimental and requires much clean up. It may still serve as an inspiration how to set up a computer vision app in ROS 2 with shared memory for high transmission efficiency.

## Requirements

As of today this package only supports ROS 2 Rolling.

ROS 2 Galactic is not supported due to some cmake command changes.

Also require yolov3 object detection parameters which are automatically downloaded at build time using the `scripts/download_yolo_weights.sh` script.

## Installation

TODO: more detail

1. Install ROS 2 Rolling

2. Clone the `shm_vision_demo` repo into your `colcon_ws/src` directory

3. Clone `iceoryx` repo into your `colcon_ws/src` directory - parallel with the repo you just cloned in step number two

4. Build using `colcon build --packages-up-to shm_vision_demo`.  While building it will automatically fetch the `yolov3.weights` file from https://pjreddie.com/darknet/yolo/ and copy them to the `shm_vision_demo/config` directory. The weights file is fairly large so it is not included in the repository but automatically downloaded during the build.

The following files are needed in the `config/` directory:
  - `coco.names`
  - `yolov3.cfg`
  - `yolov3.weights` (YOLOv3-320 for the right image size)


5. Adapt the `ros2_ws` and `video` variables in the `scripts/demo.sh` and `scripts/shm_demo.sh` scripts, located in the `scripts` folder as needed. `ros2_ws` needs to point to the ROS 2 rolling workspace and `video` to some full hd video.

## Running the applications

By default the provided `demo.sh` and `shm_demo.sh` scripts will try to use your devices camera running live. If for some reason you do not wish to use your camera, you may pass in an argument to each script specifying an `.mp4` file to use instead.

### Quickstart

```
cd shm_vision_demo/scripts
# to run on live camera connected to device
./shm_demo.sh
# to run demo on pre-recorded video
./shm_demo.sh ~/videos/my_cool_video.mp4
```

### Without Shared Memory

Ensure you are in the scripts directory: `cd shm_vision_demo/scripts/`

Run the `./demo.sh` script, it should start all applications and show the corresponding outputs.
Alternatively all (or a subset of) the applications can be started on their own (cf. parameterization in the script).

`stop_demo.sh` stops all applications and the tmux session or alternatively enter `ctrl+b` then `kill-session` to kill the running `tmux` session.

### With Shared Memory

Run the `shm_demo.sh` script, it should start all applications and show the corresponding outputs. This will automatically load the `cyclonedds.xml` configuration. In addition the RouDi shared memory daemon will be started with the configuration `roudi_config.toml`-

`stop_demo.sh` stops all applications and the tmux session or alternatively enter `ctrl+b` then `kill-session` to kill the running `tmux` session.

## TODO

1. more detailed explanation and system description
2. clean up code and add more documentation
3. proper installation (scripts, config, yolo etc.)
4. extend to image pipeline, use ROS 2 executor (?)

