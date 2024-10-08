**This project involves multi-robot exploration and mapping using an Ackerman car-like robot, and includes reconnaissance. The project is a collaborative effort by five members. The multi-robot exploration uses the open-source RRT exploration algorithm, which has been ported to ROS Melodic and ROS Noetic. For reconnaissance, YOLOv5 is used, and the visualization software for the reconnaissance results is developed using QT5 and PyQT.**

[video](./video/video.mp4)

The repository is designed for the Ubuntu 18.04 operating system environment. However, after modifying several robot URDF files, it can still run smoothly on Ubuntu 20.04.

## File Description
- `qingzhou_simulation` contains the source code for all tasks. To reproduce, create a package, move the `src` folder from this directory into the created workspace, and compile using `catkin_make` (if compilation fails, please recompile as this is due to the compilation order).
- `RTSPTool_six_windows` is the Qt source code. Open this folder with Qt Creator, compile, and run to open the video transmission system (provided that the ffmpeg tool is configured, see [Environment Installation and Configuration](./Environment_Installation_and_Configuration.md) for specific configuration methods).
- The `models` folder contains our custom model file `my_environment`. Before starting the simulation environment, add `my_environment` to the Gazebo model library.
  To add it, open the folder, use `ctrl+h` to show hidden folders, go to `.gazebo`, then to the `models` folder, and copy `my_environment` there.
  
## Installing Dependencies
```bash
# if you use ubuntu20.04, change the melodic to noetic
sudo apt install ros-melodic-ackermann-*
sudo apt install ros-melodic-gmapping
sudo apt install ros-melodic-navigation
sudo apt install python-scikits-learn
sudo apt install python-opencv
sudo apt install python-numpy
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
sudo apt-get install ros-melodic-visualization-tutorials
```


## How to Start the Simulation
1. Start the simulation environment (including launching Gazebo and RViz).
    ```bash
    roslaunch rrt_exploration_tutorial mutliple_simulated_house_qingzhou.launch
    # If the fusion map does not start successfully, manually start it using the command below.
    roslaunch rrt_exploration_tutorial map_merger.launch
    ```
    
2. Start RRT autonomous exploration mapping.
    ```bash
    # After starting, do not use the "Publish Point" feature in RViz to publish a clicked point.
    roslaunch rrt_exploration four_robots_qingzhou.launch
    ```
    
3. Start the node that publishes the fusion map to facilitate streaming to the RTSP server. This node also includes the functionality to display recognized special targets in the PyQt interface.
    ```bash
    rosrun qingzhou_description transform_merge_map.py
    ```
    
4. Start the RTSP server to stream the image data to RTSP.
    ```bash
    roslaunch ros_rtsp rtsp_streams.launch
    ```
    
5. Open Qt and run the real-time video transmission system. Use ffmpeg to pull the stream file from the RTSP server, decode it, and display it in the Qt program. The URL is already pre-filled, so you can simply click "Open." Please wait for a moment, and ensure that one window is already displaying the video before opening another window.
   
6. Start the PyQt reconnaissance system interface.
    ```bash
    roscd rviz_python_tutorial
    ./myviz.py
    ```
    It's possible that the system already has `rviz_python_tutorial` installed, which means there might also be a `myviz.py` script in the `opt` path. You will need to make a choice.
    
7. Start the YOLO detection node.
    First, activate the conda environment. Detailed configuration instructions can be found in our [Environment Installation and Configuration](./Environment_Installation_and_Configuration.md) document.
    
    ```bash
    roslaunch yolov5_ros yolo_v5.launch
    ```
    
8. Start the script to publish the coordinates of special targets.
    Use the `publish point` tool in RViz to mark five points. The first four points should form a rectangle, which defines the growth boundary of the global random tree, so it should be as large as possible (at least covering the entire environment). Importantly, the order of these four points should be top-left, bottom-left, bottom-right, and top-right. The last point sets the root node where the global random tree starts growing, and this point must be within the already mapped area.

## Contributors

<a href="https://github.com/JXHoverChan">
  <img src="https://avatars.githubusercontent.com/u/103250331?v=4" width="100" height="100"/>
</a>

<a href="https://github.com/lrlqaq">
  <img src="https://avatars.githubusercontent.com/u/121630424?v=4" width="100" height="100"/>
</a>

<a href="https://github.com/LeonickyLiu">
  <img src="https://avatars.githubusercontent.com/u/147980030?v=4" width="100" height="100"/>
</a>

<a href="https://github.com/CookedPotatoes114514">
  <img src="https://avatars.githubusercontent.com/u/184182973?v=4" width="100" height="100"/>
</a>
