# occam
The controller code for the Occam Omni Stereo

# Occam API Instructions

## Setup
### Download
 - Clone the repo into the `/src` folder of your ROS workspace
`cd ~/ros_ws/src/`
`git clone https://github.com/Cornell-RPAL/occam.git`

### Dependencies
 - Clone the other repos
`git clone https://github.com/Cornell-RPAL/beam_joy.git`

### Install
 - Run `catkin_make -j8` to build
	 - then `source ~/ros_ws/devel/setup.bash`

## ROS API

### Subscribed Topics
 -  /beam/odom [nav_msgs/Odometry]

### Published Topics
 - /occam/points_rgb_odom [beam_joy/PointcloudImagePose]

### Parameters

 - auto_exposure (bool, default: True)
	 - Image auto exposure value [False, True]

## Usage

### Connect to the Beam

You can connect to Beam with an Ethernet cable. The network should be configured as `192.168.68.2/255.255.255.0`. Beam will be `192.168.68.1`.
 - `export ROS_HOSTNAME=192.168.68.2 ROS_MASTER_URI=http://192.168.68.2:11311`
 - `roscore`
 - `ssh st@192.168.68.1 rosbeam-bridge.sh`, password is st.
Now you will be able to see ROS topics from the beam like `/beam/odom` [nav_msgs/Odometry] and you should be able send movement commands on `/beam/cmd_vel` [geometry_msgs/Twist]

### Realtime operation
  - Runs the occam node
`rosrun occam occam`

  - Runs RViz with the included config file to visualize the pointcloud and images:
 `roslaunch occam viz.launch`
 
  - Runs both the occam node and RViz with the included config file:
`roslaunch occam occam.launch`

### Joystick

 - To enable teleoperation of the Beam with the Logitech controller, use the beam_joy package.
`roslaunch beam_joy beam_joy.launch`

### Recording

 - You can create rosbags of the data coming from the occam for playback later. Use `rosbag record` and the topics you want to save.
`rosbag record --duration=30 occam/points_rgb_odom`

 - Playback the rosbag files with `rosbag play` and the name of the rosbag file
`rosbag play -l -s 10 occam_2016-11-29-17-03-04.bag`

 - More documentation on rosbags at http://wiki.ros.org/rosbag/Commandline

### Extract Files
If you would like to save PointcloudImagePose msgs as .pcd, .jpg, and .txt files:

 - run the extraction script with `rosrun occam extract_points_rgb_odom.py`
 - play the rosbag file with `rosbag play occam_2016-12-19-.bag` in a new terminal or use realtime data coming from the occam with `rosrun occam occam`
 - the files will be extracted to the current directory

## Notes

###Point Cloud Quality
If the point clouds are of poor quality (too sparse), tuning the `OCCAM_BM_UNIQUENESS_RATIO` in `indigosdk-2.0.15/src/indigo.h` may help. Additionally, if points are cut off beyond a certain distance, tune the `CULL_THRESHOLD` variable in `read_point_cloud.cc`. Alternatively, a more robust stereo algorithm can be easily implemented by including a library, at the expense of performance; see [libelas](http://www.cvlibs.net/software/libelas/). The following email from Xavier has more details:

    The SDK with the camera uses the standard "block matching" algorithm that is directly ported from OpenCV, but there are quite a few others that can be used. Block matching is completely "local", so depends the most on scene texture but is also the fastest. With passive stereo there is a core trade off between accuracy and density vs computation time. A good example of a recent semi-global algorithm is ELAS [1], which can be easily integrated given the rectified images from the SDK. There are also a number of other algorithms included in OpenCV.


###Examples
If you would like to use the Occam in a different way, run `rosrun occam read_images_opencv` (source file in `indigosdk-2.0.15/examples/read_images_opencv.cc`) and press 1 or 2 to browse the different options available. The other examples may also be useful. In order to record multiple types of data, say an OccamImage and an OccamPointCloud concurrently, for example, `indigosdk-2.0.15/examples/read_raw_images.cc` does this (also `src/read_pointcloud.cc`).

###Issues
If you run into issues working with the API, contact Zach Vinegar (zzv2@cornell.edu), Daryl Sew (darylsew@gmail.com) and Xavier Delacour (xavier@occamvisiongroup.com), our Occam support contact.  
