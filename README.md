# occam
The controller code for the Occam Omni Stereo

# Instructions
* Download the Indigo SDK tarball and uncompress it into a subdirectory (this step has already been completed on the lab computer)
* Replace the `occam/indigosdk-2.0.34/src/omnis5u3mt9v022.cc` file with the `occam/src/omnis5u3mt9v022.cc` file
* Run `cmake .` to construct the necessary makefiles
* Run `make` to build the code
* Run `./bin/occam`. This will try to construct and display a point cloud. It will also save the RGB and XYZ values of all points to the `data/pointcloud.pcd` file. The source code for this is in `src/read_point_cloud.cc`