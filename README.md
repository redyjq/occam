# occam
The controller code for the Occam Omni Stereo

# Instructions
* Download the Indigo SDK tarball and uncompress it into a subdirectory (this step has already been completed on the lab computer)
* Run `cmake .` to construct the necessary makefiles
* Run `make` to build the code
* Run `./bin/occam > output.txt`. This will try to construct and display a point cloud. It will also print the RGB and XYZ values of all points to the file. The source code for this is in `src/read_point_cloud.cc`