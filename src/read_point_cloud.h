#include <assert.h>
#include <indigo.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <limits>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Header.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <beam_joy/PointcloudAndImage.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::pair<OccamDevice *, OccamDeviceList *> initialize();
void disposeOccamAPI(std::pair<OccamDevice *, OccamDeviceList *> occamAPI);
cv::Mat getStitchedAndPointCloud(OccamDevice *device, PointCloudT::Ptr pc);
void saveImage(OccamImage *image, std::string fileName);
void saveImage(cv::Mat *image, std::string fileName);
void **captureRgbAndDisparity(OccamDevice *device);
