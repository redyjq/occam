#include "read_point_cloud.h"

void handleError(int returnCode) {
  if (returnCode != OCCAM_API_SUCCESS) {
    char errorMsg[30];
    occamGetErrorString((OccamError)returnCode, errorMsg, 30);
    fprintf(stderr, "Occam API Error: %d. %s\n", returnCode, errorMsg);
    abort();
  }
}

int convertToPcl(OccamPointCloud *occamPointCloud,
                 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  int numPointsConverted = 0;
  int i;
  for (i = 0; i < 3 * occamPointCloud->point_count; i += 3) {
    pcl::PointXYZRGBA *point = new pcl::PointXYZRGBA();

    point->r = occamPointCloud->rgb[i];
    point->g = occamPointCloud->rgb[i + 1];
    point->b = occamPointCloud->rgb[i + 2];
    point->a = 255;

    point->x = occamPointCloud->xyz[i];
    point->y = occamPointCloud->xyz[i + 1];
    point->z = occamPointCloud->xyz[i + 2];

    // printf("R: %d G: %d B: %d X: %f Y: %f Z: %f\n", point->r, point->g,
    // point->b, point->x, point->y, point->z);

    double CULL_THRESHOLD = 1000;
    double inf = std::numeric_limits<double>::infinity();
    if (point->x < inf && point->y < inf && point->z < inf) {
      double sqdist =
          point->x * point->x + point->y * point->y + point->z * point->z;
      if (sqrt(sqdist) < CULL_THRESHOLD) {
        pclPointCloud->push_back(*point);
        numPointsConverted++;
      }
    }
  }

  pclPointCloud->is_dense = false;

  return numPointsConverted;
}

void savePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud,
                    int counter) {
  double timestamp = pcl::getTime();
  std::stringstream ss;
  ss << "data/pointcloud" << counter << ".pcd";
  std::string name = ss.str();
  pcl::PCDWriter writer;
  if (point_cloud->size() > 0) {
    writer.write<pcl::PointXYZRGBA>(name, *point_cloud);
  }
}

void occamImageToCvMat(OccamImage *&image, cv::Mat *&cvImage) {
  if (image && image->format == OCCAM_GRAY8) {
    cvImage = new cv::Mat_<uchar>(image->height, image->width,
                                  (uchar *)image->data[0], image->step[0]);
  } else if (image && image->format == OCCAM_RGB24) {
    cvImage =
        new cv::Mat_<cv::Vec3b>(image->height, image->width,
                                (cv::Vec3b *)image->data[0], image->step[0]);
    cv::Mat *colorImage = new cv::Mat();
    cv::cvtColor(*cvImage, *colorImage, cv::COLOR_BGR2RGB);
    cvImage = colorImage;
  } else if (image && image->format == OCCAM_SHORT1) {
    cvImage = new cv::Mat_<short>(image->height, image->width,
                                  (short *)image->data[0], image->step[0]);
  } else {
    printf("Image type not supported: %d\n", image->format);
    cvImage = NULL;
  }
}

void saveImage(OccamImage *image, std::string fileName) {
  cv::Mat *cvImage;
  occamImageToCvMat(image, cvImage);
  saveImage(cvImage, fileName);
}

void saveImage(cv::Mat *cvImage, std::string fileName) {
  imwrite(fileName, *cvImage);
}

void capturePointCloud(OccamDevice *device,
                       pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud,
                       OccamDataName PCLOUD) {
  // Capture a point cloud
  OccamDataName requestTypes[] = {PCLOUD};
  OccamDataType returnTypes[] = {OCCAM_POINT_CLOUD};
  OccamPointCloud *pointCloud;
  handleError(occamDeviceReadData(device, 1, requestTypes, returnTypes,
                                  (void **)&pointCloud, 1));

  // Print statistics
  // printf("Number of points in Occam point cloud: %d\n", pointCloud->point_count);

  // Convert to PCL point cloud
  int numConverted = convertToPcl(pointCloud, pclPointCloud);
  // printf("Number of points converted to PCL: %d\n", numConverted);

  // Clean up
  handleError(occamFreePointCloud(pointCloud));
}

const int sensor_count = 5;
Eigen::Matrix4f extrisic_transforms[sensor_count];
void initSensorExtrisics(OccamDevice *device) {
  for (int i = 0; i < sensor_count; ++i) {
    // Get sensor extrisics
    double R[9];
    handleError(occamGetDeviceValuerv(
        device, OccamParam(OCCAM_SENSOR_ROTATION0 + i), R, 9));
    double T[3];
    handleError(occamGetDeviceValuerv(
        device, OccamParam(OCCAM_SENSOR_TRANSLATION0 + i), T, 3));

    // Init transform
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Set rotation
    transform(0, 0) = R[0];
    transform(1, 0) = R[1];
    transform(2, 0) = R[2];
    transform(0, 1) = R[3];
    transform(1, 1) = R[4];
    transform(2, 1) = R[5];
    transform(0, 2) = R[6];
    transform(1, 2) = R[7];
    transform(2, 2) = R[8];

    // Set translation
    transform(0, 3) = T[0];
    transform(1, 3) = T[1];
    transform(2, 3) = T[2];

    extrisic_transforms[i] = transform;

    printf ("Transform for sensor %d:\n", i-1);
    std::cout << extrisic_transforms[i] << std::endl;
  }
}

Eigen::Matrix4f transform_from_pose(geometry_msgs::Pose pose) {
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  // Get a rotation matrix from the quaternion
  tf::Matrix3x3 m(q);

  // Init transform
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // Set rotation
  transform (0,0) = m.getRow(0)[0];
  transform (0,1) = m.getRow(0)[1];
  transform (0,2) = m.getRow(0)[2];
  transform (1,0) = m.getRow(1)[0];
  transform (1,1) = m.getRow(1)[1];
  transform (1,2) = m.getRow(1)[2];
  transform (2,0) = m.getRow(2)[0];
  transform (2,1) = m.getRow(2)[1];
  transform (2,2) = m.getRow(2)[2];

  // Set translation
  transform (0,3) = pose.position.x;
  transform (1,3) = pose.position.y;
  transform (2,3) = pose.position.z;

  return transform;
}

Eigen::Matrix4f occam_to_beam_and_scale_transform;
Eigen::Matrix4f beam_odom_transform;
void initTransforms() {
  // scale the cloud from cm to m
  double scale = 0.01;
  Eigen::Matrix4f scale_transform = Eigen::Matrix4f::Identity();
  scale_transform (0,0) = scale;
  scale_transform (1,1) = scale;
  scale_transform (2,2) = scale;

  // use the transform from the beam robot base to the occam frame to orient the cloud
  geometry_msgs::Pose occam_to_beam_pose;
  // done so that z axis points up, x axis points forward, y axis points left
  occam_to_beam_pose.orientation.x = -0.5;
  occam_to_beam_pose.orientation.y = 0.5;
  occam_to_beam_pose.orientation.z = -0.5;
  occam_to_beam_pose.orientation.w = 0.5;
  // occam_to_beam_pose.orientation.w = 1.0;
  // occam_to_beam_pose.position.z = 0.5;  
  Eigen::Matrix4f occam_to_beam_transform = transform_from_pose(occam_to_beam_pose);
  // Eigen::Matrix4f occam_to_beam_transform = Eigen::Matrix4f::Identity();

  // combine the transforms
  occam_to_beam_and_scale_transform = occam_to_beam_transform * scale_transform;

  // if no odom recieved, assume identity transform
  beam_odom_transform = Eigen::Matrix4f::Identity();

  printf("Initialized Transforms.\n");
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Update the matrix used to transform the pointcloud to the odom frame
  beam_odom_transform = transform_from_pose(msg->pose.pose);  
}

void occamCloudsToPCL(
  OccamPointCloud** pointClouds,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  // use latest odom data to transform the cloud with the movement of the robot
  Eigen::Matrix4f odom_occam_transform = beam_odom_transform * occam_to_beam_and_scale_transform;
  for (int i = 0; i < sensor_count; ++i) {
    // Print statistics
    // printf("Number of points in OCCAM_POINT_CLOUD%d: %d\n", i, pointClouds[i]->point_count);

    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    int numConverted = convertToPcl(pointClouds[i], tempCloud);
    // printf("Number of points converted to PCL: %d\n", numConverted);

    // combine extrisic transform and then apply to the cloud
    Eigen::Matrix4f combined_transform = odom_occam_transform * extrisic_transforms[i];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud (*tempCloud, *transformedCloud, combined_transform);

    // Add to large cloud
    *pclPointCloud += *transformedCloud;
  }
  printf("Number of points in large cloud: %zu\n", pclPointCloud->size());
}

OccamPointCloud** captureAllOccamClouds(OccamDevice *device) {
  // Capture all point clouds
  OccamDataName *requestTypes = (OccamDataName *)occamAlloc(sensor_count * sizeof(OccamDataName));
  for (int i = 0; i < sensor_count; ++i) {
    requestTypes[i] = (OccamDataName)(OCCAM_POINT_CLOUD0 + i);
  }
  OccamDataType returnTypes[] = {OCCAM_POINT_CLOUD};
  OccamPointCloud** pointClouds = (OccamPointCloud**) occamAlloc(sensor_count * sizeof(OccamPointCloud*));
  handleError(occamDeviceReadData(device, sensor_count, requestTypes, 0, (void**)pointClouds, 1));
  return pointClouds;
}

void visualizePointCloud(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("PCL Viewer"));

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(
      pclPointCloud);
  viewer->addPointCloud<pcl::PointXYZRGBA>(pclPointCloud, rgb, "cloud");
  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}

OccamImage *captureImage(OccamDevice *device, OccamDataName requestType) {
  OccamDataName *req = (OccamDataName *)occamAlloc(sizeof(OccamDataName));
  req[0] = requestType;
  OccamImage **images = (OccamImage **)occamAlloc(sizeof(OccamImage *));
  handleError(occamDeviceReadData(device, 1, req, 0, (void **)images, 1));
  occamFree(req);
  return images[0];
}

void constructPointCloud(
    OccamDevice *device,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  // Capture RBG image and disparity image
  // OccamImage* rgbImage = captureImage(device, OCCAM_IMAGE2);
  // printf("RGB Image captured at time: %llu\n", (long long unsigned
  // int)rgbImage->time_ns);
  // OccamImage* disparityImage = captureImage(device, OCCAM_DISPARITY_IMAGE1);
  // printf("Disparity Image captured at time: %llu\n", (long long unsigned
  // int)disparityImage->time_ns);

  // Save the images
  // saveImage(rgbImage, "rgbImage.jpg");
  // saveImage(disparityImage, "disparityImage.jpg");

  // Get basic sensor information for the camera
  int sensor_count;
  handleError(occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensor_count));
  int sensor_width;
  handleError(occamGetDeviceValuei(device, OCCAM_SENSOR_WIDTH, &sensor_width));
  int sensor_height;
  handleError(
      occamGetDeviceValuei(device, OCCAM_SENSOR_HEIGHT, &sensor_height));

  // Initialize sensor parameter variables
  double *Dp[sensor_count];
  double *Kp[sensor_count];
  double *Rp[sensor_count];
  double *Tp[sensor_count];

  // XXX Rp and Tp are the extrinsics for the occam
  // Get sensor parameters for all sensors
  for (int i = 0; i < sensor_count; ++i) {
    double D[5];
    handleError(occamGetDeviceValuerv(
        device, OccamParam(OCCAM_SENSOR_DISTORTION_COEFS0 + i), D, 5));
    Dp[i] = D;
    double K[9];
    handleError(occamGetDeviceValuerv(
        device, OccamParam(OCCAM_SENSOR_INTRINSICS0 + i), K, 9));
    Kp[i] = K;
    double R[9];
    handleError(occamGetDeviceValuerv(
        device, OccamParam(OCCAM_SENSOR_ROTATION0 + i), R, 9));
    Rp[i] = R;
    double T[3];
    handleError(occamGetDeviceValuerv(
        device, OccamParam(OCCAM_SENSOR_TRANSLATION0 + i), T, 3));
    Tp[i] = T;
  }

  // Initialize interface to the Occam Stereo Rectify module
  void *rectifyHandle = 0;
  occamConstructModule(OCCAM_MODULE_STEREO_RECTIFY, "prec", &rectifyHandle);
  assert(rectifyHandle);
  IOccamStereoRectify *rectifyIface = 0;
  occamGetInterface(rectifyHandle, IOCCAMSTEREORECTIFY, (void **)&rectifyIface);
  assert(rectifyIface);

  // Configure the module with the sensor information
  handleError(rectifyIface->configure(rectifyHandle, sensor_count, sensor_width,
                                      sensor_height, Dp, Kp, Rp, Tp, 0));

  OccamImage *images = (OccamImage *)captureRgbAndDisparity(device);
  // Get point cloud for the image
  int indices[] = {0, 1, 2, 3, 4};
  OccamImage *rgbImages[5];
  OccamImage *disparityImages[5];
  for (int i = 0; i < 5; i++) {
    rgbImages[i] = &images[i];
    disparityImages[i] = &images[i + 5];
  }
  OccamPointCloud **pointClouds =
      (OccamPointCloud **)occamAlloc(sizeof(OccamPointCloud *) * 5);
  handleError(rectifyIface->generateCloud(
      rectifyHandle, 1, indices, 1, rgbImages, disparityImages, pointClouds));

  /* // method signature
     virtual int generateCloud(int N,const int* indices,int transform,
                          const OccamImage* const* img0,const OccamImage* const*
     disp0,
                          OccamPointCloud** cloud1) = 0;
   */

  // Print statistics
  // for (int i = 0; i < 5; i++) {
  //   printf("Number of points in Occam point cloud: %d\n", pointClouds[i]->point_count);
  // }

  /*

  // Convert to PCL point cloud
  int numConverted = convertToPcl(pointCloud, pclPointCloud);
  printf("Number of points converted to PCL: %d\n", numConverted);
  */

  // Clean up
  for (int i = 0; i < 5; i++) {
    handleError(occamFreePointCloud(pointClouds[i]));
  }
}

void **captureStitchedAndPointCloud(OccamDevice *device) {
  OccamDataName *req = (OccamDataName *)occamAlloc(6 * sizeof(OccamDataName));
  req[0] = OCCAM_STITCHED_IMAGE0;
  req[1] = OCCAM_POINT_CLOUD0;
  req[2] = OCCAM_POINT_CLOUD1;
  req[3] = OCCAM_POINT_CLOUD2;
  req[4] = OCCAM_POINT_CLOUD3;
  req[5] = OCCAM_POINT_CLOUD4;
  OccamDataType returnTypes[] = {OCCAM_IMAGE, OCCAM_POINT_CLOUD};
  void **data = (void **)occamAlloc(sizeof(void *) * 6);
  handleError(occamDeviceReadData(device, 6, req, returnTypes, data, 1));
  return data;
}

void **captureRgbAndDisparity(OccamDevice *device) {
  int num_images = 10;
  OccamDataName *req =
      (OccamDataName *)occamAlloc(num_images * sizeof(OccamDataName));
  req[0] = OCCAM_IMAGE0;
  req[1] = OCCAM_IMAGE1;
  req[2] = OCCAM_IMAGE2;
  req[3] = OCCAM_IMAGE3;
  req[4] = OCCAM_IMAGE4;
  req[5] = OCCAM_DISPARITY_IMAGE0;
  req[6] = OCCAM_DISPARITY_IMAGE1;
  req[7] = OCCAM_DISPARITY_IMAGE2;
  req[8] = OCCAM_DISPARITY_IMAGE3;
  req[9] = OCCAM_DISPARITY_IMAGE4;
  OccamDataType returnTypes[] = {OCCAM_IMAGE};
  // void** data = occamAlloc(sizeof(OccamImage*) + sizeof(OccamPointCloud*));
  void **data = (void **)occamAlloc(sizeof(void *) * num_images);
  handleError(
      occamDeviceReadData(device, num_images, req, returnTypes, data, 1));
  return data;
}

std::pair<OccamDevice *, OccamDeviceList *> initializeOccamAPI() {

  // Initialize Occam SDK
  handleError(occamInitialize());

  // Find all connected Occam devices
  OccamDeviceList *deviceList;
  handleError(occamEnumerateDeviceList(2000, &deviceList));
  int i;
  for (i = 0; i < deviceList->entry_count; ++i) {
    printf("%d. Device identifier: %s\n", i, deviceList->entries[i].cid);
  }

  // Connect to first device
  OccamDevice *device;
  char *cid = deviceList->entries[0].cid;
  handleError(occamOpenDevice(cid, &device));
  printf("Opened device: %p\n", device);
  return std::make_pair(device, deviceList);
}

void disposeOccamAPI(std::pair<OccamDevice *, OccamDeviceList *> occamAPI) {
  // Clean up
  handleError(occamCloseDevice(occamAPI.first));
  handleError(occamFreeDeviceList(occamAPI.second));
  handleError(occamShutdown());
}

void getStitchedAndPointCloud(OccamDevice *device,
                              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud,
                              cv::Mat *&cvImage) {
  void **data = captureStitchedAndPointCloud(device);
  OccamImage *image = (OccamImage *)data[0];
  occamImageToCvMat(image, cvImage);
  handleError(occamFreeImage(image));

  // use latest odom data to transform the cloud with the movement of the robot
  Eigen::Matrix4f odom_occam_transform = beam_odom_transform * occam_to_beam_and_scale_transform;
  for (int i = 0; i < sensor_count; ++i) {
    OccamPointCloud *occamCloud = (OccamPointCloud *)data[i+1];
    // Print statistics
    // printf("Number of points in OCCAM_POINT_CLOUD%d: %d\n", i, occamCloud->point_count);

    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    int numConverted = convertToPcl(occamCloud, tempCloud);
    // printf("Number of points converted to PCL: %d\n", numConverted);

    // combine extrisic transform and then apply to the cloud
    Eigen::Matrix4f combined_transform = odom_occam_transform * extrisic_transforms[i];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud (*tempCloud, *transformedCloud, combined_transform);

    // Add to large cloud
    *pclPointCloud += *transformedCloud;
  }
  for (int i = 0; i < sensor_count; ++i) {
    handleError(occamFreePointCloud((OccamPointCloud *)data[i+1]));
  }
  printf("Number of points in large cloud: %zu\n", pclPointCloud->size());
}

int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "beam_occam");
  ros::NodeHandle n;
  printf("Initialized ROS node.\n");

  // Subscribe to odometry data
  ros::Subscriber odom_sub = n.subscribe("/beam/odom", 1, odomCallback);
  // PointCloud2 publisher
  ros::Publisher pc2_pub = n.advertise<sensor_msgs::PointCloud2>("/occam/points", 1);
  ros::Publisher pc2_and_stitched_pub = n.advertise<beam_joy::PointcloudAndImage>("/occam/points_and_stitched", 1);

  // image_transport::ImageTransport it(n);
  // image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Publisher stitched_pub = n.advertise<sensor_msgs::Image>("/occam/stitched", 1);

  std::pair<OccamDevice *, OccamDeviceList *> occamAPI = initializeOccamAPI();
  OccamDevice *device = occamAPI.first;
  OccamDeviceList *deviceList = occamAPI.second;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  // initialize global constants
  initSensorExtrisics(device);
  initTransforms();

  // Capture Rate Test
  // ros::Rate loop_rate(5);
  // while (ros::ok()) {
  //   clock_t start = clock();
  //   printf("captureAllOccamClouds\n");
  //   OccamPointCloud** occamClouds = captureAllOccamClouds(device);
  //   occamFree(occamClouds);
  //   ros::spinOnce();
  //   loop_rate.sleep();
  //   cout << ( clock() - start ) / (double) CLOCKS_PER_SEC << " #######################################" << endl;
  // }
  
  cv::Mat *cvImage;
  int counter = 0;
  while (ros::ok()) {
    (*cloud).clear();

    getStitchedAndPointCloud(device, cloud, cvImage);

    // Convert cvImage to ROS Image msg and publish
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *cvImage).toImageMsg();
    stitched_pub.publish(img_msg);

    // Convert PCL pointcloud to ROS PointCloud2 msg and publish
    sensor_msgs::PointCloud2 pc2;
    pcl::PCLPointCloud2 tmp_cloud;
    pcl::toPCLPointCloud2(*cloud, tmp_cloud);
    pcl_conversions::fromPCL(tmp_cloud, pc2);
    pc2.header.frame_id = "odom";
    pc2_pub.publish(pc2);

    // Init PointcloudAndImage msg
    beam_joy::PointcloudAndImage pc_img_msg;
    pc_img_msg.pc = pc2;
    pc_img_msg.img = *img_msg;
    pc2_and_stitched_pub.publish(pc_img_msg);

    ros::spinOnce();

    ++counter;
  }
  disposeOccamAPI(occamAPI);

  return 0;
}
