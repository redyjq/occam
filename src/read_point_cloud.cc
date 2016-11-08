#include "read_point_cloud.h"

using namespace cv;

OccamDevice* globalDevice = 0;
occam::OccamConfig config;

void handleError(int returnCode) {
  if (returnCode != OCCAM_API_SUCCESS) {
    char errorMsg[30];
    occamGetErrorString((OccamError)returnCode, errorMsg, 30);
    fprintf(stderr, "Occam API Error: %d. %s\n", returnCode, errorMsg);
    abort();
  }
}

int convertToPcl(OccamPointCloud *occamPointCloud, PointCloudT::Ptr pclPointCloud) {
  int numPointsConverted = 0;
  int i;
  for (i = 0; i < 3 * occamPointCloud->point_count; i += 3) {
    PointT *point = new PointT();

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

void savePointCloud(PointCloudT::Ptr point_cloud,
                    int counter) {
  double timestamp = pcl::getTime();
  std::stringstream ss;
  ss << "data/pointcloud" << counter << ".pcd";
  std::string name = ss.str();
  pcl::PCDWriter writer;
  if (point_cloud->size() > 0) {
    writer.write<PointT>(name, *point_cloud);
  }
}

Mat occamImageToCvMat(OccamImage *image) {
  Mat *cvImage;
  Mat colorImage;
  if (image && image->format == OCCAM_GRAY8) {
    cvImage = new Mat_<uchar>(image->height, image->width,
                                  (uchar *)image->data[0], image->step[0]);
  } else if (image && image->format == OCCAM_RGB24) {
    cvImage =
        new Mat_<Vec3b>(image->height, image->width,
                                (Vec3b *)image->data[0], image->step[0]);
    cvtColor(*cvImage, colorImage, COLOR_BGR2RGB);
  } else if (image && image->format == OCCAM_SHORT1) {
    cvImage = new Mat_<short>(image->height, image->width,
                                  (short *)image->data[0], image->step[0]);
  } else {
    printf("Image type not supported: %d\n", image->format);
    cvImage = NULL;
  }
  return colorImage;
}

void saveImage(OccamImage *image, std::string fileName) {
  Mat cvImage = occamImageToCvMat(image);
  saveImage(&cvImage, fileName);
}

void saveImage(Mat *cvImage, std::string fileName) {
  imwrite(fileName, *cvImage);
}

void capturePointCloud(OccamDevice *device,
                       PointCloudT::Ptr pclPointCloud,
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

void printDblArr(double A[], int size, std::string prefix, std::string delimiter, std::string suffix) {
    cout << prefix << std::scientific;
    for(int a=0; a<size; a++)
      if(a == size-1)
        cout << A[a];
      else
        cout << A[a] << delimiter;
    cout << suffix;
}

const int sensor_count = 5;
Eigen::Matrix4f extrisic_transforms[sensor_count];
void initSensorExtrisics(OccamDevice *device) {
  printf("#################################\n");
  for (int i = 0; i < sensor_count; ++i) {
    // Get sensor extrisics
    double R[9];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_ROTATION0 + i), R, 9));
    double T[3];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_TRANSLATION0 + i), T, 3));

    double K[9];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_INTRINSICS0 + i), K, 9));
    double D[5];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_DISTORTION_COEFS0 + i), D, 5));

    cout.precision(17);
    cout << "{\n";
    cout << "  752,\n";
    cout << "  480,\n";
    printDblArr(D, 5, "  {", ", ", "},\n");
    printDblArr(K, 9, "  {", ", ", "},\n");
    printDblArr(R, 9, "  {", ", ", "},\n");
    printDblArr(T, 3, "  {", ", ", "}\n");
    if(i == sensor_count-1)
      cout << "}\n";
    else
      cout << "},\n";
    
    // cout << "{" << std::scientific;
    // for(int k=0; k<9; k++)
    //   cout << K[k] << ",";
    // cout << "}," << endl;

    // cout << "{" << std::scientific;
    // for(int r=0; r<9; r++)
    //   cout << R[r] << ",";
    //   printf("%f,", R[r]);
    // printf("},\n");

    // printf("{");
    // for(int t=0; t<3; t++)
    //   printf("%f,", T[t]);
    // printf("}\n");

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

    // printf ("Transform for sensor %d:\n", i-1);
    // std::cout << extrisic_transforms[i] << std::endl;
  }
  printf("#################################\n");
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

Eigen::Matrix4f beam_occam_scale_transform;
Eigen::Matrix4f odom_beam_transform;
geometry_msgs::Pose odom_beam_pose;
void initTransforms() {
  // scale the cloud from cm to m
  double scale = 0.01;
  Eigen::Matrix4f scale_transform = Eigen::Matrix4f::Identity();
  scale_transform (0,0) = scale;
  scale_transform (1,1) = scale;
  scale_transform (2,2) = scale;

  // use the transform from the beam robot base to the occam frame to orient the cloud
  geometry_msgs::Pose beam_occam_pose;
  // done so that z axis points up, x axis points forward, y axis points left
  beam_occam_pose.orientation.x = -0.5;
  beam_occam_pose.orientation.y = 0.5;
  beam_occam_pose.orientation.z = -0.5;
  beam_occam_pose.orientation.w = 0.5;
  // beam_occam_pose.orientation.w = 1.0;
  //beam_occam_pose.position.z = 1.658;  
  beam_occam_pose.position.z = 1.10;  
  beam_occam_pose.position.x = -0.140;  
  Eigen::Matrix4f beam_occam_transform = transform_from_pose(beam_occam_pose);
  // Eigen::Matrix4f beam_occam_transform = Eigen::Matrix4f::Identity();

  // combine the transforms
  beam_occam_scale_transform = beam_occam_transform * scale_transform;

  // if no odom recieved, assume identity transform
  odom_beam_transform = Eigen::Matrix4f::Identity();

  printf("Initialized Transforms.\n");
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Update the matrix used to transform the pointcloud to the odom frame
  odom_beam_pose = msg->pose.pose;
  odom_beam_transform = transform_from_pose(odom_beam_pose);  
}

void occamCloudsToPCL(
  OccamPointCloud** pointClouds,
  PointCloudT::Ptr pclPointCloud) {
  // use latest odom data to transform the cloud with the movement of the robot
  Eigen::Matrix4f odom_occam_transform = odom_beam_transform * beam_occam_scale_transform;
  for (int i = 0; i < sensor_count; ++i) {
    // Print statistics
    // printf("Number of points in OCCAM_POINT_CLOUD%d: %d\n", i, pointClouds[i]->point_count);

    // Convert to PCL point cloud
    PointCloudT::Ptr tempCloud(new PointCloudT);
    int numConverted = convertToPcl(pointClouds[i], tempCloud);
    // printf("Number of points converted to PCL: %d\n", numConverted);

    // combine extrisic transform and then apply to the cloud
    Eigen::Matrix4f combined_transform = odom_occam_transform * extrisic_transforms[i];
    PointCloudT::Ptr transformedCloud (new PointCloudT);
    pcl::transformPointCloud (*tempCloud, *transformedCloud, combined_transform);

    // Add to large cloud
    *pclPointCloud += *transformedCloud;
  }
  // printf("Number of points in large cloud: %zu\n", pclPointCloud->size());
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
    PointCloudT::Ptr pclPointCloud) {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("PCL Viewer"));

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(
      pclPointCloud);
  viewer->addPointCloud<PointT>(pclPointCloud, rgb, "cloud");
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
    PointCloudT::Ptr pclPointCloud) {
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
    printf("K:\n");
    for(int a=0; a<3; a++) {
      for(int b=0; b<3; b++) {
        printf("%f,\t", K[3*a+b]);
      }
      printf("\n");
    }
    printf("\n");
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

// void **captureRgbAndPointCloud(OccamDevice *device) {
//   OccamDataName *req = (OccamDataName *)occamAlloc(10 * sizeof(OccamDataName));
//   req[0] = OCCAM_IMAGE0;
//   req[1] = OCCAM_IMAGE1;
//   req[2] = OCCAM_IMAGE2;
//   req[3] = OCCAM_IMAGE3;
//   req[4] = OCCAM_IMAGE4;
//   req[5] = OCCAM_POINT_CLOUD0;
//   req[6] = OCCAM_POINT_CLOUD1;
//   req[7] = OCCAM_POINT_CLOUD2;
//   req[8] = OCCAM_POINT_CLOUD3;
//   req[9] = OCCAM_POINT_CLOUD4;
//   OccamDataType returnTypes[] = {OCCAM_IMAGE, OCCAM_POINT_CLOUD};
//   void **data = (void **)occamAlloc(sizeof(void *) * 10);
//   handleError(occamDeviceReadData(device, 10, req, returnTypes, data, 1));
//   return data;
// }

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

void getRGBPointCloudOdom(OccamDevice *device, PointCloudT::Ptr pclPointCloud, Mat imgs[], geometry_msgs::Pose *odom_beam_pose_out) {

  // void **data = captureRgbAndPointCloud(device);

  OccamDataName *req_pc = (OccamDataName *)occamAlloc(sensor_count * sizeof(OccamDataName));
  req_pc[0] = OCCAM_POINT_CLOUD0;
  req_pc[1] = OCCAM_POINT_CLOUD1;
  req_pc[2] = OCCAM_POINT_CLOUD2;
  req_pc[3] = OCCAM_POINT_CLOUD3;
  req_pc[4] = OCCAM_POINT_CLOUD4;
  OccamDataType returnTypes_pc[] = {OCCAM_POINT_CLOUD};
  void **data_pc;
  int resp_pc = -1;
  while(resp_pc != 0) {
    data_pc = (void **)occamAlloc(sizeof(void *) * sensor_count);
    resp_pc = occamDeviceReadData(device, sensor_count, req_pc, returnTypes_pc, data_pc, 1);
    // printf("resp_pc: %d #########################\n", resp_pc);
    // handleError(resp_pc);
  }

  OccamDataName *req_img = (OccamDataName *)occamAlloc(sensor_count * sizeof(OccamDataName));
  req_img[0] = OCCAM_IMAGE0;
  req_img[1] = OCCAM_IMAGE2;
  req_img[2] = OCCAM_IMAGE4;
  req_img[3] = OCCAM_IMAGE6;
  req_img[4] = OCCAM_IMAGE8;
  // req_img[0] = OCCAM_RECTIFIED_IMAGE0;
  // req_img[1] = OCCAM_RECTIFIED_IMAGE2;
  // req_img[2] = OCCAM_RECTIFIED_IMAGE4;
  // req_img[3] = OCCAM_RECTIFIED_IMAGE6;
  // req_img[4] = OCCAM_RECTIFIED_IMAGE8;
  OccamDataType returnTypes_img[] = {OCCAM_IMAGE};
  void **data_img;
  int resp_img = -1;
  while(resp_img != 0) {
    data_img = (void **)occamAlloc(sizeof(void *) * sensor_count);
    resp_img = occamDeviceReadData(device, sensor_count, req_img, returnTypes_img, data_img, 1);
    // printf("resp_img: %d #########################\n", resp_img);
  }

  for (int i = 0; i < sensor_count; ++i) {
    Mat img = occamImageToCvMat((OccamImage *)data_img[i]);
    // printf("image %d: r: %d c:%d\n", i, img.rows, img.cols);
    // // rotate rectified img
    // flip(img.t(), img, 1);
    // flip(img, img, 1);
    imgs[i] = img.clone();
  }

  // use latest odom data to transform the cloud with the movement of the robot
  *odom_beam_pose_out = odom_beam_pose;
  Eigen::Matrix4f odom_occam_transform = odom_beam_transform * beam_occam_scale_transform;
  for (int i = 0; i < sensor_count; ++i) {
    OccamPointCloud *occamCloud = (OccamPointCloud *)data_pc[i];
    // Convert to PCL point cloud
    PointCloudT::Ptr tempCloud(new PointCloudT);
    int numConverted = convertToPcl(occamCloud, tempCloud);

    if(config.filtering_enabled && config.crop_box_filter) {
        // Crop out far away points
        Eigen::Vector4f minP, maxP;
        float inf = std::numeric_limits<float>::infinity();
        minP[0] = -inf; minP[1] = -inf; minP[2] = 0.0;
        maxP[0] = inf; maxP[1] = inf; maxP[2] = config.max_dist*100;
        pcl::CropBox<PointT> cropFilter;
        cropFilter.setInputCloud (tempCloud);
        cropFilter.setMin(minP);
        cropFilter.setMax(maxP);
        cropFilter.setKeepOrganized(false);
        cropFilter.filter (*tempCloud);
    }

    // combine extrisic transform and then apply to the cloud
    Eigen::Matrix4f combined_transform = odom_occam_transform * extrisic_transforms[i];
    PointCloudT::Ptr transformedCloud (new PointCloudT);
    pcl::transformPointCloud (*tempCloud, *transformedCloud, combined_transform);
    // Add to large cloud
    *pclPointCloud += *transformedCloud;
  }
  for (int i = 0; i < sensor_count; ++i) {
    handleError(occamFreePointCloud((OccamPointCloud *)data_pc[i]));
    handleError(occamFreeImage((OccamImage *)data_img[i]));
  }
}

Mat getStitchedAndPointCloud(OccamDevice *device,
                              PointCloudT::Ptr pclPointCloud) {
  void **data = captureStitchedAndPointCloud(device);
  OccamImage *image = (OccamImage *)data[0];
  Mat img = occamImageToCvMat(image);
  handleError(occamFreeImage(image));

  // use latest odom data to transform the cloud with the movement of the robot
  Eigen::Matrix4f odom_occam_transform = odom_beam_transform * beam_occam_scale_transform;
  for (int i = 0; i < sensor_count; ++i) {
    OccamPointCloud *occamCloud = (OccamPointCloud *)data[i+1];
    // Print statistics
    // printf("Number of points in OCCAM_POINT_CLOUD%d: %d\n", i, occamCloud->point_count);

    // Convert to PCL point cloud
    PointCloudT::Ptr tempCloud(new PointCloudT);
    int numConverted = convertToPcl(occamCloud, tempCloud);
    // printf("Number of points converted to PCL: %d\n", numConverted);

    // Downsample the pointcloud
    // pcl::VoxelGrid<PointT> vgf;
    // vgf.setInputCloud (tempCloud);
    // float leaf_size = 0.01f;
    // vgf.setLeafSize (leaf_size, leaf_size, leaf_size);
    // vgf.filter (*tempCloud);

    // combine extrisic transform and then apply to the cloud
    Eigen::Matrix4f combined_transform = odom_occam_transform * extrisic_transforms[i];
    PointCloudT::Ptr transformedCloud (new PointCloudT);
    pcl::transformPointCloud (*tempCloud, *transformedCloud, combined_transform);
    
    // Add to large cloud
    *pclPointCloud += *transformedCloud;
  }
  for (int i = 0; i < sensor_count; ++i) {
    handleError(occamFreePointCloud((OccamPointCloud *)data[i+1]));
  }
  // printf("Number of points in large cloud: %zu\n", pclPointCloud->size());
  return img;
}

void changeParam(OccamParam param, std::string paramName, int newVal) {
  if(!globalDevice)
    return;
  int value; occamGetDeviceValuei(globalDevice, param, &value);
  if(value != newVal) {
    occamSetDeviceValuei(globalDevice, param, newVal);
    // int newvalue; occamGetDeviceValuei(globalDevice, param, &newvalue);
    printf("################### %s changed to %d ###################\n", paramName.c_str(), newVal);
  }
}

void config_callback(occam::OccamConfig &c, uint32_t level) {
  // set all changed parameters
  ROS_INFO("Occam Reconfigure Request");
  config = c;
  changeParam(OCCAM_PREFERRED_BACKEND, "OCCAM_PREFERRED_BACKEND", c.OCCAM_PREFERRED_BACKEND);
  changeParam(OCCAM_AUTO_EXPOSURE, "OCCAM_AUTO_EXPOSURE", c.OCCAM_AUTO_EXPOSURE);
  changeParam(OCCAM_AUTO_GAIN, "OCCAM_AUTO_GAIN", c.OCCAM_AUTO_GAIN);
  changeParam(OCCAM_BM_PREFILTER_TYPE, "OCCAM_BM_PREFILTER_TYPE", c.OCCAM_BM_PREFILTER_TYPE);
  changeParam(OCCAM_BM_PREFILTER_SIZE, "OCCAM_BM_PREFILTER_SIZE", c.OCCAM_BM_PREFILTER_SIZE);
  changeParam(OCCAM_BM_PREFILTER_CAP, "OCCAM_BM_PREFILTER_CAP", c.OCCAM_BM_PREFILTER_CAP);
  changeParam(OCCAM_BM_SAD_WINDOW_SIZE, "OCCAM_BM_SAD_WINDOW_SIZE", c.OCCAM_BM_SAD_WINDOW_SIZE);
  changeParam(OCCAM_BM_MIN_DISPARITY, "OCCAM_BM_MIN_DISPARITY", c.OCCAM_BM_MIN_DISPARITY);
  changeParam(OCCAM_BM_NUM_DISPARITIES, "OCCAM_BM_NUM_DISPARITIES", c.OCCAM_BM_NUM_DISPARITIES);
  changeParam(OCCAM_BM_TEXTURE_THRESHOLD, "OCCAM_BM_TEXTURE_THRESHOLD", c.OCCAM_BM_TEXTURE_THRESHOLD);
  changeParam(OCCAM_BM_UNIQUENESS_RATIO, "OCCAM_BM_UNIQUENESS_RATIO", c.OCCAM_BM_UNIQUENESS_RATIO);
  changeParam(OCCAM_BM_SPECKLE_RANGE, "OCCAM_BM_SPECKLE_RANGE", c.OCCAM_BM_SPECKLE_RANGE);
  changeParam(OCCAM_BM_SPECKLE_WINDOW_SIZE, "OCCAM_BM_SPECKLE_WINDOW_SIZE", c.OCCAM_BM_SPECKLE_WINDOW_SIZE);
  changeParam(OCCAM_FILTER_LAMBDA, "OCCAM_FILTER_LAMBDA", c.OCCAM_FILTER_LAMBDA);
  changeParam(OCCAM_FILTER_SIGMA, "OCCAM_FILTER_SIGMA", c.OCCAM_FILTER_SIGMA);
  changeParam(OCCAM_FILTER_DDR, "OCCAM_FILTER_DDR", c.OCCAM_FILTER_DDR);
}

int main(int argc, char **argv) {

  // Init ROS node
  ros::init(argc, argv, "beam_occam");
  ros::NodeHandle n;
  printf("Initialized ROS node.\n");

  // Init dynamic reconfigure param server
  dynamic_reconfigure::Server<occam::OccamConfig> server;
  dynamic_reconfigure::Server<occam::OccamConfig>::CallbackType f;
  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  // Subscribe to odometry data
  ros::Subscriber odom_sub = n.subscribe("/beam/odom", 1, odomCallback);
  // PointCloud2 publisher
  // ros::Publisher pc2_pub = n.advertise<sensor_msgs::PointCloud2>("/occam/points", 1);
  ros::Publisher pc2_and_stitched_pub = n.advertise<beam_joy::PointcloudImagePose>("/occam/points_rgb_odom", 1);

  // image_transport::ImageTransport it(n);
  // image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Publisher stitched_pub = n.advertise<sensor_msgs::Image>("/occam/stitched", 1);

  std::pair<OccamDevice *, OccamDeviceList *> occamAPI = initializeOccamAPI();
  OccamDevice *device = occamAPI.first;
  globalDevice = device;
  OccamDeviceList *deviceList = occamAPI.second;  

  // Enable auto exposure and gain **important**
  occamSetDeviceValuei(device, OCCAM_AUTO_EXPOSURE, 1);
  occamSetDeviceValuei(device, OCCAM_AUTO_GAIN, 1);

  PointCloudT::Ptr cloud(new PointCloudT);
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  // initialize global constants
  initSensorExtrisics(device);
  initTransforms();
  
  Mat cvImage;
  int counter = 0;
  while (ros::ok()) {
    (*cloud).clear();

    ros::Time capture_time = ros::Time::now();
    // cvImage = getStitchedAndPointCloud(device, cloud);
    Mat imgs[sensor_count];
    geometry_msgs::Pose odom_beam_pose_out;
    getRGBPointCloudOdom(device, cloud, imgs, &odom_beam_pose_out);
    // cout << odom_beam_pose_out << " $$$$$$$$$$$$$$$$" << endl;

    for (int i = 0; i < sensor_count; ++i) {
      std::stringstream index;
      index << i;
      imwrite("img/rgb/rgb"+index.str()+".jpg", imgs[i]);
    }


    printf("Cloud size: %lu\n", cloud->size());
    clock_t start;
    if(config.filtering_enabled) {
        if(config.crop_box_filter) {
            // Crop out the floor and ceiling and points farther than dist
            Eigen::Vector4f minP, maxP;
            float inf = std::numeric_limits<float>::infinity();
            minP[0] = -inf; minP[1] = -inf; minP[2] = config.min_z;
            maxP[0] = inf; maxP[1] = inf; maxP[2] = config.max_z; 
            pcl::CropBox<PointT> cropFilter;
            cropFilter.setInputCloud (cloud);
            cropFilter.setMin(minP);
            cropFilter.setMax(maxP);
            cropFilter.setKeepOrganized(false);
            cropFilter.filter (*cloud);
        }

        if(config.voxel_grid_filter) {
            // Downsample the pointcloud
            pcl::VoxelGrid<PointT> vgf;
            vgf.setInputCloud (cloud);
            vgf.setLeafSize (config.leaf_size, config.leaf_size, config.leaf_size);
            vgf.filter (*cloud);
        }

        if(config.plane_removal_filter) {
            // Remove the ground using the given plane coefficients 
            Eigen::Vector4f gc;   
            gc[0] = 0.0;
            gc[1] = 0.0;
            gc[2] = -1.0;
            gc[3] = 0.0;
            pcl::SampleConsensusModelPlane<PointT>::Ptr dit (new pcl::SampleConsensusModelPlane<PointT> (cloud));
            std::vector<int> ground_inliers;
            dit->selectWithinDistance (gc, config.plane_dist_thresh, ground_inliers);
            pcl::PointIndices::Ptr ground_ptr (new pcl::PointIndices);
            ground_ptr->indices = ground_inliers;   
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud (cloud);  
            extract.setIndices (ground_ptr);  
            extract.setNegative (true);
            extract.setKeepOrganized(false);
            extract.filter (*cloud);
                
            // Remove other planes such as walls
            //ransacRemoveMultiple (cloud, config.plane_dist_thresh, 100, 500); 
        }
            
        if(config.statistical_outlier_filter) {
            // Remove statistical outliers to make point cloud cleaner
            pcl::StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud (cloud);
            sor.setMeanK (config.outlier_num_points);
            sor.setStddevMulThresh (config.outlier_std_dev);
            sor.setKeepOrganized(false);
            sor.filter (*cloud);
        }
            
        if(config.radius_outlier_filter) {
            // Remove radius outliers to make point cloud cleaner
            pcl::RadiusOutlierRemoval<PointT> outrem;
            outrem.setInputCloud(cloud);
            outrem.setRadiusSearch(config.radius_search);
            outrem.setMinNeighborsInRadius(config.radius_neighbors);
            outrem.setKeepOrganized(false);
            outrem.filter (*cloud);
        }

        // Remove NAN from cloud
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

        printf("Cloud size after filtering: %lu\n", cloud->size());
    }
        
    // Init PointcloudImagePose msg
    beam_joy::PointcloudImagePose pc_img_odom_msg;

    // Add beam odom to msg
    pc_img_odom_msg.odom = odom_beam_pose_out;

    // Convert cvImage to ROS Image msg
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImage).toImageMsg();
    img_msg->header.frame_id = "occam_optical_link";
    img_msg->header.stamp = capture_time;
    // pc_img_odom_msg.img = *img_msg;
    
    for (int i = 0; i < sensor_count; ++i) {
      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgs[i]).toImageMsg();
      img_msg->header.frame_id = "occam_optical_link";
      img_msg->header.stamp = capture_time;
      pc_img_odom_msg.imgs.push_back(*img_msg);
    }

    // Convert PCL pointcloud to ROS PointCloud2 msg
    sensor_msgs::PointCloud2 pc2;
    pcl::PCLPointCloud2 tmp_cloud;
    pcl::toPCLPointCloud2(*cloud, tmp_cloud);
    // pcl::toPCLPointCloud2(*cloud_filtered, tmp_cloud);
    pcl_conversions::fromPCL(tmp_cloud, pc2);
    pc2.header.frame_id = "odom";
    pc2.header.stamp = capture_time;
    pc_img_odom_msg.pc = pc2;

    // Publish all msgs
    // stitched_pub.publish(img_msg);
    // pc2_pub.publish(pc2);
    pc2_and_stitched_pub.publish(pc_img_odom_msg);

    ros::spinOnce();

    ++counter;
  }
  disposeOccamAPI(occamAPI);

  return 0;
}
