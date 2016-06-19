#include "read_point_cloud.h"

void handleError(int returnCode) {
  if (returnCode != OCCAM_API_SUCCESS) {
    char errorMsg[30];
    occamGetErrorString((OccamError)returnCode, errorMsg, 30);
    fprintf(stderr, "Occam API Error: %d. %s\n", returnCode, errorMsg);
    abort();
  }
}

int convertToPcl(OccamPointCloud * occamPointCloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  int numPointsConverted = 0;
  int i;
  for (i = 0; i < 3*occamPointCloud->point_count; i+=3) {
    pcl::PointXYZRGBA* point = new pcl::PointXYZRGBA();

    point->r = occamPointCloud->rgb[i];
    point->g = occamPointCloud->rgb[i+1];
    point->b = occamPointCloud->rgb[i+2];
    point->a = 255;

    point->x = occamPointCloud->xyz[i];
    point->y = occamPointCloud->xyz[i+1];
    point->z = occamPointCloud->xyz[i+2];

    //printf("R: %d G: %d B: %d X: %f Y: %f Z: %f\n", point->r, point->g, point->b, point->x, point->y, point->z);

    double CULL_THRESHOLD = 1000;
    double inf = std::numeric_limits<double>::infinity();
    if (point->x < inf && point->y < inf && point->z < inf) {
      double sqdist = point->x*point->x + point->y*point->y + point->z * point->z;
      if (sqrt(sqdist) < CULL_THRESHOLD) {
        pclPointCloud->push_back(*point);
        numPointsConverted++;
      }
    }
  }

  pclPointCloud->is_dense = false;

  return numPointsConverted;
}

void savePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud, int counter) {
  double timestamp = pcl::getTime();
  std::stringstream ss;
  ss << "data/pointcloud" << counter << ".pcd";
  std::string name = ss.str();
  pcl::PCDWriter writer;
  if (point_cloud->size() > 0) {
    writer.write<pcl::PointXYZRGBA> (name, *point_cloud); 
  }
}

void occamImageToCvMat(OccamImage* &image, cv::Mat* &cvImage) {
  if (image && image->format == OCCAM_GRAY8) {
    cvImage = new cv::Mat_<uchar>(image->height, image->width, (uchar*)image->data[0], image->step[0]);
  } else if (image && image->format == OCCAM_RGB24) {
    cvImage =  new cv::Mat_<cv::Vec3b>(image->height, image->width, (cv::Vec3b*)image->data[0], image->step[0]);
    cv::Mat* colorImage = new cv::Mat();
    cv::cvtColor(*cvImage, *colorImage, cv::COLOR_BGR2RGB);
    cvImage = colorImage;
  } else if (image && image->format == OCCAM_SHORT1) {
    cvImage = new cv::Mat_<short>(image->height, image->width, (short*)image->data[0], image->step[0]);
  } else {
    printf("Image type not supported: %d\n", image->format);
    cvImage = NULL;
  }
}

void saveImage(OccamImage* image, std::string fileName) {
  cv::Mat* cvImage;
  occamImageToCvMat(image, cvImage);
  saveImage(cvImage,fileName);
}

void saveImage(cv::Mat* cvImage, std::string fileName) {
  imwrite(fileName, *cvImage);
}

void capturePointCloud(OccamDevice* device, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud, OccamDataName PCLOUD) {
  // Capture a point cloud
  OccamDataName requestTypes[] = { PCLOUD };
  OccamDataType returnTypes[] = { OCCAM_POINT_CLOUD };
  OccamPointCloud* pointCloud;
  handleError(occamDeviceReadData(device, 1, requestTypes, returnTypes, (void**)&pointCloud, 1));

  // Print statistics
  printf("Number of points in Occam point cloud: %d\n", pointCloud->point_count);

  // Convert to PCL point cloud
  int numConverted = convertToPcl(pointCloud, pclPointCloud);
  printf("Number of points converted to PCL: %d\n", numConverted);

  // Clean up
  handleError(occamFreePointCloud(pointCloud));
}

void captureAllPointClouds(OccamDevice* device, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  // Initialize variables
  int i;

  // Capture point clouds
  OccamDataName* requestTypes = (OccamDataName*)occamAlloc(5*sizeof(OccamDataName));
  for (i = 0; i < 5; ++i) {
    requestTypes[i] = (OccamDataName) (OCCAM_POINT_CLOUD0 + i);
  }
  OccamDataType returnTypes[] = { OCCAM_POINT_CLOUD };
  OccamPointCloud** pointClouds = (OccamPointCloud**)occamAlloc(5*sizeof(OccamPointCloud*));;
  handleError(occamDeviceReadData(device, 5, requestTypes, 0, (void**)pointClouds, 1));

  for (i = 0; i < 5; ++i) {
    // Print statistics
    printf("Number of points in OCCAM_POINT_CLOUD%d: %d\n", i, pointClouds[i]->point_count);

    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    int numConverted = convertToPcl(pointClouds[i], tempCloud);
    printf("Number of points converted to PCL: %d\n", numConverted);

    // Add to large cloud
    *pclPointCloud += *tempCloud;
  }
  printf("Number of points in large cloud: %zu\n", pclPointCloud->size());

  // Clean up
  occamFree(pointClouds);
}

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));   

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(pclPointCloud);             
  viewer->addPointCloud<pcl::PointXYZRGBA> (pclPointCloud, rgb, "cloud");
  while (!viewer->wasStopped()) {
    viewer->spinOnce();  
  }
}

OccamImage* captureImage(OccamDevice* device, OccamDataName requestType) {
  OccamDataName* req = (OccamDataName*)occamAlloc(sizeof(OccamDataName));
  req[0] = requestType;
  OccamImage** images = (OccamImage**)occamAlloc(sizeof(OccamImage*));
  handleError(occamDeviceReadData(device, 1, req, 0, (void**)images, 1));
  occamFree(req);
  return images[0];
}


void constructPointCloud(OccamDevice* device, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
  // Capture RBG image and disparity image
  //OccamImage* rgbImage = captureImage(device, OCCAM_IMAGE2);
  //printf("RGB Image captured at time: %llu\n", (long long unsigned int)rgbImage->time_ns);
  //OccamImage* disparityImage = captureImage(device, OCCAM_DISPARITY_IMAGE1);
  //printf("Disparity Image captured at time: %llu\n", (long long unsigned int)disparityImage->time_ns);

  // Save the images
  //saveImage(rgbImage, "rgbImage.jpg");
  //saveImage(disparityImage, "disparityImage.jpg");

  // Get basic sensor information for the camera
  int sensor_count;
  handleError(occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensor_count));
  int sensor_width;
  handleError(occamGetDeviceValuei(device, OCCAM_SENSOR_WIDTH, &sensor_width));
  int sensor_height;
  handleError(occamGetDeviceValuei(device, OCCAM_SENSOR_HEIGHT, &sensor_height));

  // Initialize sensor parameter variables
  double* Dp[sensor_count];
  double* Kp[sensor_count];
  double* Rp[sensor_count];
  double* Tp[sensor_count];

  // Get sensor parameters for all sensors
  for (int i = 0; i < sensor_count; ++i) {
    double D[5];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_DISTORTION_COEFS0 + i), D, 5));
    Dp[i] = D;
    double K[9];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_INTRINSICS0 + i), K, 9));
    Kp[i] = K;
    double R[9];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_ROTATION0 + i), R, 9));
    Rp[i] = R;
    double T[3];
    handleError(occamGetDeviceValuerv(device, OccamParam(OCCAM_SENSOR_TRANSLATION0 + i), T, 3));
    Tp[i] = T;
  }

  // Initialize interface to the Occam Stereo Rectify module
  void* rectifyHandle = 0;
  occamConstructModule(OCCAM_MODULE_STEREO_RECTIFY, "prec", &rectifyHandle);
  assert(rectifyHandle);
  IOccamStereoRectify* rectifyIface = 0;
  occamGetInterface(rectifyHandle, IOCCAMSTEREORECTIFY, (void**)&rectifyIface);
  assert(rectifyIface);

  // Configure the module with the sensor information
  handleError(rectifyIface->configure(rectifyHandle, sensor_count, sensor_width, sensor_height, Dp, Kp, Rp, Tp, 0));

  printf("i made it here\n");

  OccamImage* images = (OccamImage*)captureRgbAndDisparity(device);
  // printf("what happen\n");
  // // Get point cloud for the image
  // int indices[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  // OccamImage* rgbImages[10];
  // OccamImage* disparityImages[5];
  // printf("about to do somethign dangerous\n");
  // // dangerous stuff?
  // for (int i = 0; i < 5; i++) {
  //   rgbImages[i] = &images[i];
  //   disparityImages[i] = &images[i+5];
  // }
  // OccamPointCloud** pointClouds = (OccamPointCloud**)occamAlloc(sizeof(OccamPointCloud*) * 5);
  // handleError(rectifyIface->generateCloud(rectifyHandle, 1, indices, 1, rgbImages, disparityImages, pointClouds));

  // method signature
  // virtual int generateCloud(int N,const int* indices,int transform,
  // const OccamImage* const* img0,const OccamImage* const* disp0, OccamPointCloud** cloud1) = 0;


  // // Print statistics
  // for (int i = 0; i < 5; i++) {
  //   printf("Number of points in Occam point cloud: %d\n", pointClouds[i]->point_count);
  // }

  // // Convert to PCL point cloud
  // int numConverted = convertToPcl(pointCloud, pclPointCloud);
  // printf("Number of points converted to PCL: %d\n", numConverted);

  // // Clean up
  // for (int i = 0; i < 5; i++) {
  //   handleError(occamFreePointCloud(pointClouds[i]));
  // }
}

void** captureStitchedAndPointCloud(OccamDevice* device) {
  OccamDataName* req = (OccamDataName*)occamAlloc(6*sizeof(OccamDataName));
  req[0] = OCCAM_STITCHED_IMAGE0;
  req[1] = OCCAM_POINT_CLOUD0;
  req[2] = OCCAM_POINT_CLOUD1;
  req[3] = OCCAM_POINT_CLOUD2;
  req[4] = OCCAM_POINT_CLOUD3;
  req[5] = OCCAM_POINT_CLOUD4;
  OccamDataType returnTypes[] = {OCCAM_IMAGE, OCCAM_POINT_CLOUD};
  void** data = (void**)occamAlloc(sizeof(void*) * 6);
  handleError(occamDeviceReadData(device, 6, req, returnTypes, data, 1));
  return data;
}

void** captureRgbAndDisparity(OccamDevice* device) {
  printf("capturing rgb and disparity...\n");
  int num_images = 15;
  OccamDataName* req = (OccamDataName*)occamAlloc(num_images*sizeof(OccamDataName));
  req[0] = OCCAM_IMAGE0;
  req[1] = OCCAM_IMAGE1;
  req[2] = OCCAM_IMAGE2;
  req[3] = OCCAM_IMAGE3;
  req[4] = OCCAM_IMAGE4;
  req[5] = OCCAM_IMAGE5;
  req[6] = OCCAM_IMAGE6;
  req[7] = OCCAM_IMAGE7;
  req[8] = OCCAM_IMAGE8;
  req[9] = OCCAM_IMAGE9;
  req[10] = OCCAM_DISPARITY_IMAGE0;
  req[11] = OCCAM_DISPARITY_IMAGE1;
  req[12] = OCCAM_DISPARITY_IMAGE2;
  req[13] = OCCAM_DISPARITY_IMAGE3;
  req[14] = OCCAM_DISPARITY_IMAGE4;
  OccamDataType returnTypes[] = {OCCAM_IMAGE};
  void** data = (void**)occamAlloc(sizeof(OccamImage*) * num_images);
  printf("attempting to read rgb&disparity from occam...\n");
  handleError(occamDeviceReadData(device, 10, req, returnTypes, data, 1));
  printf("hah it worked");
  return data;
}

std::pair<OccamDevice*, OccamDeviceList*> initializeOccamAPI() {

  // Initialize Occam SDK
  handleError(occamInitialize());

  // Find all connected Occam devices
  OccamDeviceList* deviceList;
  handleError(occamEnumerateDeviceList(2000, &deviceList));
  int i;
  for (i = 0; i < deviceList->entry_count; ++i) {
    printf("%d. Device identifier: %s\n", i, deviceList->entries[i].cid);
  }

  // Connect to first device
  OccamDevice* device;
  char* cid = deviceList->entries[0].cid;
  handleError(occamOpenDevice(cid, &device));
  printf("Opened device: %p\n", device);
  return std::make_pair(device, deviceList);
}

void disposeOccamAPI(std::pair<OccamDevice*, OccamDeviceList*> occamAPI) {
  // Clean up
  handleError(occamCloseDevice(occamAPI.first));
  handleError(occamFreeDeviceList(occamAPI.second));
  handleError(occamShutdown());
}

// this is the externally facing API function! you want to use this
void getStitchedAndPointCloud(
    OccamDevice* device,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pc, 
    cv::Mat* &cvImage) {
  void** data = captureStitchedAndPointCloud(device);
  OccamImage* image = (OccamImage*)data[0];
  occamImageToCvMat(image, cvImage);
  handleError(occamFreeImage(image));

  for (int i = 1; i < 6; i++) {
    OccamPointCloud* occamCloud = (OccamPointCloud*)data[i];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    int numConverted = convertToPcl(occamCloud, tempCloud);
    printf("Number of points converted to PCL: %d\n", numConverted);

    // Add to large cloud
    *pc += *tempCloud;
  }
  for (int i = 1; i < 6; i++) {
    handleError(occamFreePointCloud((OccamPointCloud*)data[i]));
  }
}

int main(int argc, char** argv) {

  std::pair<OccamDevice*, OccamDeviceList*> occamAPI = initializeOccamAPI();
  OccamDevice* device = occamAPI.first;
  OccamDeviceList* deviceList = occamAPI.second;
  // Initialize viewer
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));

  // Display initial point cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  while (true) {
    constructPointCloud(device, cloud);
    printf("captured something\n");
  }
  /*
  void** data = captureStitchedAndPointCloud(device);
  OccamImage* image = (OccamImage*)data[0];
  OccamPointCloud* occamCloud = (OccamPointCloud*)data[1];
  convertToPcl(occamCloud, cloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "cloud");
  viewer->spinOnce();

  int counter = 0;
  // Keep updating point cloud until viewer is stopped
  cv::Mat* cvImage;
  while(!viewer->wasStopped()) {
    (*cloud).clear();

    constructPointCloud(device, cloud);
    getStitchedAndPointCloud(device, cloud, cvImage);

    savePointCloud(cloud, counter);
    std::ostringstream imagename;
    imagename << "data/stitched" << counter << ".jpg";
    saveImage(cvImage, imagename.str());

    rgb.setInputCloud(cloud);
    viewer->updatePointCloud(cloud, rgb, "cloud");
    viewer->spinOnce();
    ++counter;
  }
*/
  disposeOccamAPI(occamAPI);

  return 0;
}
