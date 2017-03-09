#include "read_point_cloud.h"

using namespace std;
using namespace cv;

OccamDevice* globalDevice = 0;
occam::OccamConfig config;
const int sensor_count = 5;
Eigen::Matrix4f extrisic_transforms[sensor_count];
Eigen::Matrix4f beam_occam_scale_transform;
Eigen::Matrix4f odom_beam_transform;
// geometry_msgs::Pose odom_beam_pose;
std::deque<nav_msgs::Odometry> odom_msgs;
nav_msgs::Odometry odom_msg;
ros::Publisher pc_rgb_odom_pub, odom_req;
// pthread_mutex_t mutex;
// pthread_cond_t cond;
// bool condition = true;

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
  PointT point = PointT();
  uint8_t* rgb = occamPointCloud->rgb;
  float* xyz = occamPointCloud->xyz;
  int point_count = occamPointCloud->point_count;
  for (i = 0; i < 3 * point_count; i += 3) {

    point.r = rgb[i];
    point.g = rgb[i + 1];
    point.b = rgb[i + 2];
    point.a = 255;

    point.x = xyz[i];
    point.y = xyz[i + 1];
    point.z = xyz[i + 2];

    pclPointCloud->push_back(point);
    numPointsConverted++;
  }
  pclPointCloud->is_dense = false;
  return numPointsConverted;
}


Mat occamImageToCvMat(OccamImage *image) {
  Mat *cvImage;
  if (image && image->format == OCCAM_GRAY8) {
    cvImage = new Mat_<uchar>(image->height, image->width,
                                  (uchar *)image->data[0], image->step[0]);
  } else if (image && image->format == OCCAM_RGB24) {
    cvImage =
        new Mat_<Vec3b>(image->height, image->width,
                                (Vec3b *)image->data[0], image->step[0]);
    cvtColor(*cvImage, *cvImage, COLOR_BGR2RGB);
  } else if (image && image->format == OCCAM_SHORT1) {
    cvImage = new Mat_<short>(image->height, image->width,
                                  (short *)image->data[0], image->step[0]);
  } else {
    printf("Image type not supported: %d\n", image->format);
    cvImage = NULL;
  }
  return *cvImage;
}

void printDblArr(double A[], int size, string prefix, string delimiter, string suffix) {
    cout << prefix << scientific;
    for(int a=0; a<size; a++)
      if(a == size-1)
        cout << A[a];
      else
        cout << A[a] << delimiter;
    cout << suffix;
}

void initSensorExtrisics(OccamDevice *device) {
  // printf("#################################\n");
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

    // cout.precision(17);
    // cout << "{\n";
    // cout << "  752,\n";
    // cout << "  480,\n";
    // printDblArr(D, 5, "  {", ", ", "},\n");
    // printDblArr(K, 9, "  {", ", ", "},\n");
    // printDblArr(R, 9, "  {", ", ", "},\n");
    // printDblArr(T, 3, "  {", ", ", "}\n");
    // if(i == sensor_count-1)
    //   cout << "}\n";
    // else
    //   cout << "},\n";

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
    // cout << extrisic_transforms[i] << endl;
  }
  // printf("#################################\n");
  printf("Initialized Sensor Extrisics.\n");
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

void initTransforms() {
  // scale the cloud from cm to m
  Eigen::Matrix4f scale_transform = Eigen::Matrix4f::Identity();
  scale_transform (0,0) = config.scale;
  scale_transform (1,1) = config.scale;
  scale_transform (2,2) = config.scale;

  // use the transform from the beam robot base to the occam frame to orient the cloud
  geometry_msgs::Pose beam_occam_pose;
  // done so that z axis points up, x axis points forward, y axis points left
  beam_occam_pose.orientation.x = -0.5;
  beam_occam_pose.orientation.y = 0.5;
  beam_occam_pose.orientation.z = -0.5;
  beam_occam_pose.orientation.w = 0.5;
  beam_occam_pose.position.z = config.beam_occam_pos_z;  
  beam_occam_pose.position.x = config.beam_occam_pos_x;  
  Eigen::Matrix4f beam_occam_transform = transform_from_pose(beam_occam_pose);
  // Eigen::Matrix4f beam_occam_transform = Eigen::Matrix4f::Identity();

  // combine the transforms
  beam_occam_scale_transform = beam_occam_transform * scale_transform;

  // if no odom recieved, assume identity transform
  odom_beam_transform = Eigen::Matrix4f::Identity();

  printf("Initialized Transforms.\n");
}

nav_msgs::Odometry getClosestOdom(ros::Time offset_time) {
      nav_msgs::Odometry new_odom;
      double min_diff = 10.0;
      if(odom_msgs.empty())
          new_odom.pose.pose.orientation.w = 1.0;

      for (std::deque<nav_msgs::Odometry>::iterator it = odom_msgs.begin(); it != odom_msgs.end(); ++it) {
          nav_msgs::Odometry odom = *it;
          double diff = (offset_time - odom.header.stamp).toSec();
          // printf("diff: %.2f\n", diff);
          if(fabs(diff) < fabs(min_diff)) {
              new_odom = odom;
              min_diff = diff;
          }
      }

      // if(fabs(min_diff) > 0.1)
      //     ROS_WARN("min_diff is large: %.2f; couldn't find correctly timed odom", min_diff);

      return new_odom;
}

void odomCallback(nav_msgs::Odometry msg) {
  // if(config.cond_wait)
  //   pthread_mutex_lock(&mutex);

  odom_msg = msg;
  odom_msgs.push_back(msg);

  while((int)odom_msgs.size() > config.odom_queue_size)
      odom_msgs.pop_front();

  // if(config.cond_wait) {
  //   condition = true;
  //   pthread_cond_signal(&cond);
  //   pthread_mutex_unlock(&mutex);
  // }

  // odom_beam_pose = msg.pose.pose;
  // Update the matrix used to transform the pointcloud to the odom frame
  // odom_beam_transform = transform_from_pose(odom_beam_pose);
}

pair<OccamDevice *, OccamDeviceList *> initializeOccamAPI() {

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
  return make_pair(device, deviceList);
}

void disposeOccamAPI(pair<OccamDevice *, OccamDeviceList *> occamAPI) {
  // Clean up
  handleError(occamCloseDevice(occamAPI.first));
  handleError(occamFreeDeviceList(occamAPI.second));
  handleError(occamShutdown());
}

void **captureRgbAndPointCloud(OccamDevice *device) {
  int num_data = sensor_count * 2;
  OccamDataName *req = (OccamDataName *)occamAlloc(num_data * sizeof(OccamDataName));
  req[0] = OCCAM_IMAGE0;
  req[1] = OCCAM_IMAGE1;
  req[2] = OCCAM_IMAGE2;
  req[3] = OCCAM_IMAGE3;
  req[4] = OCCAM_IMAGE4;
  req[5] = OCCAM_POINT_CLOUD0;
  req[6] = OCCAM_POINT_CLOUD1;
  req[7] = OCCAM_POINT_CLOUD2;
  req[8] = OCCAM_POINT_CLOUD3;
  req[9] = OCCAM_POINT_CLOUD4;
  OccamDataType returnTypes[] = {OCCAM_IMAGE, OCCAM_POINT_CLOUD};
  void **data = (void **)occamAlloc(sizeof(void *) * num_data);
  handleError(occamDeviceReadData(device, num_data, req, returnTypes, data, 1));
  return data;
}

void getRGBPointCloudOdom(OccamDevice *device, PointCloudT::Ptr pclPointCloud, Mat imgs[], geometry_msgs::Pose *odom_beam_pose_out) {
  // request odom from the beam
  // condition = false;
  odom_req.publish(std_msgs::Empty());

  ros::Time pc_capture_time = ros::Time::now();
  ros::Duration time_offset(config.odom_time_offset);
  // calc new offset time
  ros::Time offset_time = pc_capture_time + time_offset;

  // void **data = captureRgbAndPointCloud(device);
  OccamDataName *req_img = (OccamDataName *)occamAlloc(sensor_count * sizeof(OccamDataName));
  req_img[0] = OCCAM_IMAGE0;
  req_img[1] = OCCAM_IMAGE2;
  req_img[2] = OCCAM_IMAGE4;
  req_img[3] = OCCAM_IMAGE6;
  req_img[4] = OCCAM_IMAGE8;
  OccamDataType returnTypes_img[] = {OCCAM_IMAGE};
  void **data_img;
  int resp_img = -1;
  while(ros::ok() && resp_img != 0) {
    data_img = (void **)occamAlloc(sizeof(void *) * sensor_count);
    resp_img = occamDeviceReadData(device, sensor_count, req_img, returnTypes_img, data_img, 1);
  }

  OccamDataName *req_pc = (OccamDataName *)occamAlloc(sensor_count * sizeof(OccamDataName));
  req_pc[0] = OCCAM_POINT_CLOUD0;
  req_pc[1] = OCCAM_POINT_CLOUD1;
  req_pc[2] = OCCAM_POINT_CLOUD2;
  req_pc[3] = OCCAM_POINT_CLOUD3;
  req_pc[4] = OCCAM_POINT_CLOUD4;
  OccamDataType returnTypes_pc[] = {OCCAM_POINT_CLOUD};
  void **data_pc;
  int resp_pc = -1;
  while(ros::ok() && resp_pc != 0) {
    data_pc = (void **)occamAlloc(sizeof(void *) * sensor_count);
    resp_pc = occamDeviceReadData(device, sensor_count, req_pc, returnTypes_pc, data_pc, 1);
  }

  for (int i = 0; i < sensor_count; ++i) {
    imgs[i] = (occamImageToCvMat((OccamImage *)data_img[i])).clone();
  }

  // int timeout = 500;
  // struct timespec delta, abstime;
  // delta.tv_sec = 0;
  // // convert milliseconds to nanoseconds.
  // delta.tv_nsec = timeout * 1000 * 1000 ;

  // int status = pthread_get_expiration_np(&delta,&abstime);

  // if (status == -1) {
  //   ROS_ERROR("pthread_get_expiration_np() ");
  // }

  // pthread_mutex_lock(&mutex);

  // while (ros::ok() && !condition) {
  //   status = pthread_cond_timedwait(&cond, &mutex, &abstime);
  //   if (status == -1) {
  //     ROS_ERROR("TIMEOUT!\n");
  //   }
  // }

  // if(config.cond_wait) {
  //   pthread_mutex_lock(&mutex);
  //   while (ros::ok() && !condition)
  //     pthread_cond_wait(&cond, &mutex);
  // }

  // TODO: sleep or wait for odom

  // nav_msgs::Odometry close_odom = odom_msg;

  // match with the msg closest to the desired offset time
  nav_msgs::Odometry close_odom = getClosestOdom(offset_time);
  double time_diff = (offset_time - close_odom.header.stamp).toSec();
  if(fabs(time_diff) > .05)
    ROS_WARN("time_diff: %.2f", time_diff);
  // else
  //   ROS_INFO("time_diff: %.2f", time_diff);

  // use correctly timed odom data to transform the cloud with the movement of the robot
  geometry_msgs::Pose odom_beam_pose = close_odom.pose.pose;
  odom_beam_transform = transform_from_pose(odom_beam_pose);
  *odom_beam_pose_out = odom_beam_pose;

  // if(config.cond_wait)
  //   pthread_mutex_unlock(&mutex);

  Eigen::Matrix4f odom_occam_transform = odom_beam_transform * beam_occam_scale_transform;
  for (int i = 0; i < sensor_count; ++i) {
    OccamPointCloud *occamCloud = (OccamPointCloud *)data_pc[i];
    // Convert to PCL point cloud
    PointCloudT::Ptr sub_cloud(new PointCloudT);
    int numConverted = convertToPcl(occamCloud, sub_cloud);

    if(config.filtering_enabled && config.crop_box_filter) {
      // Crop out far away points
      Eigen::Vector4f minP, maxP;
      float inf = numeric_limits<float>::infinity();
      minP[0] = -inf; minP[1] = -inf; minP[2] = 0.0;
      maxP[0] = inf; maxP[1] = inf; maxP[2] = config.max_dist / config.scale;
      pcl::CropBox<PointT> cropFilter;
      cropFilter.setInputCloud (sub_cloud);
      cropFilter.setMin(minP);
      cropFilter.setMax(maxP);
      cropFilter.setKeepOrganized(false);
      cropFilter.filter (*sub_cloud);
    }

    // combine extrisic transform and then apply to the cloud
    Eigen::Matrix4f combined_transform = odom_occam_transform * extrisic_transforms[i];
    pcl::transformPointCloud (*sub_cloud, *sub_cloud, combined_transform);
    // Add to large cloud
    *pclPointCloud += *sub_cloud;
  }
  for (int i = 0; i < sensor_count; ++i) {
    handleError(occamFreePointCloud((OccamPointCloud *)data_pc[i]));
    handleError(occamFreeImage((OccamImage *)data_img[i]));
  }
}

void changeParam(OccamParam param, string paramName, int newVal) {
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

  initTransforms();
}

void capture() {
  PointCloudT::Ptr cloud(new PointCloudT);

  // int count = 0;
  // int num_iter = 50;
  // ProfilerStart("prof/profile.log");
  while (ros::ok()) {
    // if(count++ > num_iter) { break; }
    (*cloud).clear();

    Mat imgs[sensor_count];
    geometry_msgs::Pose odom_beam_pose_out;

    ros::Time capture_time = ros::Time::now();
    getRGBPointCloudOdom(globalDevice, cloud, imgs, &odom_beam_pose_out);

    printf("Cloud size: %lu\n", cloud->size());
    if(config.filtering_enabled) {
        if(config.crop_box_filter) {
            // Crop out the floor and ceiling and points farther than dist
            Eigen::Vector4f minP, maxP;
            float inf = numeric_limits<float>::infinity();
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
            vector<int> ground_inliers;
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
        vector<int> index;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, index);

        printf("Cloud size after filtering: %lu\n", cloud->size());
    }
        
    // Init PointcloudImagePose msg
    beam_joy::PointcloudImagePose pc_img_odom_msg;

    // Add beam odom to msg
    pc_img_odom_msg.odom = odom_beam_pose_out;

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
    pcl_conversions::fromPCL(tmp_cloud, pc2);
    pc2.header.frame_id = "odom";
    pc2.header.stamp = capture_time;
    pc_img_odom_msg.pc = pc2;

    // Publish PointcloudImagePose
    pc_rgb_odom_pub.publish(pc_img_odom_msg);

    ros::spinOnce();
  }
}

int main(int argc, char **argv) {

  // Init ROS node
  ros::init(argc, argv, "beam_occam");
  ros::NodeHandle n;
  ROS_INFO("Initialized ROS node.");

  // Init dynamic reconfigure param server
  dynamic_reconfigure::Server<occam::OccamConfig> server;
  dynamic_reconfigure::Server<occam::OccamConfig>::CallbackType f;
  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);

  // Subscribe to odometry data
  ros::Subscriber odom_sub = n.subscribe("/beam/odom", 1, odomCallback);
  // PointcloudImagePose publisher
  pc_rgb_odom_pub = n.advertise<beam_joy::PointcloudImagePose>("/occam/points_rgb_odom", 1);
  
  odom_req = n.advertise<std_msgs::Empty>("/beam/publish_odom", 1);

  ROS_INFO("Initializing OCCAM");

  pair<OccamDevice *, OccamDeviceList *> occamAPI = initializeOccamAPI();
  OccamDevice *device = occamAPI.first;
  globalDevice = device;
  OccamDeviceList *deviceList = occamAPI.second;  

  ROS_INFO("Setting up OCCAM");

  // Enable auto exposure and gain **important**
  occamSetDeviceValuei(globalDevice, OCCAM_AUTO_EXPOSURE, 1);
  occamSetDeviceValuei(globalDevice, OCCAM_AUTO_GAIN, 1);

  // initialize global constants
  initSensorExtrisics(globalDevice);
  initTransforms();

  boost::thread captureThread = boost::thread(capture);
  ROS_INFO("Spinning up ROS");
  ros::spin();

  // ProfilerStop();
  disposeOccamAPI(occamAPI);

  return 0;
}


