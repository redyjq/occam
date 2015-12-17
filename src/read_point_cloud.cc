#include <indigo.h>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <assert.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <sstream>


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

		pclPointCloud->push_back(*point);
		numPointsConverted++;
	}

	pclPointCloud->is_dense = false;

	return numPointsConverted;
}

void savePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud) {
    std::string name = "data/pointcloud.pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGBA> (name, *point_cloud); 
}

void saveImage(OccamImage* image, std::string fileName) {
	cv::Mat cvImage;
	if (image && image->format == OCCAM_GRAY8) {
		cvImage = cv::Mat_<uchar>(image->height, image->width, (uchar*)image->data[0], image->step[0]);
	}
	else if (image && image->format == OCCAM_RGB24) {
		cvImage = cv::Mat_<cv::Vec3b>(image->height, image->width, (cv::Vec3b*)image->data[0], image->step[0]);
		cv::Mat colorImage;
		cv::cvtColor(cvImage, colorImage, cv::COLOR_BGR2RGB);
		cvImage = colorImage;
	}
	else if (image && image->format == OCCAM_SHORT1) {
		cvImage = cv::Mat_<short>(image->height, image->width, (short*)image->data[0], image->step[0]);
	}
	else {
		printf("Image type not supported: %d\n", image->format);
	}

	imwrite(fileName, cvImage);
}

void capturePointCloud(OccamDevice* device, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud) {
	// Capture a point cloud
	OccamDataName requestTypes[] = { OCCAM_POINT_CLOUD4 };
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
	OccamImage* rgbImage = captureImage(device, OCCAM_IMAGE2);
	printf("RGB Image captured at time: %llu\n", (long long unsigned int)rgbImage->time_ns);
	OccamImage* disparityImage = captureImage(device, OCCAM_DISPARITY_IMAGE1);
	printf("Disparity Image captured at time: %llu\n", (long long unsigned int)disparityImage->time_ns);

	// Save the images
	saveImage(rgbImage, "rgbImage.jpg");
	saveImage(disparityImage, "disparityImage.jpg");

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

  	// Get point cloud for the image
  	int indices[] = {0};
  	OccamImage* rgbImages[1];
  	rgbImages[0] = rgbImage;
  	OccamImage* disparityImages[1];
  	disparityImages[0] = disparityImage;
  	OccamPointCloud* pointCloud;
  	handleError(rectifyIface->generateCloud(rectifyHandle, 1, indices, 1, rgbImages, disparityImages, &pointCloud));

	// Print statistics
	printf("Number of points in Occam point cloud: %d\n", pointCloud->point_count);

	// Convert to PCL point cloud
	int numConverted = convertToPcl(pointCloud, pclPointCloud);
	printf("Number of points converted to PCL: %d\n", numConverted);

  	// Clean up
  	handleError(occamFreePointCloud(pointCloud));
}

int main(int argc, char** argv) {

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

	// Capture point cloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pclPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// capturePointCloud(device, pclPointCloud);
	constructPointCloud(device, pclPointCloud);

	// Save point cloud
	savePointCloud(pclPointCloud);

	// Display created pointcloud
	visualizePointCloud(pclPointCloud);

	// Clean up
	
	handleError(occamCloseDevice(device));
	handleError(occamFreeDeviceList(deviceList));
	handleError(occamShutdown());

	return 0;
}