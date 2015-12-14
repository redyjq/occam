#include <indigo.h>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

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

		printf("R: %d G: %d B: %d X: %f Y: %f Z: %f\n", point->r, point->g, point->b, point->x, point->y, point->z);

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
	capturePointCloud(device, pclPointCloud);

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