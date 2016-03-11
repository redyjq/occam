#include <indigo.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

// OCCAM CONSTANTS
#define BASELINE 0.12f //in meters 
#define FOCAL 2.8f //in mm
#define FOCAL_PIXELS 10.583 //in pixels

void handleError(int returnCode) {
    if (returnCode != OCCAM_API_SUCCESS) {
        char errorMsg[30];
        occamGetErrorString((OccamError)returnCode, errorMsg, 30);
        fprintf(stderr, "Occam API Error: %d. %s\n", returnCode, errorMsg);
        abort();
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
    
    // Capture RGB and disparity image
	OccamImage* rgbImage = captureImage(device, OCCAM_IMAGE2);
	OccamImage* disparityImage = captureImage(device, OCCAM_DISPARITY_IMAGE1);

	// Convert to point cloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	int width = rgbImage->width;
	int height = rgbImage->height; 
       
	for (int h = 0; h < height; h++) {
	    for (int w = 0; w < width; w++) {
	        pcl::PointXYZRGBA* point = new pcl::PointXYZRGBA();                     
	        
	        // calculate depth
	        int index = (height * 480) + w;
	        float disparity = disparityImage->data[0][index];
	        float depth = BASELINE * FOCAL_PIXELS / disparity; 

	        // set rgba            
	        point->r = rgbImage->data[0][index*3];
	        point->g = rgbImage->data[0][index*3 + 1];
	        point->b = rgbImage->data[0][index*3 + 2];      
	        point->a = 255;       
	     
	        // set xyz
	        point->x = w * depth / FOCAL_PIXELS;
	        point->y = h * depth / FOCAL_PIXELS;
	        point->z = depth;     

                int CULL_THRESHOLD = 20000;
                if (point->x + point->y + point->z < CULL_THRESHOLD) {
                  // add point to point cloud   
                  cloud->push_back(*point);
                }

	        //std::cout << "disparity: " << disparity << "   depth: " << depth << std::endl;
	        
	    }
	}

	cloud->width = width;
	cloud->height = height; 

	std::cout << "width: " << cloud->width << "    height: " << cloud->height << std::endl;

	// Display created pointcloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));   
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);                     
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "cloud");
    while (!viewer->wasStopped()) {
        viewer->spinOnce();    
    }

    return 0;

}
