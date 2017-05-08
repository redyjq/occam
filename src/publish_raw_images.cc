/*
Copyright 2011 - 2015 Occam Robotics Inc - All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of Occam Vision Group, Occam Robotics Inc, nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL OCCAM ROBOTICS INC BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "indigo.h"
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

static void reportError(int error_code) {
  fprintf(stderr,"Occam API Error: %i\n",error_code);
  abort();
}

int main(int argc, char** argv) {
  int r;
  int i;
  int j;
  int dev_index = argc>=2?atoi(argv[1]):0;
  OccamDeviceList* device_list;
  OccamDevice* device;

  if ((r = occamInitialize()) != OCCAM_API_SUCCESS)
    reportError(r);

  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  printf("%i devices found\n", device_list->entry_count);
  for (i=0;i<device_list->entry_count;++i) {
    printf("device[%i]: cid = %s\n",
	   i,device_list->entries[i].cid);
  }
  if (dev_index<0 || dev_index >= device_list->entry_count) {
    fprintf(stderr,"device index %i out of range\n",dev_index);
    return 1;
  }

  if ((r = occamOpenDevice(device_list->entries[dev_index].cid, &device)) != OCCAM_API_SUCCESS)
    reportError(r);

  int sensor_count = 0;
  occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensor_count);
  OccamDataName* req = (OccamDataName*)occamAlloc(sensor_count*sizeof(OccamDataName));
  OccamImage** images = (OccamImage**)occamAlloc(sensor_count*sizeof(OccamImage*));
  for (i=0;i<sensor_count;++i)
    req[i] = static_cast<OccamDataName>((int)OCCAM_IMAGE0+i);

  if (sensor_count != 10){
    fprintf(stderr, "This node expects an occam with 10 sensors!");
    return 0;
  }

  ros::init(argc, argv, "occam_raw");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  std::string base_topic = "/occam/rgb/rgb";

  sensor_msgs::ImagePtr msgs [sensor_count];
  //image_transport::Publisher publishers [sensor_count];
  ros::Publisher publishers [sensor_count];
  for (j=0;j<sensor_count;j++)
  {
    std::ostringstream ss;
    ss << j;
    std::string topic = base_topic + ss.str() + "_local";
    //publishers[j] = it.advertise(topic, 1);
    publishers[j] = n.advertise <sensor_msgs::Image>(topic, 1);
  }

  printf("Publishing images...");

  ros::Rate rate(10);
  while (ros::ok())
  {
    if ((r = occamDeviceReadData(device, sensor_count, req, 0, (void**)images, 1)) != OCCAM_API_SUCCESS)
      reportError(r);

    ros::Time stamp = ros::Time::now();

    for (j=0;j<sensor_count;++j)
    {
       /*printf("read image %i.%i: time_ns = %llu, index = %i, format = %i, width = %i, height = %i\n",
	     i, j, (long long unsigned int)images[j]->time_ns, images[j]->index,
	     images[j]->format, images[j]->width, images[j]->height);*/

       /*Convert to opencv/ROS and publish via imagetransport*/
       cv::Mat img;
       if (images[j] && images[j]->format == OCCAM_GRAY8)
       {
          img = cv::Mat_<uchar>(images[j]->height,images[j]->width,(uchar*)images[j]->data[0],images[j]->step[0]);
       }
       else if (images[j] && images[j]->format == OCCAM_RGB24) {
          img = cv::Mat_<cv::Vec3b>(images[j]->height,images[j]->width,(cv::Vec3b*)images[j]->data[0],images[j]->step[0]);
          cv::Mat img1;
          cv::cvtColor(img, img1, cv::COLOR_BGR2RGB);
          img.release ();
          img = img1;
       } else if (images[j] && images[j]->format == OCCAM_SHORT1) {
          img = cv::Mat_<short>(images[j]->height,images[j]->width,(short*)images[j]->data[0],images[j]->step[0]);
       } else {
          printf("image format not supported \n");
       }
       occamFreeImage(images[j]);

       std_msgs::Header header = std_msgs::Header();
       header.stamp = stamp;
       header.frame_id = "/occam_optical_link";
       cv_bridge::CvImage cv_image = cv_bridge::CvImage(header, "bgr8", img);
       img.release ();
       sensor_msgs::ImagePtr msg = cv_image.toImageMsg();
       publishers[j].publish (msg);
    }

    ros::spinOnce();
    rate.sleep();
  }

  occamFree(images);
  occamFree(req);
  occamCloseDevice(device);
  occamFreeDeviceList(device_list);
  occamShutdown();

  return 0;
}
