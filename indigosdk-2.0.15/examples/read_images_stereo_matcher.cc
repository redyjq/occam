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
#include <time.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

enum BMPrefilterTypes {
  OCCAM_PREFILTER_NONE = 0,
  OCCAM_PREFILTER_XSOBEL = 1,
  OCCAM_PREFILTER_NORMALIZED_RESPONSE = 2
};

static void reportError(int error_code) {
  char str[1024] = {0};
  occamGetErrorString((OccamError)error_code, str, sizeof(str));
  std::cerr<<"Occam API Error: "<<str<<" ("<<error_code<<")"<<std::endl;
  abort();
}

static std::string dataName(OccamDataName data_name) {
  switch (data_name) {
  case OCCAM_IMAGE0: return "image0";
  case OCCAM_IMAGE1: return "image1";
  case OCCAM_IMAGE2: return "image2";
  case OCCAM_IMAGE3: return "image3";
  case OCCAM_IMAGE4: return "image4";
  case OCCAM_IMAGE5: return "image5";
  case OCCAM_IMAGE6: return "image6";
  case OCCAM_IMAGE7: return "image7";
  case OCCAM_IMAGE8: return "image8";
  case OCCAM_IMAGE9: return "image9";
  case OCCAM_IMAGE10: return "image10";
  case OCCAM_IMAGE11: return "image11";
  case OCCAM_IMAGE12: return "image12";
  case OCCAM_IMAGE13: return "image13";
  case OCCAM_IMAGE14: return "image14";
  case OCCAM_RAW_IMAGE0: return "raw_image0";
  case OCCAM_RAW_IMAGE1: return "raw_image1";
  case OCCAM_RAW_IMAGE2: return "raw_image2";
  case OCCAM_RAW_IMAGE3: return "raw_image3";
  case OCCAM_RAW_IMAGE4: return "raw_image4";
  case OCCAM_RAW_IMAGE5: return "raw_image5";
  case OCCAM_RAW_IMAGE6: return "raw_image6";
  case OCCAM_RAW_IMAGE7: return "raw_image7";
  case OCCAM_RAW_IMAGE8: return "raw_image8";
  case OCCAM_RAW_IMAGE9: return "raw_image9";
  case OCCAM_RAW_IMAGE10: return "raw_image10";
  case OCCAM_RAW_IMAGE11: return "raw_image11";
  case OCCAM_RAW_IMAGE12: return "raw_image12";
  case OCCAM_RAW_IMAGE13: return "raw_image13";
  case OCCAM_RAW_IMAGE14: return "raw_image14";
  case OCCAM_IMAGE_TILES0: return "image_tiles0";
  case OCCAM_IMAGE_TILES1: return "image_tiles1";
  case OCCAM_IMAGE_TILES2: return "image_tiles2";
  case OCCAM_RAW_IMAGE_TILES0: return "raw_image_tiles0";
  case OCCAM_RAW_IMAGE_TILES1: return "raw_image_tiles1";
  case OCCAM_RAW_IMAGE_TILES2: return "raw_image_tiles2";
  case OCCAM_UNDISTORTED_IMAGE_TILES0: return "undistorted_image_tiles0";
  case OCCAM_UNDISTORTED_IMAGE_TILES1: return "undistorted_image_tiles1";
  case OCCAM_UNDISTORTED_IMAGE_TILES2: return "undistorted_image_tiles2";
  case OCCAM_UNDISTORTED_IMAGE0: return "undistorted_image0";
  case OCCAM_UNDISTORTED_IMAGE1: return "undistorted_image1";
  case OCCAM_UNDISTORTED_IMAGE2: return "undistorted_image2";
  case OCCAM_UNDISTORTED_IMAGE3: return "undistorted_image3";
  case OCCAM_UNDISTORTED_IMAGE4: return "undistorted_image4";
  case OCCAM_UNDISTORTED_IMAGE5: return "undistorted_image5";
  case OCCAM_UNDISTORTED_IMAGE6: return "undistorted_image6";
  case OCCAM_UNDISTORTED_IMAGE7: return "undistorted_image7";
  case OCCAM_UNDISTORTED_IMAGE8: return "undistorted_image8";
  case OCCAM_UNDISTORTED_IMAGE9: return "undistorted_image9";
  case OCCAM_UNDISTORTED_IMAGE10: return "undistorted_image10";
  case OCCAM_UNDISTORTED_IMAGE11: return "undistorted_image11";
  case OCCAM_UNDISTORTED_IMAGE12: return "undistorted_image12";
  case OCCAM_UNDISTORTED_IMAGE13: return "undistorted_image13";
  case OCCAM_UNDISTORTED_IMAGE14: return "undistorted_image14";
  case OCCAM_STITCHED_IMAGE0: return "stitched_image0";
  case OCCAM_STITCHED_IMAGE1: return "stitched_image1";
  case OCCAM_STITCHED_IMAGE2: return "stitched_image2";
  case OCCAM_RECTIFIED_IMAGE0: return "rectified_image0";
  case OCCAM_RECTIFIED_IMAGE1: return "rectified_image1";
  case OCCAM_RECTIFIED_IMAGE2: return "rectified_image2";
  case OCCAM_RECTIFIED_IMAGE3: return "rectified_image3";
  case OCCAM_RECTIFIED_IMAGE4: return "rectified_image4";
  case OCCAM_RECTIFIED_IMAGE5: return "rectified_image5";
  case OCCAM_RECTIFIED_IMAGE6: return "rectified_image6";
  case OCCAM_RECTIFIED_IMAGE7: return "rectified_image7";
  case OCCAM_RECTIFIED_IMAGE8: return "rectified_image8";
  case OCCAM_RECTIFIED_IMAGE9: return "rectified_image9";
  case OCCAM_DISPARITY_IMAGE0: return "disparity_image0";
  case OCCAM_DISPARITY_IMAGE1: return "disparity_image1";
  case OCCAM_DISPARITY_IMAGE2: return "disparity_image2";
  case OCCAM_DISPARITY_IMAGE3: return "disparity_image3";
  case OCCAM_DISPARITY_IMAGE4: return "disparity_image4";
  case OCCAM_TILED_DISPARITY_IMAGE: return "tiled_disparity_image";
  case OCCAM_STITCHED_DISPARITY_IMAGE: return "stitched_disparity_image";
  case OCCAM_POINT_CLOUD0: return "point_cloud0";
  case OCCAM_POINT_CLOUD1: return "point_cloud1";
  case OCCAM_POINT_CLOUD2: return "point_cloud2";
  case OCCAM_POINT_CLOUD3: return "point_cloud3";
  case OCCAM_POINT_CLOUD4: return "point_cloud4";
  case OCCAM_STITCHED_POINT_CLOUD: return "stitched_point_cloud";
  }
  return std::string();
}

bool getParamRange(OccamDevice* device, OccamParam param_type, int& min_value, int& max_value) {
  int r;
  OccamParamList* param_list;
  if ((r = occamEnumerateParamList(device, &param_list)) != OCCAM_API_SUCCESS)
    return false;
  for (int j=0;j<param_list->param_count;++j) {
    if (param_list->params[j].id == param_type) {
      min_value = int(param_list->params[j].min_value);
      max_value = int(param_list->params[j].max_value);
      occamFreeParamList(param_list);
      return true;
    }
  }
  occamFreeParamList(param_list);
  return false;
}

int main(int argc, const char** argv) {
  int r;
  int i;
  int dev_index = argc>=2?atoi(argv[1]):0;
  OccamDeviceList* device_list;
  OccamDevice* device;
  OccamImage* image;

  if ((r = occamInitialize()) != OCCAM_API_SUCCESS)
    reportError(r);

  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  std::cerr<<device_list->entry_count<<" devices found"<<std::endl;
  for (i=0;i<device_list->entry_count;++i) {
    std::cerr<<"device["<<i<<"]: cid = "<<device_list->entries[i].cid<<std::endl;
  }
  if (dev_index<0 || dev_index >= device_list->entry_count) {
    std::cerr<<"device index "<<dev_index<<" out of range"<<std::endl;
    return 1;
  }

  if ((r = occamOpenDevice(device_list->entries[dev_index].cid, &device)) != OCCAM_API_SUCCESS)
    reportError(r);

  occamSetDeviceValuei(device,OCCAM_MAX_DEFERRED_REAPING_FRAMES,1);
  occamSetDeviceValuei(device,OCCAM_MAX_DEFERRED_PENDING_FRAMES,1);

  int min_exposure;
  int max_exposure;
  int min_gain;
  int max_gain;
  if (!getParamRange(device, OCCAM_EXPOSURE, min_exposure, max_exposure)) {
    std::cerr<<"failed to discover exposure range"<<std::endl;
    return 1;
  }
  if (!getParamRange(device, OCCAM_GAIN, min_gain, max_gain)) {
    std::cerr<<"failed to discover exposure range"<<std::endl;
    return 1;
  }
  printf("exposure range: [%i, %i]\n", min_exposure, max_exposure);
  printf("gain range: [%i, %i]\n", min_gain, max_gain);
  int exposure = (max_exposure + min_exposure) / 2;
  int gain = (max_gain + min_gain) / 2;
  bool downsample = true;

  printf("COMMANDS:\n");
  printf("  ESC/Q/q: quit this application\n");
  printf("  1: next image output\n");
  printf("  2: previous image output\n");
  printf("  3: cpu backend\n");
  printf("  4: opengl backend\n");
  printf("  r: disable auto-exposure\n");
  printf("  t: enable auto-exposure\n");
  printf("  f: disable auto-gain\n");
  printf("  g: enable auto-gain\n");
  printf("  [: decrease exposure\n");
  printf("  ]: increase exposure\n");
  printf("  ;: decrease gain\n");
  printf("  ': increase gain\n");

  printf("  q: set OCCAM_BM_PREFILTER_TYPE to OCCAM_PREFILTER_XSOBEL\n");
  printf("  a: set OCCAM_BM_PREFILTER_TYPE to OCCAM_PREFILTER_NONE\n");
  printf("  z: set OCCAM_BM_PREFILTER_TYPE to OCCAM_PREFILTER_NORMALIZED_RESPONSE\n");
  printf("  w: OCCAM_BM_PREFILTER_SIZE increase\n");
  printf("  e: OCCAM_BM_PREFILTER_SIZE decrease\n");
  printf("  s: OCCAM_BM_PREFILTER_CAP increase\n");
  printf("  d: OCCAM_BM_PREFILTER_CAP decrease\n");
  printf("  x: OCCAM_BM_SAD_WINDOW_SIZE increase\n");
  printf("  c: OCCAM_BM_SAD_WINDOW_SIZE decrease\n");
  printf("  v: OCCAM_BM_MIN_DISPARITY increase\n");
  printf("  b: OCCAM_BM_MIN_DISPARITY decrease\n");
  printf("  y: OCCAM_BM_NUM_DISPARITIES increase\n");
  printf("  u: OCCAM_BM_NUM_DISPARITIES decrease\n");
  printf("  h: OCCAM_BM_TEXTURE_THRESHOLD increase\n");
  printf("  j: OCCAM_BM_TEXTURE_THRESHOLD decrease\n");
  printf("  n: OCCAM_BM_UNIQUENESS_RATIO increase\n");
  printf("  m: OCCAM_BM_UNIQUENESS_RATIO decrease\n");
  printf("  i: OCCAM_BM_SPECKLE_RANGE increase\n");
  printf("  o: OCCAM_BM_SPECKLE_RANGE decrease\n");
  printf("  k: OCCAM_BM_SPECKLE_WINDOW_SIZE increase\n");
  printf("  l: OCCAM_BM_SPECKLE_WINDOW_SIZE decrease\n");

  int req_count = 0;
  OccamDataName* req;
  OccamDataType* types;
  occamDeviceAvailableData(device, &req_count, &req, &types);
  std::vector<OccamDataName> image_req;
  for (int j=0;j<req_count;++j) {
    if (types[j] == OCCAM_IMAGE)
      image_req.push_back(req[j]);
  }
  occamFree(req);
  occamFree(types);
  int current_req_index = 0;
  std::cerr<<"Available image outputs: ";
  for (int j=0;j<image_req.size();++j)
    std::cerr<<(j?", ":"")<<dataName(image_req[j])<<" ("<<image_req[j]<<")";
  std::cerr<<std::endl;

  int fps = 0;
  time_t last_reset = 0;

  // init to start img
  auto start = OCCAM_STITCHED_IMAGE1;
  // auto start = OCCAM_STITCHED_DISPARITY_IMAGE;
  int pos = std::find(image_req.begin(), image_req.end(), start) - image_req.begin();
  if( pos < image_req.size() ) {
    current_req_index = pos;
    std::cerr<<"Changing output to "<<dataName(image_req[current_req_index])<<std::endl;
  }
  else
    std::cerr << "start not found." << std::endl;

  // init with auto exposure/gain
  occamSetDeviceValuei(device, OCCAM_AUTO_EXPOSURE, 1);
  occamSetDeviceValuei(device, OCCAM_AUTO_GAIN, 1);

  // init default values
  int _OCCAM_BM_PREFILTER_SIZE = 9;
  int _OCCAM_BM_PREFILTER_CAP = 31;
  int _OCCAM_BM_SAD_WINDOW_SIZE = 15;
  int _OCCAM_BM_MIN_DISPARITY = 0;
  int _OCCAM_BM_NUM_DISPARITIES = 64;
  int _OCCAM_BM_TEXTURE_THRESHOLD = 10;
  int _OCCAM_BM_UNIQUENESS_RATIO = 60;
  int _OCCAM_BM_SPECKLE_RANGE = 120;
  int _OCCAM_BM_SPECKLE_WINDOW_SIZE = 400;

  for (i=0;;++i) {
    image = 0;
    if ((r = occamDeviceReadData(device,1,&image_req[current_req_index],0,
         (void**)&image, 1)) != OCCAM_API_SUCCESS) {
      if (r != OCCAM_API_UNSUPPORTED_DATA)
  reportError(r);
    }
    ++fps;

    time_t now = time(0);
    if (now != last_reset) {
      last_reset = now;
      printf("fps: %i\n",fps);
      fps = 0;
    }

    cv::Mat img;
    if (image && image->format == OCCAM_GRAY8)
      img = cv::Mat_<uchar>(image->height,image->width,(uchar*)image->data[0],image->step[0]);
    else if (image && image->format == OCCAM_RGB24) {
      img = cv::Mat_<cv::Vec3b>(image->height,image->width,(cv::Vec3b*)image->data[0],image->step[0]);
      cv::Mat img1;
      cv::cvtColor(img, img1, cv::COLOR_BGR2RGB);
      img = img1;
    } else if (image && image->format == OCCAM_SHORT1) {
      img = cv::Mat_<short>(image->height,image->width,(short*)image->data[0],image->step[0]);
    } else {
      //      printf("image format not supported by this demo\n");
    }

    
    int small_width = 1900;
    if (image && downsample && img.cols>small_width) {
      int small_height = image->height * small_width / image->width;
      cv::Mat small_img;
      cv::resize(img, small_img, cv::Size(small_width,small_height));
      img = small_img;
    }

    if (!img.empty())
      imshow("indigosdk", img);

    occamFreeImage(image);

    char key = (char)cv::waitKey(1);
    if (key == 27) // || key == 'Q' || key == 'q') // ESC/Q/q keys to quit
      break;

    else if (key == '1') { // next image output
      if (++current_req_index>=image_req.size())
        current_req_index = 0;
      if (current_req_index<image_req.size())
        std::cerr<<"Changing output to "<<dataName(image_req[current_req_index])<<std::endl;
    }
    else if (key == '2') { // previous image output
      if (--current_req_index<0)
        current_req_index = std::max(0,int(image_req.size()-1));
      if (current_req_index<image_req.size())
        std::cerr<<"Changing output to "<<dataName(image_req[current_req_index])<<std::endl;
    }
    else if (key == '3') { // cpu backend
      occamSetDeviceValuei(device, OCCAM_PREFERRED_BACKEND, OCCAM_CPU);
    }
    else if (key == '4') { // opengl backend
      occamSetDeviceValuei(device, OCCAM_PREFERRED_BACKEND, OCCAM_OPENGL);
    }
    else if (key == '5') {
      int pos = std::find(image_req.begin(), image_req.end(), OCCAM_STITCHED_IMAGE1) - image_req.begin();
      if( pos < image_req.size() ) {
        current_req_index = pos;
        std::cerr<<"Changing output to "<<dataName(image_req[current_req_index])<<std::endl;
      }
      else
        std::cerr << OCCAM_STITCHED_IMAGE1 << " not found." << std::endl;
    }
    

    else if (key == 'r') { // disable auto-exposure
      occamSetDeviceValuei(device, OCCAM_AUTO_EXPOSURE, 0);
      std::cerr<<"auto-exposure disabled"<<std::endl;
    }
    else if (key == 't') { // enable auto-exposure
      occamSetDeviceValuei(device, OCCAM_AUTO_EXPOSURE, 1);
      std::cerr<<"auto-exposure enabled"<<std::endl;
    }

    else if (key == 'f') { // disable auto-gain
      occamSetDeviceValuei(device, OCCAM_AUTO_GAIN, 0);
      std::cerr<<"auto-gain disabled"<<std::endl;
    }
    else if (key == 'g') { // enable auto-gain
      occamSetDeviceValuei(device, OCCAM_AUTO_GAIN, 1);
      std::cerr<<"auto-gain enabled"<<std::endl;
    }

    else if (key == ']') { // increase exposure
      exposure = std::min(max_exposure,exposure+1);
      std::cerr<<"manual exposure: "<<exposure<<std::endl;
      occamSetDeviceValuei(device, OCCAM_EXPOSURE, exposure);
    } 
    else if (key == '[') { // decrease exposure
      exposure = std::max(min_exposure,exposure-1);
      std::cerr<<"manual exposure: "<<exposure<<std::endl;
      occamSetDeviceValuei(device, OCCAM_EXPOSURE, exposure);
    }

    else if (key == '\'') { // increase gain
      gain = std::min(max_gain,gain+1);
      std::cerr<<"manual gain: "<<gain<<std::endl;
      occamSetDeviceValuei(device, OCCAM_GAIN, gain);
    }
    else if (key == ';') { // decrease gain
      gain = std::max(min_gain,gain-1);
      std::cerr<<"manual gain: "<<gain<<std::endl;
      occamSetDeviceValuei(device, OCCAM_GAIN, gain);
    }

    else if (key == 'q') {
      occamSetDeviceValuei(device, OCCAM_BM_PREFILTER_TYPE,OCCAM_PREFILTER_XSOBEL);
      std::cerr<<"OCCAM_BM_PREFILTER_TYPE OCCAM_PREFILTER_XSOBEL"<<std::endl;
    }
    else if (key == 'a') {
      occamSetDeviceValuei(device, OCCAM_BM_PREFILTER_TYPE,OCCAM_PREFILTER_NONE); // (default
      std::cerr<<"OCCAM_BM_PREFILTER_TYPE OCCAM_PREFILTER_NONE"<<std::endl;
    }
    else if (key == 'z') {
      occamSetDeviceValuei(device, OCCAM_BM_PREFILTER_TYPE,OCCAM_PREFILTER_NORMALIZED_RESPONSE);
      std::cerr<<"OCCAM_BM_PREFILTER_TYPE OCCAM_PREFILTER_NORMALIZED_RESPONSE"<<std::endl;
    }
    
    else if (key == 'w') {
      _OCCAM_BM_PREFILTER_SIZE++;
      occamSetDeviceValuei(device, OCCAM_BM_PREFILTER_SIZE,_OCCAM_BM_PREFILTER_SIZE);
      std::cerr<<"OCCAM_BM_PREFILTER_SIZE (default: 9) increased to "<<_OCCAM_BM_PREFILTER_SIZE<<std::endl;
    }
    else if (key == 'e') {
      _OCCAM_BM_PREFILTER_SIZE--;
      occamSetDeviceValuei(device, OCCAM_BM_PREFILTER_SIZE,_OCCAM_BM_PREFILTER_SIZE);
      std::cerr<<"OCCAM_BM_PREFILTER_SIZE (default: 9) decreased to "<<_OCCAM_BM_PREFILTER_SIZE<<std::endl;
    }
    
    else if (key == 's') {
      _OCCAM_BM_PREFILTER_CAP++;
      occamSetDeviceValuei(device, OCCAM_BM_PREFILTER_CAP,_OCCAM_BM_PREFILTER_CAP);
      std::cerr<<"OCCAM_BM_PREFILTER_CAP (default: 31) increased to "<<_OCCAM_BM_PREFILTER_CAP<<std::endl;
    }
    else if (key == 'd') {
      _OCCAM_BM_PREFILTER_CAP--;
      occamSetDeviceValuei(device, OCCAM_BM_PREFILTER_CAP,_OCCAM_BM_PREFILTER_CAP);
      std::cerr<<"OCCAM_BM_PREFILTER_CAP (default: 31) decreased to "<<_OCCAM_BM_PREFILTER_CAP<<std::endl;
    }
    
    else if (key == 'x') {
      _OCCAM_BM_SAD_WINDOW_SIZE++;
      occamSetDeviceValuei(device, OCCAM_BM_SAD_WINDOW_SIZE,_OCCAM_BM_SAD_WINDOW_SIZE);
      std::cerr<<"OCCAM_BM_SAD_WINDOW_SIZE (default: 15) increased to "<<_OCCAM_BM_SAD_WINDOW_SIZE<<std::endl;
    }
    else if (key == 'c') {
      _OCCAM_BM_SAD_WINDOW_SIZE--;
      occamSetDeviceValuei(device, OCCAM_BM_SAD_WINDOW_SIZE,_OCCAM_BM_SAD_WINDOW_SIZE);
      std::cerr<<"OCCAM_BM_SAD_WINDOW_SIZE (default: 15) decreased to "<<_OCCAM_BM_SAD_WINDOW_SIZE<<std::endl;
    }
    
    else if (key == 'v') {
      _OCCAM_BM_MIN_DISPARITY++;
      occamSetDeviceValuei(device, OCCAM_BM_MIN_DISPARITY,_OCCAM_BM_MIN_DISPARITY);
      std::cerr<<"OCCAM_BM_MIN_DISPARITY (default: 0) increased to "<<_OCCAM_BM_MIN_DISPARITY<<std::endl;
    }
    else if (key == 'b') {
      _OCCAM_BM_MIN_DISPARITY--;
      occamSetDeviceValuei(device, OCCAM_BM_MIN_DISPARITY,_OCCAM_BM_MIN_DISPARITY);
      std::cerr<<"OCCAM_BM_MIN_DISPARITY (default: 0) decreased to "<<_OCCAM_BM_MIN_DISPARITY<<std::endl;
    }
    
    else if (key == 'y') {
      _OCCAM_BM_NUM_DISPARITIES++;
      occamSetDeviceValuei(device, OCCAM_BM_NUM_DISPARITIES,_OCCAM_BM_NUM_DISPARITIES);
      std::cerr<<"OCCAM_BM_NUM_DISPARITIES (default: 64) increased to "<<_OCCAM_BM_NUM_DISPARITIES<<std::endl;
    }
    else if (key == 'u') {
      _OCCAM_BM_NUM_DISPARITIES--;
      occamSetDeviceValuei(device, OCCAM_BM_NUM_DISPARITIES,_OCCAM_BM_NUM_DISPARITIES);
      std::cerr<<"OCCAM_BM_NUM_DISPARITIES (default: 64) decreased to "<<_OCCAM_BM_NUM_DISPARITIES<<std::endl;
    }
    
    else if (key == 'h') {
      _OCCAM_BM_TEXTURE_THRESHOLD++;
      occamSetDeviceValuei(device, OCCAM_BM_TEXTURE_THRESHOLD,_OCCAM_BM_TEXTURE_THRESHOLD);
      std::cerr<<"OCCAM_BM_TEXTURE_THRESHOLD (default: 10) increased to "<<_OCCAM_BM_TEXTURE_THRESHOLD<<std::endl;
    }
    else if (key == 'j') {
      _OCCAM_BM_TEXTURE_THRESHOLD--;
      occamSetDeviceValuei(device, OCCAM_BM_TEXTURE_THRESHOLD,_OCCAM_BM_TEXTURE_THRESHOLD);
      std::cerr<<"OCCAM_BM_TEXTURE_THRESHOLD (default: 10) decreased to "<<_OCCAM_BM_TEXTURE_THRESHOLD<<std::endl;
    }
    
    else if (key == 'n') {
      _OCCAM_BM_UNIQUENESS_RATIO++;
      occamSetDeviceValuei(device, OCCAM_BM_UNIQUENESS_RATIO,_OCCAM_BM_UNIQUENESS_RATIO);
      std::cerr<<"OCCAM_BM_UNIQUENESS_RATIO (default: 60) increased to "<<_OCCAM_BM_UNIQUENESS_RATIO<<std::endl;
    }
    else if (key == 'm') {
      _OCCAM_BM_UNIQUENESS_RATIO--;
      occamSetDeviceValuei(device, OCCAM_BM_UNIQUENESS_RATIO,_OCCAM_BM_UNIQUENESS_RATIO);
      std::cerr<<"OCCAM_BM_UNIQUENESS_RATIO (default: 60) decreased to "<<_OCCAM_BM_UNIQUENESS_RATIO<<std::endl;
    }
    
    else if (key == 'i') {
      _OCCAM_BM_SPECKLE_RANGE+=10;
      occamSetDeviceValuei(device, OCCAM_BM_SPECKLE_RANGE,_OCCAM_BM_SPECKLE_RANGE);
      std::cerr<<"OCCAM_BM_SPECKLE_RANGE (default: 120) increased to "<<_OCCAM_BM_SPECKLE_RANGE<<std::endl;
    }
    else if (key == 'o') {
      _OCCAM_BM_SPECKLE_RANGE-=10;
      occamSetDeviceValuei(device, OCCAM_BM_SPECKLE_RANGE,_OCCAM_BM_SPECKLE_RANGE);
      std::cerr<<"OCCAM_BM_SPECKLE_RANGE (default: 120) decreased to "<<_OCCAM_BM_SPECKLE_RANGE<<std::endl;
    }
    
    else if (key == 'k') {
      _OCCAM_BM_SPECKLE_WINDOW_SIZE+=10;
      occamSetDeviceValuei(device, OCCAM_BM_SPECKLE_WINDOW_SIZE,_OCCAM_BM_SPECKLE_WINDOW_SIZE);
      std::cerr<<"OCCAM_BM_SPECKLE_WINDOW_SIZE (default: 400) increased to "<<_OCCAM_BM_SPECKLE_WINDOW_SIZE<<std::endl;
    }
    else if (key == 'l') {
      _OCCAM_BM_SPECKLE_WINDOW_SIZE-=10;
      occamSetDeviceValuei(device, OCCAM_BM_SPECKLE_WINDOW_SIZE,_OCCAM_BM_SPECKLE_WINDOW_SIZE);
      std::cerr<<"OCCAM_BM_SPECKLE_WINDOW_SIZE (default: 400) decreased to "<<_OCCAM_BM_SPECKLE_WINDOW_SIZE<<std::endl;
    }
    
  }

  occamCloseDevice(device);
  occamFreeDeviceList(device_list);
  occamShutdown();

  return 0;
}
