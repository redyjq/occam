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

#include "gl_utils.h"
#include "device_iface.h"
#include "omni_libusb.h"
#include "serialize_utils.h"
#include "image_collect.h"
#include <algorithm>
#include <iostream>
#include <assert.h>
#include <string.h>
#include <indigo.h>
#include <math.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

using namespace cv;
using namespace ximgproc;
using namespace std;

#undef max

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);

static DeferredImage subImage(const DeferredImage& img0,
        int x,
        int y,
        int width,
        int height) {
    auto gen_fn = [=](){
        OccamImage* img1 = 0;
        occamSubImage(img0->get(), &img1, x, y, width, height);
        return std::shared_ptr<OccamImage>(img1,occamFreeImage);
    };
    return DeferredImage(gen_fn,img0);
};

static DeferredImage processImage(std::shared_ptr<void> imagef_handle,
        std::shared_ptr<void> debayerf_handle,
        bool is_color,
        DeferredImage img0) {
    if (is_color) {
        auto gen_fn = [=](){
            IOccamImageFilter* imagef_iface;
            occamGetInterface(debayerf_handle.get(),IOCCAMIMAGEFILTER,(void**)&imagef_iface);
            OccamImage* img1 = 0;
            imagef_iface->compute(debayerf_handle.get(),img0->get(),&img1);
            return std::shared_ptr<OccamImage>(img1,occamFreeImage);
        };
        img0 = DeferredImage(gen_fn,img0);
    }

    auto gen_fn = [=](){
        IOccamImageFilter* imagef_iface;
        occamGetInterface(imagef_handle.get(),IOCCAMIMAGEFILTER,(void**)&imagef_iface);
        OccamImage* img1 = 0;
        imagef_iface->compute(imagef_handle.get(),img0->get(),&img1);
        return std::shared_ptr<OccamImage>(img1,occamFreeImage);
    };
    return DeferredImage(gen_fn,img0);
}

static DeferredImage htile(const std::vector<DeferredImage>& img0) {
    auto gen_fn = [=](){
        const OccamImage* img0p = img0[0]->get();

        OccamImage* img1 = new OccamImage;
        memset(img1,0,sizeof(OccamImage));
        img1->cid = strdup(img0p->cid);
        memcpy(img1->timescale,img0p->timescale,sizeof(img1->timescale));
        img1->time_ns = img0p->time_ns;
        img1->index = img0p->index;
        img1->refcnt = 1;
        img1->backend = img0p->backend;
        img1->format = img0p->format;

        int width = 0;
        int height = 0;
        img1->subimage_count = img0.size();
        for (int j=0;j<img0.size();++j) {
            img1->si_x[j] = width;
            img1->si_y[j] = 0;
            img1->si_width[j] = img0[j]->get()->width;
            img1->si_height[j] = img0[j]->get()->height;
            width += img0[j]->get()->width;
            height = std::max(height,img0[j]->get()->height);
        }

        img1->width = width;
        img1->height = height;

        int bpp = 1;
        occamImageFormatBytesPerPixel(img1->format, &bpp);
        img1->step[0] = ((img1->width*bpp)+15)&~15;
        img1->data[0] = new uint8_t[img1->height * img1->step[0]];

        uint8_t* imgp0 = img1->data[0];
        for (int y=0;y<img1->height;++y,imgp0+=img1->step[0]) {
            uint8_t* imgp1 = imgp0;
            for (int j=0,x=0;j<img0.size();++j) {
                const OccamImage* imgjp = img0[j]->get();
                if (y<imgjp->height)
                    memcpy(imgp1,imgjp->data[0]+y*imgjp->step[0],imgjp->width*bpp);
                else
                    memset(imgp1,0,imgjp->width*bpp);
                x += imgjp->width*bpp;
                imgp1 += imgjp->width*bpp;
            }
        }

        return std::shared_ptr<OccamImage>(img1,occamFreeImage);
    };  
    const Deferred** deps = (const Deferred**)alloca(sizeof(Deferred*)*img0.size());
    for (int j=0;j<img0.size();++j)
        deps[j] = &img0[j];
    return DeferredImage(gen_fn,img0.size(),deps);
}

static DeferredImage vtile(const std::vector<DeferredImage>& img0) {
    auto gen_fn = [=](){
        const OccamImage* img0p = img0[0]->get();

        OccamImage* img1 = new OccamImage;
        memset(img1,0,sizeof(OccamImage));
        img1->cid = strdup(img0p->cid);
        memcpy(img1->timescale,img0p->timescale,sizeof(img1->timescale));
        img1->time_ns = img0p->time_ns;
        img1->index = img0p->index;
        img1->refcnt = 1;
        img1->backend = img0p->backend;
        img1->format = img0p->format;

        int width = 0;
        int height = 0;
        img1->subimage_count = 0;
        for (int j=0;j<img0.size();++j) {
            for (int k=0;k<img0[j]->get()->subimage_count;++k) {
                int si_index = img1->subimage_count++;
                img1->si_x[si_index] = img0[j]->get()->si_x[k];
                img1->si_y[si_index] = img0[j]->get()->si_y[k]+height;
                img1->si_width[si_index] = img0[j]->get()->si_width[k];
                img1->si_height[si_index] = img0[j]->get()->si_height[k];
            }
            height += img0[j]->get()->height;
            width = std::max(width,img0[j]->get()->width);
        }

        img1->width = width;
        img1->height = height;

        int bpp = 1;
        occamImageFormatBytesPerPixel(img1->format, &bpp);
        img1->step[0] = ((img1->width*bpp)+15)&~15;
        img1->data[0] = new uint8_t[img1->height * img1->step[0]];

        uint8_t* imgp0 = img1->data[0];
        for (int j=0,x=0;j<img0.size();++j) {
            const OccamImage* imgjp = img0[j]->get();
            uint8_t* imgp1 = imgjp->data[0];
            for (int y=0;y<imgjp->height;++y,imgp0+=img1->step[0],imgp1+=imgjp->step[0]) {
                memcpy(imgp0,imgp1,imgjp->width*bpp);
            }
        }

        return std::shared_ptr<OccamImage>(img1,occamFreeImage);
    };  
    const Deferred** deps = (const Deferred**)alloca(sizeof(Deferred*)*img0.size());
    for (int j=0;j<img0.size();++j)
        deps[j] = &img0[j];
    return DeferredImage(gen_fn,img0.size(),deps);
}

static DeferredImage makeMonoImage(DeferredImage img0) {
    auto gen_fn = [=](){
        OccamImage* img1 = img0->get();

        if (img1->format != OCCAM_RGB24) {
            OccamImage* img2;
            occamCopyImage(img1, &img2, 0);
            return std::shared_ptr<OccamImage>(img2,occamFreeImage);
        }

        OccamImage* img2 = new OccamImage;
        memset(img2,0,sizeof(OccamImage));
        img2->cid = strdup(img1->cid);
        memcpy(img2->timescale,img1->timescale,sizeof(img1->timescale));
        img2->time_ns = img1->time_ns;
        img2->index = img1->index;
        img2->refcnt = 1;
        img2->backend = img1->backend;
        img2->format = OCCAM_GRAY8;
        img2->width = img1->width;
        img2->height = img1->height;
        img2->subimage_count = img1->subimage_count;
        memcpy(img2->si_x,img1->si_x,sizeof(img1->si_x));
        memcpy(img2->si_y,img1->si_y,sizeof(img1->si_y));
        memcpy(img2->si_width,img1->si_width,sizeof(img1->si_width));
        memcpy(img2->si_height,img1->si_height,sizeof(img1->si_height));
        img2->step[0] = (img2->width+15)&~15;
        img2->data[0] = new uint8_t[img2->height*img2->step[0]];

        unsigned char* srcp = img1->data[0];
        unsigned char* dstp = img2->data[0];
        for (int y=0;y<img1->height;++y,srcp+=img1->step[0],dstp+=img2->step[0])
            for (int x=0,x3=0;x<img1->width;++x,x3+=3)
                dstp[x] = (int(srcp[x3+0])*4899+int(srcp[x3+1])*9617+int(srcp[x3+2])*1868)>>14;

        return std::shared_ptr<OccamImage>(img2,occamFreeImage);
    };
    return DeferredImage(gen_fn,img0);
}

static DeferredImage rectifyImage(std::shared_ptr<void> rectify_handle,
        int index,
        DeferredImage img0) {
    auto gen_fn = [=](){
        OccamImage* img1 = img0->get();
        IOccamStereoRectify* rectify_iface = 0;
        occamGetInterface(rectify_handle.get(),IOCCAMSTEREORECTIFY,(void**)&rectify_iface);
        OccamImage* img2 = 0;
        rectify_iface->rectify(rectify_handle.get(),index,img1,&img2);
        return std::shared_ptr<OccamImage>(img2,occamFreeImage);
    };  
    return DeferredImage(gen_fn,img0);
}

static DeferredImage unrectifyImage(std::shared_ptr<void> rectify_handle,
        int index,
        DeferredImage img0) {
    auto gen_fn = [=](){
        OccamImage* img1 = img0->get();
        IOccamStereoRectify* rectify_iface = 0;
        occamGetInterface(rectify_handle.get(),IOCCAMSTEREORECTIFY,(void**)&rectify_iface);
        OccamImage* img2 = 0;
        rectify_iface->unrectify(rectify_handle.get(),index,img1,&img2);
        return std::shared_ptr<OccamImage>(img2,occamFreeImage);
    };  
    return DeferredImage(gen_fn,img0);
}

static Mat occamImageToCvMat(OccamImage *image) {
    Mat img;
    if (image && image->format == OCCAM_GRAY8)
        img = Mat_<uchar>(image->height,image->width,(uchar*)image->data[0],image->step[0]);
    else if (image && image->format == OCCAM_RGB24) {
        img = Mat_<Vec3b>(image->height,image->width,(Vec3b*)image->data[0],image->step[0]);
        Mat img1;
        cvtColor(img, img1, COLOR_BGR2RGB);
        img = img1;
    } else if (image && image->format == OCCAM_SHORT1) {
        img = Mat_<short>(image->height,image->width,(short*)image->data[0],image->step[0]);
    } else {
        //      printf("image format not supported by this demo\n");
    }
    auto s = img.size();
    // printf("width: %d, height: %d\n", s.width, s.height);
    return img;
}

static void convertImage(const Mat& src, OccamImage* dst) {
	memset(dst,0,sizeof(*dst));
	dst->refcnt = 1;
	dst->backend = OCCAM_CPU;
	dst->cid = (char*)"";
	dst->width = src.cols;
	dst->height = src.rows;
	dst->step[0] = src.step;
	dst->data[0] = src.data;
    if (src.type() == CV_8UC1) {
        dst->format = OCCAM_GRAY8;
    } else if (src.type() == CV_16SC1) {
        dst->format = OCCAM_SHORT1;
    } else if (src.type() == CV_8UC3) {
        dst->format = OCCAM_RGB24;
    } else {
	std::cerr<<"Unknown image type "<<src.type()<<std::endl;
	abort();
	}
}

static std::string getImageType(int number)
{
    // find type
    int imgTypeInt = number%8;
    std::string imgTypeString;

    switch (imgTypeInt)
    {
        case 0:
            imgTypeString = "8U";
            break;
        case 1:
            imgTypeString = "8S";
            break;
        case 2:
            imgTypeString = "16U";
            break;
        case 3:
            imgTypeString = "16S";
            break;
        case 4:
            imgTypeString = "32S";
            break;
        case 5:
            imgTypeString = "32F";
            break;
        case 6:
            imgTypeString = "64F";
            break;
        default:
            break;
    }

    // find channel
    int channel = (number/8) + 1;

    std::stringstream type;
    type<<"CV_"<<imgTypeString<<"C"<<channel;

    return type.str();
}

static DeferredImage computeDisparityImage(std::shared_ptr<void> stereo_handle,
        int index,
        DeferredImage img0r,
        DeferredImage img1r) {
    auto gen_fn = [=](){
        OccamImage* img0rp = img0r->get();
        OccamImage* img1rp = img1r->get();
        IOccamStereo* stereo_iface = 0;
        occamGetInterface(stereo_handle.get(),IOCCAMSTEREO,(void**)&stereo_iface);
        OccamImage* disp = 0;
        stereo_iface->compute(stereo_handle.get(),index,img0rp,img1rp,&disp);
return std::shared_ptr<OccamImage>(disp,occamFreeImage);
    };  
    return DeferredImage(gen_fn,img0r,img1r);
}

static DeferredPointCloud computePointCloud(std::shared_ptr<void> rectify_handle,
        int index,
        DeferredImage img0,
        DeferredImage disp0) {
    auto gen_fn = [=](){
        IOccamStereoRectify* rectify_iface = 0;
        occamGetInterface(rectify_handle.get(),IOCCAMSTEREORECTIFY,(void**)&rectify_iface);
        OccamPointCloud* cloud1 = 0;
        const OccamImage* img0p = img0->get();
        const OccamImage* disp0p = disp0->get();
        rectify_iface->generateCloud(rectify_handle.get(),1,&index,0,&img0p,&disp0p,&cloud1);
        return std::shared_ptr<OccamPointCloud>(cloud1,occamFreePointCloud);
    };
    return DeferredPointCloud(gen_fn,img0,disp0);
}

static DeferredPointCloud computePointCloud(std::shared_ptr<void> rectify_handle,
        std::vector<int> indices,
        const std::vector<DeferredImage>& img0,
        const std::vector<DeferredImage>& disp0) {
    auto gen_fn = [=](){
        IOccamStereoRectify* rectify_iface = 0;
        occamGetInterface(rectify_handle.get(),IOCCAMSTEREORECTIFY,(void**)&rectify_iface);
        OccamPointCloud* cloud1 = 0;
        const OccamImage** img0p = (const OccamImage**)alloca(sizeof(const OccamImage*)*img0.size());
        const OccamImage** disp0p = (const OccamImage**)alloca(sizeof(const OccamImage*)*disp0.size());
        assert(img0.size() == disp0.size());
        for (int j=0;j<img0.size();++j) {
            img0p[j] = img0[j]->get();
            disp0p[j] = disp0[j]->get();
        }
        rectify_iface->generateCloud(rectify_handle.get(),indices.size(),&indices[0],1,img0p,disp0p,&cloud1);
        return std::shared_ptr<OccamPointCloud>(cloud1,occamFreePointCloud);
    };
    const Deferred** deps = (const Deferred**)alloca(sizeof(Deferred*)*(img0.size()+disp0.size()));
    for (int j=0;j<img0.size();++j)
        deps[j] = &img0[j];
    for (int j=0;j<img0.size();++j)
        deps[img0.size()+j] = &disp0[j];
    return DeferredPointCloud(gen_fn,img0.size()+disp0.size(),deps);
}

static int heatmapImage(const OccamImage* img0, OccamImage** img1out,
        int min_value = 0, int max_value = 64*16) {
    if (img0->format != OCCAM_SHORT1)
        return OCCAM_API_INVALID_FORMAT;

    OccamImage* img1 = new OccamImage;
    *img1out = img1;
    memset(img1,0,sizeof(OccamImage));
    img1->cid = strdup(img0->cid);
    memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
    img1->time_ns = img0->time_ns;
    img1->index = img0->index;
    img1->refcnt = 1;
    img1->backend = OCCAM_CPU;
    img1->format = OCCAM_RGB24;
    img1->width = img0->width;
    img1->height = img0->height;
    img1->subimage_count = img0->subimage_count;
    memcpy(img1->si_x,img0->si_x,sizeof(img1->si_x));
    memcpy(img1->si_y,img0->si_y,sizeof(img1->si_y));
    memcpy(img1->si_width,img0->si_width,sizeof(img1->si_width));
    memcpy(img1->si_height,img0->si_height,sizeof(img1->si_height));
    img1->step[0] = (img1->width*3+15)&~15;
    img1->data[0] = new uint8_t[img1->step[0]*img1->height];

    const int num_colors = 6;
    const int colors[6*3] = {
        255,0,255,
        0,0,255,
        0,255,255,
        0,255,0,
        255,255,0,
        255,0,0
    };
    int num_values = max_value - min_value;
    uint8_t* lut = (uint8_t*)alloca(num_values*3);
    auto clip = [](int value) {
        return uint8_t(std::min(255,std::max(0,value)));
    };
    for (int j=min_value,k=0;j<max_value;++j,k+=3) {
        float v = float(j)*num_colors/num_values;
        int c0 = int(std::max(0,std::min(num_colors-1,int(floor(v)))));
        int c1 = int(std::min(num_colors-1,c0+1));
        float vj = v - floor(v);
        float vi = 1.f - vj;
        lut[k+0] = clip(int(colors[c0*3+0]*vi + colors[c1*3+0]*vj));
        lut[k+1] = clip(int(colors[c0*3+1]*vi + colors[c1*3+1]*vj));
        lut[k+2] = clip(int(colors[c0*3+2]*vi + colors[c1*3+2]*vj));
    }

    if (img0->format == OCCAM_SHORT1) {
        const uint8_t* srcp0 = img0->data[0];
        uint8_t* dstp0 = img1->data[0];
        for (int y=0;y<img1->height;++y,srcp0+=img0->step[0],dstp0+=img1->step[0]) {
            const short* srcp = (const short*)srcp0;
            uint8_t* dstp = dstp0;
            for (int x=0;x<img1->width;++x,dstp+=3,srcp++) {
                int value = *srcp - min_value;
                if (value < 0 || value > max_value) {
                    dstp[0] = 0;
                    dstp[1] = 0;
                    dstp[2] = 0;
                } else {
                    const uint8_t* lut_value = lut+value*3;
                    dstp[0] = lut_value[0];
                    dstp[1] = lut_value[1];
                    dstp[2] = lut_value[2];
                }
            }
        }
    }

    return OCCAM_API_SUCCESS;
}

static DeferredImage heatmapImage(DeferredImage img0) {
    auto gen_fn = [=](){
        OccamImage* img0p = img0->get();
        OccamImage* img1p = 0;
        heatmapImage(img0p, &img1p);
        return std::shared_ptr<OccamImage>(img1p,occamFreeImage);
    };  
    return DeferredImage(gen_fn,img0);
}

static int makeRGBImage(const OccamImage* img0, OccamImage** img1out) {
    if (img0->format != OCCAM_GRAY8 && 
            img0->format != OCCAM_RGB24)
        return OCCAM_API_INVALID_FORMAT;
    if (img0->format == OCCAM_RGB24)
        return occamCopyImage(img0,img1out,0);

    OccamImage* img1 = new OccamImage;
    *img1out = img1;
    memset(img1,0,sizeof(OccamImage));
    img1->cid = strdup(img0->cid);
    memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
    img1->time_ns = img0->time_ns;
    img1->index = img0->index;
    img1->refcnt = 1;
    img1->backend = OCCAM_CPU;
    img1->format = OCCAM_RGB24;
    img1->width = img0->width;
    img1->height = img0->height;
    img1->subimage_count = img0->subimage_count;
    memcpy(img1->si_x,img0->si_x,sizeof(img1->si_x));
    memcpy(img1->si_y,img0->si_y,sizeof(img1->si_y));
    memcpy(img1->si_width,img0->si_width,sizeof(img1->si_width));
    memcpy(img1->si_height,img0->si_height,sizeof(img1->si_height));
    img1->step[0] = (img1->width*3+15)&~15;
    img1->data[0] = new uint8_t[img1->step[0]*img1->height];

    if (img0->format == OCCAM_GRAY8) {
        const uint8_t* srcp0 = img0->data[0];
        uint8_t* dstp0 = img1->data[0];
        for (int y=0;y<img1->height;++y,srcp0+=img0->step[0],dstp0+=img1->step[0]) {
            const uint8_t* srcp = srcp0;
            uint8_t* dstp = dstp0;
            for (int x=0;x<img1->width;++x,dstp+=3,srcp++) {
                dstp[0] = *srcp;
                dstp[1] = *srcp;
                dstp[2] = *srcp;
            }
        }
    }

    return OCCAM_API_SUCCESS;
}

static DeferredImage makeRGBImage(DeferredImage img0) {
    auto gen_fn = [=](){
        OccamImage* img0p = img0->get();
        OccamImage* img1p = 0;
        makeRGBImage(img0p, &img1p);
        return std::shared_ptr<OccamImage>(img1p,occamFreeImage);
    };  
    return DeferredImage(gen_fn,img0);
}

static DeferredImage blendImages(std::shared_ptr<void> blend_handle,
        const std::vector<DeferredImage>& srcimg) {
    auto gen_fn = [=](){
        OccamImage* img1 = 0;
        int N = srcimg.size();
        OccamImage** img0 = (OccamImage**)alloca(N*sizeof(OccamImage*));
        for (int j=0;j<N;++j)
            img0[j] = srcimg[j]->get();
        IOccamBlendFilter* blend_iface = 0;
        occamGetInterface(blend_handle.get(),IOCCAMBLENDFILTER,(void**)&blend_iface);
        blend_iface->compute(blend_handle.get(),img0,&img1);
        return std::shared_ptr<OccamImage>(img1,occamFreeImage);
    };
    const Deferred** deps = (const Deferred**)alloca(sizeof(Deferred*)*srcimg.size());
    for (int j=0;j<srcimg.size();++j)
        deps[j] = &srcimg[j];
    return DeferredImage(gen_fn,srcimg.size(),deps);
}

class OccamDevice_omnis5u3mt9v022 : public OccamMetaDeviceBase {
    OmniDevice* top;
    OmniDevice* bottom;
    ImageCollector pair_collect;
    bool loaded_settings;
    int target_fps;
    int filter_sigma;
    int filter_lambda;
    int filter_ddr;

    int bm_prefilter_size;
    int bm_prefilter_cap;
    int bm_sad_window_size;
    int bm_min_disparity;
    int bm_num_disparities;
    int bm_texture_threshold;
    int bm_uniqueness_ratio;
    int bm_speckle_range;
    int bm_speckle_window_size;

    double D[10][5];
    double K[10][9];
    double R[10][9];
    double T[10][3];

    // exposure/gain options
    int get_exposure() {
        return int(program("w0xcc02=1;w0xcc01=0xb8;r0xb")[0]);
    }
    void set_exposure(int value) {
        value = std::min(std::max(1,value),480);
        program("w0xcc02=1;w0xcc01=0xb8;w0xb=?",value);
    }
    int get_exposure_microseconds() {
        return int(get_exposure()*32.538);
    }
    void set_exposure_microseconds(int value) {
        set_exposure(int(value/32.538));
    }
    bool get_auto_exposure() {
        return int(program("w0xcc02=1;w0xcc01=0xb8;r0xaf")[0])&1?true:false;
    }
    void set_auto_exposure(bool value) {
        while (get_auto_exposure() != value)
            program("w0xcc02=1;w0xcc01=0xb8;W0xaf(1)=?",value?1:0);
    }
    int get_gain() {
        return int(program("w0xcc02=1;w0xcc01=0xb8;r0x35")[0]);
    }
    void set_gain(int value) {
        value = std::min(std::max(1,value),64);
        program("w0xcc02=1;w0xcc01=0xb8;W0x35(127)=?",value);
    }
    bool get_auto_gain() {
        return int(program("w0xcc02=1;w0xcc01=0xb8;r0xaf")[0]&2)?true:false;
    }
    void set_auto_gain(bool value) {
        while (get_auto_gain() != value)
            program("w0xcc02=1;w0xcc01=0xb8;W0xaf(2)=?",value?2:0);
    }

    void set_target_fps(int value) {
        if (value != 15 && value != 30 && value != 60)
            return;
        target_fps = value;
        if (target_fps == 60) {
            setMinMaxValues(OCCAM_EXPOSURE,1,480);
            setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(480*32.538));
        } else if (target_fps == 30) {
            setMinMaxValues(OCCAM_EXPOSURE,1,960);
            setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(960*32.538));
        } else if (target_fps == 15) {
            setMinMaxValues(OCCAM_EXPOSURE,1,1920);
            setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(1920*32.538));
        }
        initDevices();
    }
    int get_target_fps() {
        return target_fps;
    }

    int get_wire_fps() {
        int wire_fps = 0;
        if (top)
            top->getDeviceValuei(OCCAM_WIRE_FPS,&wire_fps);
        else if (bottom)
            bottom->getDeviceValuei(OCCAM_WIRE_FPS,&wire_fps);
        return wire_fps;
    };

    int nextVal(std::vector<int> vec, int value, int last) {
        std::vector<int> diff;
        for (auto i = vec.begin(); i != vec.end(); ++i) { diff.push_back(abs(value-*i)); }
        auto idx = std::distance(diff.begin(), std::min_element(diff.begin(), diff.end()));
        int min_count = 0; for(auto it = diff.begin(); it != diff.end() ; ++it) { if (*it==diff[idx]) { min_count++; } }
        if (min_count > 1) {
            if (value < last)
                idx++;
        }
        if (value == vec[idx]) { return vec[idx]; }
        else if (value > last) { return idx >= vec.size()-1 ? vec[vec.size()-1] : vec[idx+1]; }
        else if (value < last) { return idx <= 0 ? vec[0] : vec[idx-1]; }
    }

    int get_adc_vref() {
        return int(program("w0xcc02=1;w0xcc01=0xb8;r0x2c")[0]);
    }
    void set_adc_vref(int value) {
        program("w0xcc02=1;w0xcc01=0xb8;w0x2c=?",value);
    }

    bool get_adc_companding() {
        return int(program("w0xcc02=1;w0xcc01=0xb8;r0x1c")[0])==3;
    }
    void set_adc_companding(bool value) {
        program("w0xcc02=1;w0xcc01=0xb8;w0x1c=?",value?3:2);
    }

    // camera information
    int get_sensor_width() {
        return 752;
    }
    int get_sensor_height() {
        return 480;
    }
    int get_sensor_count() {
        return 10;
    }
    int get_wire_bps() {
        int top_bps = 0;
        int bottom_bps = 0;
        if (top)
            top->getDeviceValuei(OCCAM_WIRE_BPS,&top_bps);
        if (bottom)
            bottom->getDeviceValuei(OCCAM_WIRE_BPS,&bottom_bps);
        return top_bps + bottom_bps;
    };
    bool get_color() {
        if (!top)
            return false;
        char cn[] = {top->serial().end()[-1],0};
        return atoi(cn)&1;
    }
    void get_D(int index, double* D0) {
        std::copy(D[index],D[index]+5,D0);
    }
    void set_D(int index, const double* D0) {
        std::copy(D0,D0+5,D[index]);
    }
    void get_K(int index, double* K0) {
        std::copy(K[index],K[index]+9,K0);
    }
    void set_K(int index, const double* K0) {
        std::copy(K0,K0+9,K[index]);
    }
    void get_R(int index, double* R0) {
        std::copy(R[index],R[index]+9,R0);
    }
    void set_R(int index, const double* R0) {
        std::copy(R0,R0+9,R[index]);
    }
    void get_T(int index, double* T0) {
        std::copy(T[index],T[index]+3,T0);
    }
    void set_T(int index, const double* T0) {
        std::copy(T0,T0+3,T[index]);
    }

    void read_geometric_calib(OmniDevice* dev) {
#pragma pack(push,1)
        struct GeometricCalibData_generic {
            uint32_t magic;
            float D[10][5];
            float K[10][9];
            float R[10][9];
            float T[10][3];
        };
#pragma pack(pop)
        GeometricCalibData_generic data;
        if (!dev->readDeviceData(0, 256, &data, sizeof(data))) {
            std::cerr<<"failed reading geometric calibration (device)"<<std::endl;
            return;
        }
        const uint32_t GEOMETRIC_CALIB_MAGIC = 0x0e2f4f11;
        if (data.magic != GEOMETRIC_CALIB_MAGIC) {
            std::cerr<<"failed reading geometric calibration (header)"<<std::endl;
            return;
        }
        for (int j=0;j<10;++j) {
            for (int k=0;k<5;++k)
                D[j][k] = data.D[j][k];
            for (int k=0;k<9;++k)
                K[j][k] = data.K[j][k];
            for (int k=0;k<9;++k)
                R[j][k] = data.R[j][k];
            for (int k=0;k<3;++k)
                T[j][k] = data.T[j][k];
        }
    }

    virtual OccamDeviceBase* addDevice(const std::string& cid) {
        OmniDevice* dev = new OmniDevice(cid);
        char flags_str[] = {dev->serial().end()[-1], 0};
        int flags = strtol(flags_str, 0, 16);
        if (flags&2) {
            top = dev;
            read_geometric_calib(top);
        }
        if (flags&4) {
            bottom = dev;
            read_geometric_calib(bottom);
        }
        return dev;
    }
    virtual void clearDevices() {
        top = 0;
        bottom = 0;
    }

    public:
    OccamDevice_omnis5u3mt9v022(const std::string& cid)
        : OccamMetaDeviceBase(cid, 2),
        top(0),
        bottom(0),
        loaded_settings(false),
        target_fps(60),
        filter_lambda(30),
        filter_sigma(10),
        filter_ddr(5),
        bm_prefilter_size(9),
        bm_prefilter_cap(31),
        bm_sad_window_size(15),
        bm_min_disparity(0),
        bm_num_disparities(64),
        bm_texture_threshold(10),
        bm_uniqueness_ratio(60),
        bm_speckle_range(120),
        bm_speckle_window_size(40)
        {

            for (int j=0;j<10;++j) {
                double* Dj = D[j];
                double* Kj = K[j];
                double* Rj = R[j];
                double* Tj = T[j];
                Dj[0]=Dj[1]=Dj[2]=Dj[3]=Dj[4]=0;
                Kj[0]=1,Kj[1]=0,Kj[2]=0,Kj[3]=0,Kj[4]=1,Kj[5]=0,Kj[6]=0,Kj[7]=0,Kj[8]=1;
                Rj[0]=1,Rj[1]=0,Rj[2]=0,Rj[3]=0,Rj[4]=1,Rj[5]=0,Rj[6]=0,Rj[7]=0,Rj[8]=1;
                Tj[0]=Tj[1]=Tj[2]=0;
            }

            using namespace std::placeholders;

            addConfigurableModule(OCCAM_STEREO_MATCHER0,"stereo_matcher0",
                    OCCAM_MODULE_STEREO);
            addConfigurableModule(OCCAM_STEREO_RECTIFIER0,"stereo_rectifier0",
                    OCCAM_MODULE_STEREO_RECTIFY);
            addConfigurableModule(OCCAM_DEBAYER_FILTER0,"debayer_filter0",
                    OCCAM_MODULE_DEBAYER_FILTER);
            addConfigurableModule(OCCAM_IMAGE_FILTER0,"image_filter0",
                    OCCAM_MODULE_IMAGE_FILTER);
            addConfigurableModule(OCCAM_UNDISTORT_FILTER0,"undistort_filter0",
                    OCCAM_MODULE_UNDISTORT_FILTER);
            addConfigurableModule(OCCAM_BLENDER0,"blender0",
                    OCCAM_MODULE_BLEND_FILTER);

            registerParami(OCCAM_EXPOSURE,"exposure",OCCAM_SETTINGS, 1,480,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_exposure,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_exposure,this,_1));
            registerParami(OCCAM_EXPOSURE_MICROSECONDS,"exposure_microseconds",OCCAM_NOT_STORED,1,1000000,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_exposure_microseconds,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_exposure_microseconds,this,_1));
            registerParamb(OCCAM_AUTO_EXPOSURE,"auto_exposure",OCCAM_SETTINGS,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_auto_exposure,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_auto_exposure,this,_1));
            registerParami(OCCAM_GAIN,"gain",OCCAM_SETTINGS,17,64,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_gain,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_gain,this,_1));
            registerParamb(OCCAM_AUTO_GAIN,"auto_gain",OCCAM_SETTINGS,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_auto_gain,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_auto_gain,this,_1));
            setDefaultDeviceValuei(OCCAM_EXPOSURE,480);
            setDefaultDeviceValueb(OCCAM_AUTO_EXPOSURE,true);
            setDefaultDeviceValuei(OCCAM_GAIN,17);
            setDefaultDeviceValueb(OCCAM_AUTO_GAIN,true);

            registerParami(OCCAM_TARGET_FPS,"target_fps",OCCAM_SETTINGS,0,0,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_target_fps,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_target_fps,this,_1));
            std::vector<std::pair<std::string,int> > target_fps_values;
            target_fps_values.push_back(std::make_pair("60", 60));
            target_fps_values.push_back(std::make_pair("30", 30));
            target_fps_values.push_back(std::make_pair("15", 15));
            setAllowedValues(OCCAM_TARGET_FPS,target_fps_values);
            setDefaultDeviceValuei(OCCAM_TARGET_FPS,60);

            registerParami(OCCAM_ADC_VREF,"adc_vref",OCCAM_SETTINGS,0,0,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_adc_vref,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_adc_vref,this,_1));
            std::vector<std::pair<std::string,int> > adc_vref_values;
            adc_vref_values.push_back(std::make_pair("1.0V",0));
            adc_vref_values.push_back(std::make_pair("1.1V",1));
            adc_vref_values.push_back(std::make_pair("1.2V",2));
            adc_vref_values.push_back(std::make_pair("1.3V",3));
            adc_vref_values.push_back(std::make_pair("1.4V",4));
            adc_vref_values.push_back(std::make_pair("1.5V",5));
            adc_vref_values.push_back(std::make_pair("1.6V",6));
            adc_vref_values.push_back(std::make_pair("2.1V",7));
            setAllowedValues(OCCAM_ADC_VREF,adc_vref_values);
            setDefaultDeviceValuei(OCCAM_ADC_VREF,4);
            registerParamb(OCCAM_ADC_COMPANDING,"adc_companding",OCCAM_SETTINGS,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_adc_companding,this),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_adc_companding,this,_1));
            setDefaultDeviceValueb(OCCAM_ADC_COMPANDING,false);

            registerParami(OCCAM_SENSOR_WIDTH,"sensor_width",OCCAM_NOT_STORED,0,0,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_sensor_width,this));
            registerParami(OCCAM_SENSOR_HEIGHT,"sensor_height",OCCAM_NOT_STORED,0,0,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_sensor_height,this));
            registerParami(OCCAM_SENSOR_COUNT,"sensor_count",OCCAM_NOT_STORED,0,0,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_sensor_count,this));
            registerParamb(OCCAM_COLOR,"color",OCCAM_NOT_STORED,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_color,this));

            registerParami(OCCAM_WIRE_FPS,"wire_fps",OCCAM_NOT_STORED,0,0,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_wire_fps,this));
            registerParami(OCCAM_WIRE_BPS,"wire_bps",OCCAM_NOT_STORED,0,0,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_wire_bps,this));

            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS0,
                    "D[0]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,0,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,0,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS1,
                    "D[1]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,1,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,1,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS2,
                    "D[2]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,2,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,2,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS3,
                    "D[3]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,3,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,3,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS4,
                    "D[4]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,4,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,4,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS5,
                    "D[5]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,5,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,5,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS6,
                    "D[6]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,6,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,6,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS7,
                    "D[7]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,7,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,7,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS8,
                    "D[8]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,8,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,8,_1));
            registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS9,
                    "D[9]", OCCAM_CALIBRATION, 0, 0, 5,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_D,this,9,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_D,this,9,_1));

            registerParamrv(OCCAM_SENSOR_INTRINSICS0,
                    "K[0]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,0,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,0,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS1,
                    "K[1]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,1,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,1,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS2,
                    "K[2]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,2,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,2,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS3,
                    "K[3]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,3,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,3,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS4,
                    "K[4]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,4,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,4,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS5,
                    "K[5]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,5,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,5,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS6,
                    "K[6]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,6,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,6,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS7,
                    "K[7]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,7,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,7,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS8,
                    "K[8]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,8,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,8,_1));
            registerParamrv(OCCAM_SENSOR_INTRINSICS9,
                    "K[9]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_K,this,9,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_K,this,9,_1));

            registerParamrv(OCCAM_SENSOR_ROTATION0,
                    "R[0]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,0,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,0,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION1,
                    "R[1]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,1,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,1,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION2,
                    "R[2]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,2,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,2,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION3,
                    "R[3]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,3,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,3,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION4,
                    "R[4]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,4,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,4,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION5,
                    "R[5]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,5,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,5,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION6,
                    "R[6]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,6,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,6,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION7,
                    "R[7]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,7,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,7,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION8,
                    "R[8]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,8,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,8,_1));
            registerParamrv(OCCAM_SENSOR_ROTATION9,
                    "R[9]", OCCAM_CALIBRATION, 0, 0, 9,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_R,this,9,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_R,this,9,_1));

            registerParamrv(OCCAM_SENSOR_TRANSLATION0,
                    "T[0]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,0,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,0,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION1,
                    "T[1]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,1,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,1,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION2,
                    "T[2]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,2,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,2,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION3,
                    "T[3]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,3,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,3,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION4,
                    "T[4]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,4,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,4,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION5,
                    "T[5]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,5,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,5,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION6,
                    "T[6]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,6,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,6,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION7,
                    "T[7]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,7,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,7,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION8,
                    "T[8]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,8,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,8,_1));
            registerParamrv(OCCAM_SENSOR_TRANSLATION9,
                    "T[9]", OCCAM_CALIBRATION, 0, 0, 3,
                    std::bind(&OccamDevice_omnis5u3mt9v022::get_T,this,9,_1),
                    std::bind(&OccamDevice_omnis5u3mt9v022::set_T,this,9,_1));

            updateDevices();
        }

    virtual void initDevices() {
        if (!top || !bottom)
            return;
        if (!loaded_settings) {
            setDeviceValueb(OCCAM_IMAGE_PROCESSING_ENABLED,get_color());
            setDefaultDeviceValueb(OCCAM_IMAGE_PROCESSING_ENABLED,get_color());
            setDefaultDeviceValueb(OCCAM_COLOR,get_color());
            setDefaultDeviceValuei(OCCAM_BRIGHTNESS,1970);
            setDefaultDeviceValuei(OCCAM_GAMMA,1000);
            setDefaultDeviceValuei(OCCAM_BLACK_LEVEL,20);
            setDefaultDeviceValuei(OCCAM_WHITE_BALANCE_RED,1177);
            setDefaultDeviceValuei(OCCAM_WHITE_BALANCE_GREEN,1268);
            setDefaultDeviceValuei(OCCAM_WHITE_BALANCE_BLUE,1645);

            loaded_settings = true;
            loadSettings();
        }
        if (target_fps==60)
            program("w0xcc30=0;w0xcc32=133;w0xcc02=1;w0xcc01=0xb8;w0x6=30;W0xd(0x30)=0x30;"
                    "W0xa(0xf)=5;w0xcc02=2;w0xcc01=0x94;w8115=10000;w8116=8460;w8117=423;w8114=2;"
                    "w0xdd01=1;");
        else if (target_fps==30)
            program("w0xcc30=0;w0xcc32=266;w0xcc02=1;w0xcc01=0xb8;w0x6=541;W0xd(0x30)=0x30;"
                    "w0xcc02=2;w0xcc01=0x94;w8115=10000;w8116=8460;w8117=846;w8114=2;w0xdd01=1;");
        else if (target_fps==15)
            program("w0xcc30=0;w0xcc32=533;w0xcc02=1;w0xcc01=0xb8;w0x6=1566;W0xd(0x30)=0x30;"
                    "w0xcc02=2;w0xcc01=0x94;w8115=10000;w8116=3000;w8117=850;w8114=2;w0xdd01=1;");

        pair_collect.addStream(0,top);
        pair_collect.addStream(1,bottom);
    }

    virtual int writeRegister(uint32_t addr, uint32_t value) {
        if (addr == 0xdd01) {
            uint32_t adjust = 0;
            int r;
            if ((r = top->readRegister(0xcc31, &adjust)) != OCCAM_API_SUCCESS)
                return r;
            if ((r = top->writeRegister(0xcc31, adjust)) != OCCAM_API_SUCCESS)
                return r;
            if ((r = bottom->writeRegister(0xcc31, adjust)) != OCCAM_API_SUCCESS)
                return r;
            if ((r = top->writeRegister(0xcc30, 1)) != OCCAM_API_SUCCESS)
                return r;
            if ((r = bottom->writeRegister(0xcc30, 1)) != OCCAM_API_SUCCESS)
                return r;
            return OCCAM_API_SUCCESS;
        } else
            return OccamMetaDeviceBase::writeRegister(addr, value);
    }

    virtual int readData(DeviceOutput& out) {
        if (!top || !bottom)
            updateDevices();

        const int sensor_width = 752;
        const int sensor_height = 480;
        bool is_color = get_color();
        // printf("retrieving color image...");

        std::shared_ptr<OccamImage> imgout[2];
        int r0 = pair_collect.read(imgout, 2);
        if (r0 != OCCAM_API_SUCCESS)
            return r0;

        assert(bool(imgout[0]) && bool(imgout[1]));
        occamFree(imgout[0]->cid);
        occamFree(imgout[1]->cid);
        imgout[0]->cid = strdup(cid().c_str());
        imgout[1]->cid = strdup(cid().c_str());
        DeferredImage img0(imgout[0]);
        DeferredImage img1(imgout[1]);

        auto img0_raw0 = subImage(img0,0,0*sensor_height,sensor_width,sensor_height);
        auto img0_raw1 = subImage(img0,0,2*sensor_height,sensor_width,sensor_height);
        auto img0_raw2 = subImage(img0,0,4*sensor_height,sensor_width,sensor_height);
        auto img0_raw3 = subImage(img0,0,1*sensor_height,sensor_width,sensor_height);
        auto img0_raw4 = subImage(img0,0,3*sensor_height,sensor_width,sensor_height);
        auto img1_raw0 = subImage(img1,0,0*sensor_height,sensor_width,sensor_height);
        auto img1_raw1 = subImage(img1,0,2*sensor_height,sensor_width,sensor_height);
        auto img1_raw2 = subImage(img1,0,4*sensor_height,sensor_width,sensor_height);
        auto img1_raw3 = subImage(img1,0,1*sensor_height,sensor_width,sensor_height);
        auto img1_raw4 = subImage(img1,0,3*sensor_height,sensor_width,sensor_height);
        out.set(OCCAM_RAW_IMAGE0,img0_raw0);
        out.set(OCCAM_RAW_IMAGE2,img0_raw1);
        out.set(OCCAM_RAW_IMAGE4,img0_raw2);
        out.set(OCCAM_RAW_IMAGE6,img0_raw3);
        out.set(OCCAM_RAW_IMAGE8,img0_raw4);
        out.set(OCCAM_RAW_IMAGE1,img1_raw0);
        out.set(OCCAM_RAW_IMAGE3,img1_raw1);
        out.set(OCCAM_RAW_IMAGE5,img1_raw2);
        out.set(OCCAM_RAW_IMAGE7,img1_raw3);
        out.set(OCCAM_RAW_IMAGE9,img1_raw4);

        {
            auto htile0 = htile({img0_raw0,img0_raw1,img0_raw2,img0_raw3,img0_raw4});
            auto htile1 = htile({img1_raw0,img1_raw1,img1_raw2,img1_raw3,img1_raw4});
            out.set(OCCAM_RAW_IMAGE_TILES0,vtile({htile0,htile1}));
        }

        std::shared_ptr<void> debayerf_handle = module(OCCAM_DEBAYER_FILTER0);
        std::shared_ptr<void> imagef_handle = module(OCCAM_IMAGE_FILTER0);
        auto img0_pro0 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw0);
        auto img0_pro1 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw1);
        auto img0_pro2 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw2);
        auto img0_pro3 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw3);
        auto img0_pro4 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw4);
        auto img1_pro0 = processImage(imagef_handle,debayerf_handle,is_color,img1_raw0);
        auto img1_pro1 = processImage(imagef_handle,debayerf_handle,is_color,img1_raw1);
        auto img1_pro2 = processImage(imagef_handle,debayerf_handle,is_color,img1_raw2);
        auto img1_pro3 = processImage(imagef_handle,debayerf_handle,is_color,img1_raw3);
        auto img1_pro4 = processImage(imagef_handle,debayerf_handle,is_color,img1_raw4);

        out.set(OCCAM_IMAGE0,img0_pro0);
        out.set(OCCAM_IMAGE2,img0_pro1);
        out.set(OCCAM_IMAGE4,img0_pro2);
        out.set(OCCAM_IMAGE6,img0_pro3);
        out.set(OCCAM_IMAGE8,img0_pro4);
        out.set(OCCAM_IMAGE1,img1_pro0);
        out.set(OCCAM_IMAGE3,img1_pro1);
        out.set(OCCAM_IMAGE5,img1_pro2);
        out.set(OCCAM_IMAGE7,img1_pro3);
        out.set(OCCAM_IMAGE9,img1_pro4);
        {
            auto htile0 = htile({img0_pro0,img0_pro1,img0_pro2,img0_pro3,img0_pro4});
            auto htile1 = htile({img1_pro0,img1_pro1,img1_pro2,img1_pro3,img1_pro4});
            out.set(OCCAM_IMAGE_TILES0,vtile({htile0,htile1}));
        }

        std::shared_ptr<void> blend_handle = module(OCCAM_BLENDER0);
        {
            IOccamBlendFilter* blend_iface = 0;
            occamGetInterface(blend_handle.get(),IOCCAMBLENDFILTER,(void**)&blend_iface);
            int sensor_width[] = {752,752,752,752,752};
            int sensor_height[] = {480,480,480,480,480};
            double* Dp[] = {D[0],D[1],D[2],D[3],D[4]};
            double* Kp[] = {K[0],K[1],K[2],K[3],K[4]};
            double* Rp[] = {R[0],R[1],R[2],R[3],R[4]};
            double* Tp[] = {T[0],T[1],T[2],T[3],T[4]};
            blend_iface->configure(blend_handle.get(),5,sensor_width,sensor_height,Dp,Kp,Rp,Tp);
        }
        auto img0_blend =
            blendImages(blend_handle,{img0_pro0,img0_pro1,img0_pro2,img0_pro3,img0_pro4});
        out.set(OCCAM_STITCHED_IMAGE0,img0_blend);

        auto img0_mon0 = makeMonoImage(img0_pro0);
        auto img0_mon1 = makeMonoImage(img0_pro1);
        auto img0_mon2 = makeMonoImage(img0_pro2);
        auto img0_mon3 = makeMonoImage(img0_pro3);
        auto img0_mon4 = makeMonoImage(img0_pro4);
        auto img1_mon0 = makeMonoImage(img1_pro0);
        auto img1_mon1 = makeMonoImage(img1_pro1);
        auto img1_mon2 = makeMonoImage(img1_pro2);
        auto img1_mon3 = makeMonoImage(img1_pro3);
        auto img1_mon4 = makeMonoImage(img1_pro4);

        std::shared_ptr<void> rectify_handle = module(OCCAM_STEREO_RECTIFIER0);
        double* Dp[] = {D[0],D[5],D[1],D[6],D[2],D[7],D[3],D[8],D[4],D[9]};
        double* Kp[] = {K[0],K[5],K[1],K[6],K[2],K[7],K[3],K[8],K[4],K[9]};
        double* Rp[] = {R[0],R[5],R[1],R[6],R[2],R[7],R[3],R[8],R[4],R[9]};
        double* Tp[] = {T[0],T[5],T[1],T[6],T[2],T[7],T[3],T[8],T[4],T[9]};
        {
            IOccamStereoRectify* rectify_iface = 0;
            occamGetInterface(rectify_handle.get(),IOCCAMSTEREORECTIFY,(void**)&rectify_iface);
            rectify_iface->configure(rectify_handle.get(),10,sensor_width,sensor_height,Dp,Kp,Rp,Tp,1);
        }

        auto img0_mon0r = rectifyImage(rectify_handle,0,img0_mon0);
        auto img1_mon0r = rectifyImage(rectify_handle,1,img1_mon0);
        auto img0_mon1r = rectifyImage(rectify_handle,2,img0_mon1);
        auto img1_mon1r = rectifyImage(rectify_handle,3,img1_mon1);
        auto img0_mon2r = rectifyImage(rectify_handle,4,img0_mon2);
        auto img1_mon2r = rectifyImage(rectify_handle,5,img1_mon2);
        auto img0_mon3r = rectifyImage(rectify_handle,6,img0_mon3);
        auto img1_mon3r = rectifyImage(rectify_handle,7,img1_mon3);
        auto img0_mon4r = rectifyImage(rectify_handle,8,img0_mon4);
        auto img1_mon4r = rectifyImage(rectify_handle,9,img1_mon4);
        out.set(OCCAM_RECTIFIED_IMAGE0,img0_mon0r);
        out.set(OCCAM_RECTIFIED_IMAGE2,img0_mon1r);
        out.set(OCCAM_RECTIFIED_IMAGE4,img0_mon2r);
        out.set(OCCAM_RECTIFIED_IMAGE6,img0_mon3r);
        out.set(OCCAM_RECTIFIED_IMAGE8,img0_mon4r);
        out.set(OCCAM_RECTIFIED_IMAGE1,img1_mon0r);
        out.set(OCCAM_RECTIFIED_IMAGE3,img1_mon1r);
        out.set(OCCAM_RECTIFIED_IMAGE5,img1_mon2r);
        out.set(OCCAM_RECTIFIED_IMAGE7,img1_mon3r);
        out.set(OCCAM_RECTIFIED_IMAGE9,img1_mon4r);
        

        std::shared_ptr<void> stereo_handle = module(OCCAM_STEREO_MATCHER0);
        auto disp0 = computeDisparityImage(stereo_handle,0,img0_mon0r,img1_mon0r);
        auto disp1 = computeDisparityImage(stereo_handle,1,img0_mon1r,img1_mon1r);
        auto disp2 = computeDisparityImage(stereo_handle,2,img0_mon2r,img1_mon2r);
        auto disp3 = computeDisparityImage(stereo_handle,3,img0_mon3r,img1_mon3r);
        auto disp4 = computeDisparityImage(stereo_handle,4,img0_mon4r,img1_mon4r);

        auto disp0r = unrectifyImage(rectify_handle,0,disp0);
        auto disp1r = unrectifyImage(rectify_handle,2,disp1);
        auto disp2r = unrectifyImage(rectify_handle,4,disp2);
        auto disp3r = unrectifyImage(rectify_handle,6,disp3);
        auto disp4r = unrectifyImage(rectify_handle,8,disp4);

        out.set(OCCAM_DISPARITY_IMAGE0,disp0r);
        out.set(OCCAM_DISPARITY_IMAGE1,disp1r);
        out.set(OCCAM_DISPARITY_IMAGE2,disp2r);
        out.set(OCCAM_DISPARITY_IMAGE3,disp3r);
        out.set(OCCAM_DISPARITY_IMAGE4,disp4r);
        out.set(OCCAM_TILED_DISPARITY_IMAGE,htile({disp0r,disp1r,disp2r,disp3r,disp4r}));

        auto img0_pro0r = rectifyImage(rectify_handle,0,img0_pro0);
        auto img0_pro1r = rectifyImage(rectify_handle,2,img0_pro1);
        auto img0_pro2r = rectifyImage(rectify_handle,4,img0_pro2);
        auto img0_pro3r = rectifyImage(rectify_handle,6,img0_pro3);
        auto img0_pro4r = rectifyImage(rectify_handle,8,img0_pro4);

        out.set(OCCAM_POINT_CLOUD0,computePointCloud(rectify_handle,0,img0_pro0r,disp0));
        out.set(OCCAM_POINT_CLOUD1,computePointCloud(rectify_handle,2,img0_pro1r,disp1));
        out.set(OCCAM_POINT_CLOUD2,computePointCloud(rectify_handle,4,img0_pro2r,disp2));
        out.set(OCCAM_POINT_CLOUD3,computePointCloud(rectify_handle,6,img0_pro3r,disp3));
        out.set(OCCAM_POINT_CLOUD4,computePointCloud(rectify_handle,8,img0_pro4r,disp4));

        {
            auto htile0 = htile({img0_mon0,img0_mon1,img0_mon2,img0_mon3,img0_mon4});
            auto rgb_htile0 = makeRGBImage(htile0);
            auto htile1 = htile({disp0r,disp1r,disp2r,disp3r,disp4r});
            auto heatmap_htile1 = heatmapImage(htile1);
            out.set(OCCAM_IMAGE_TILES1,vtile({rgb_htile0,heatmap_htile1}));
        }

        auto dispr_blend =
            heatmapImage(blendImages(blend_handle,{disp0r,disp1r,disp2r,disp3r,disp4r}));
        out.set(OCCAM_STITCHED_DISPARITY_IMAGE,dispr_blend);

        out.set(OCCAM_STITCHED_IMAGE1,vtile({makeRGBImage(img0_blend),dispr_blend}));

        return OCCAM_API_SUCCESS;
    }

    virtual void availableData(std::vector<std::pair<OccamDataName,OccamDataType> >& available_data) {
        available_data.push_back(std::make_pair(OCCAM_IMAGE_TILES0,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE_TILES0,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE_TILES1,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_TILED_DISPARITY_IMAGE,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_STITCHED_IMAGE0,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_STITCHED_IMAGE1,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_STITCHED_DISPARITY_IMAGE,OCCAM_IMAGE));

        available_data.push_back(std::make_pair(OCCAM_IMAGE0,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE1,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE2,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE3,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE4,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE5,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE6,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE7,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE8,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_IMAGE9,OCCAM_IMAGE));

        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE0,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE1,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE2,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE3,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE4,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE5,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE6,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE7,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE8,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE9,OCCAM_IMAGE));

        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE0,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE1,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE2,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE3,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE4,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE5,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE6,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE7,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE8,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_RECTIFIED_IMAGE9,OCCAM_IMAGE));

        available_data.push_back(std::make_pair(OCCAM_DISPARITY_IMAGE0,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_DISPARITY_IMAGE1,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_DISPARITY_IMAGE2,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_DISPARITY_IMAGE3,OCCAM_IMAGE));
        available_data.push_back(std::make_pair(OCCAM_DISPARITY_IMAGE4,OCCAM_IMAGE));

        available_data.push_back(std::make_pair(OCCAM_POINT_CLOUD0,OCCAM_POINT_CLOUD));
        available_data.push_back(std::make_pair(OCCAM_POINT_CLOUD1,OCCAM_POINT_CLOUD));
        available_data.push_back(std::make_pair(OCCAM_POINT_CLOUD2,OCCAM_POINT_CLOUD));
        available_data.push_back(std::make_pair(OCCAM_POINT_CLOUD3,OCCAM_POINT_CLOUD));
        available_data.push_back(std::make_pair(OCCAM_POINT_CLOUD4,OCCAM_POINT_CLOUD));
    }

};

void init_omnis5u3mt9v022() {
    registerDevice<OccamDevice_omnis5u3mt9v022>("omnis5u3mt9v022", 2);
}

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}