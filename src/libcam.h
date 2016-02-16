/*
 Copyright (C) 2015 Stefan Kohlbrecher, TU Darmstadt

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * Copyright (C) 2009 Giacomo Spigler
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */

#ifndef __LIBCAM__H__
#define __LIBCAM__H__

#define USE_OPENCV 1

#ifdef USE_OPENCV
#include <cv.h>
//#include <opencv2/opencv.hpp>
#endif

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#include <boost/shared_ptr.hpp>



struct buffer {
        void *                  start;
        size_t                  length;
};

typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR
} io_method;





class Camera {
private:
  void Open();
  void Close();

  void Init();
  void UnInit();

  void Start();
  void Stop();

  void init_userp(unsigned int buffer_size);
  void init_mmap();
  void init_read(unsigned int buffer_size);

  bool initialised;


public:

  const char *name;  //dev_name
  int width;
  int height;
  int fps;

  int w2;

  //unsigned char *data;

  io_method io;
  int fd;
  buffer *buffers;
  int n_buffers;

  struct v4l2_buffer buf;


  int mb, Mb, db, mc, Mc, dc, ms, Ms, ds, mh, Mh, dh, msh, Msh, dsh;
  bool ha;

  Camera(const char *name, int w, int h, int fps=30);
  ~Camera();

  bool Get(bool retrieve_image);    //deprecated
  bool Update(bool retrieve_img = true, unsigned int t=100, int timeout_ms=500); //better  (t=0.1ms, in usecs)
  bool Update(Camera *c2, unsigned int t=100, int timeout_ms=500);

  bool freeBuffer();

#ifdef USE_OPENCV
  void toMonoMat(cv::Mat *im);
  void toMonoMat(cv::Mat *l, int offset_x, int width, int height);
  void toColorMat(cv::Mat *l, int offset_x, int width, int height);
#endif


  void StopCam();

  int minBrightness();
  int maxBrightness();
  int defaultBrightness();
  int minContrast();
  int maxContrast();
  int defaultContrast();
  int minSaturation();
  int maxSaturation();
  int defaultSaturation();
  int minHue();
  int maxHue();
  int defaultHue();
  bool isHueAuto();
  int minSharpness();
  int maxSharpness();
  int defaultSharpness();

  int setBrightness(int v);
  int setContrast(int v);
  int setSaturation(int v);
  int setHue(int v);
  int setHueAuto(bool v);
  int setSharpness(int v);



};





#endif
