//
// Created by Zero on 2020/8/17.
//

#ifndef REALSENSEEXAMPLESSAVETODISK_LENZ_CAMERA_H
#define REALSENSEEXAMPLESSAVETODISK_LENZ_CAMERA_H
#include <vector>

//#define LENZ_IS_MAC 1

#ifdef LENZ_IS_MAC
#else
#include <k4a/k4a.h>
#endif
#include <math.h>
#include <string.h>
#include "assert.h"

#include <iostream>
#include <fstream>

#include <fcntl.h> /* For O_* constants */
#include <sys/stat.h>
#include <semaphore.h>
#include <unistd.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/rgbd/kinfu.hpp>

class LenzCamera {

public:
    LenzCamera();
    ~LenzCamera();

private:
#ifdef LENZ_IS_MAC
#else
    k4a_device_t m_device;
    k4a_capture_t m_capture;
    k4a_device_configuration_t m_deviceConfig;
    k4a_calibration_t calibration;
#endif
public:
    std::string filepath; //call this after take
    int take(std::string (*before_save)(void));

    int start();
    int stop();


    int writeToFile(const  char *name,void *buffer,size_t bufferSize);
    int k4a_take(std::string (*before_save)(void));

};


#endif //REALSENSEEXAMPLESSAVETODISK_LENZ_CAMERA_H
/*
 ln -s /usr/local/bin/cmake /usr/bin/cmake
/bin/sh: 1: ln -s /usr/local/bin/cmake /usr/bin/cmake: not found
 ln -s /usr/local/bin/ctest /usr/bin/ctest
/bin/sh: 1: ln -s /usr/local/bin/ctest: not found
 ln -s /usr/local/bin/cpack /usr/bin/cpack
/bin/sh: 1: ln -s /usr/local/bin/cpack: not found

 */