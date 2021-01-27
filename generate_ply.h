//
// Created by Hao Bai on 2020/12/18.
//


#include <iostream>
#include <fstream>
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <k4a/k4a.h>
#include <opencv2/viz.hpp>
#include <k4ainternal/common.h>
#include <opencv2/rgbd/kinfu.hpp>


#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <math.h>
using namespace cv::kinfu;
using namespace cv;
using namespace std;
void initialize_kinfu_params(cv::kinfu::Params &params,
                             const int width,
                             const int height,
                             const float fx,
                             const float fy,
                             const float cx,
                             const float cy);

//template<typename T> Mat create_mat_from_buffer(T *data, int width, int height, int channels);


#define INVALID INT32_MIN
typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
    int x;
    int y;
    float weight[4];
} coordinate_t;

typedef enum
{
    INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
    INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
    INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                                data with value 0 */
} interpolation_t;

void create_undistortion_lut(const k4a_calibration_t* calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t* pinhole,
                                    k4a_image_t lut,
                                    interpolation_t type);
void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type);


void compute_xy_range(const k4a_calibration_t* calibration,
                             const k4a_calibration_type_t camera,
                             const int width,
                             const int height,
                             float& x_min,
                             float& x_max,
                             float& y_min,
                             float& y_max);

pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t* calibration, const k4a_calibration_type_t camera);


int _stricmp(const char *a, const char *b) ;



