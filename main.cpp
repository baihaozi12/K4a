#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <k4a/k4a.h>
#include <math.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>
//#include <k4ainternal/common.h>
#include <opencv2/rgbd/kinfu.hpp>
using namespace cv;
using namespace cv::kinfu;
//
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

template<typename T> Mat create_mat_from_buffer(T *data, int width, int height, int channels )
{
    cv::Mat mat(height, width, CV_MAKETYPE(DataType<T>::type, channels));
    memcpy(mat.data, data, width * height * channels * sizeof(T));
    return mat;
}

// Compute a conservative bounding box on the unit plane in which all the points have valid projections
void compute_xy_range(const k4a_calibration_t* calibration,
                      const k4a_calibration_type_t camera,
                      const int width,
                      const int height,
                      float& x_min,
                      float& x_max,
                      float& y_min,
                      float& y_max)
{
    // Step outward from the centre point until we find the bounds of valid projection
    const float step_u = 0.25f;
    const float step_v = 0.25f;
    const float min_u = 0;
    const float min_v = 0;
    const float max_u = (float)width - 1;
    const float max_v = (float)height - 1;
    const float center_u = 0.5f * width;
    const float center_v = 0.5f * height;

    int valid;
    k4a_float2_t p;
    k4a_float3_t ray;

    // search x_min
    for (float uv[2] = { center_u, center_v }; uv[0] >= min_u; uv[0] -= step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_min = ray.xyz.x;
    }

    // search x_max
    for (float uv[2] = { center_u, center_v }; uv[0] <= max_u; uv[0] += step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_max = ray.xyz.x;
    }

    // search y_min
    for (float uv[2] = { center_u, center_v }; uv[1] >= min_v; uv[1] -= step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_min = ray.xyz.y;
    }

    // search y_max
    for (float uv[2] = { center_u, center_v }; uv[1] <= max_v; uv[1] += step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_max = ray.xyz.y;
    }
}
pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t* calibration, const k4a_calibration_type_t camera)
{
    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        width = calibration->color_camera_calibration.resolution_width;
        height = calibration->color_camera_calibration.resolution_height;
    }

    float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    compute_xy_range(calibration, camera, width, height, x_min, x_max, y_min, y_max);

    pinhole_t pinhole;

    float fx = 1.f / (x_max - x_min);
    float fy = 1.f / (y_max - y_min);
    float px = -x_min * fx;
    float py = -y_min * fy;

    pinhole.fx = fx * width;
    pinhole.fy = fy * height;
    pinhole.px = px * width;
    pinhole.py = py * height;
    pinhole.width = width;
    pinhole.height = height;

    return pinhole;
}

void create_undistortion_lut(const k4a_calibration_t* calibration,
                             const k4a_calibration_type_t camera,
                             const pinhole_t* pinhole,
                             k4a_image_t lut,
                             interpolation_t type)
{
    coordinate_t* lut_data = (coordinate_t*)(void*)k4a_image_get_buffer(lut);

    k4a_float3_t ray;
    ray.xyz.z = 1.f;

    int src_width = calibration->depth_camera_calibration.resolution_width;
    int src_height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        src_width = calibration->color_camera_calibration.resolution_width;
        src_height = calibration->color_camera_calibration.resolution_height;
    }

    for (int y = 0, idx = 0; y < pinhole->height; y++)
    {
        ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

        for (int x = 0; x < pinhole->width; x++, idx++)
        {
            ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

            k4a_float2_t distorted;
            int valid;
            k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

            coordinate_t src;
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                // Remapping via nearest neighbor interpolation
                src.x = (int)floorf(distorted.xy.x + 0.5f);
                src.y = (int)floorf(distorted.xy.y + 0.5f);
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                // Remapping via bilinear interpolation
                src.x = (int)floorf(distorted.xy.x);
                src.y = (int)floorf(distorted.xy.y);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }

            if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
            {
                lut_data[idx] = src;

                if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // Compute the floating point weights, using the distance from projected point src to the
                    // image coordinate of the upper left neighbor
                    float w_x = distorted.xy.x - src.x;
                    float w_y = distorted.xy.y - src.y;
                    float w0 = (1.f - w_x) * (1.f - w_y);
                    float w1 = w_x * (1.f - w_y);
                    float w2 = (1.f - w_x) * w_y;
                    float w3 = w_x * w_y;

                    // Fill into lut
                    lut_data[idx].weight[0] = w0;
                    lut_data[idx].weight[1] = w1;
                    lut_data[idx].weight[2] = w2;
                    lut_data[idx].weight[3] = w3;
                }
            }
            else
            {
                lut_data[idx].x = INVALID;
                lut_data[idx].y = INVALID;
            }
        }
    }
}

void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t* src_data = (uint16_t*)(void*)k4a_image_get_buffer(src);
    uint16_t* dst_data = (uint16_t*)(void*)k4a_image_get_buffer(dst);
    coordinate_t* lut_data = (coordinate_t*)(void*)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint16_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                // introduce noise on the edge. If the image is color or ir images, user should use
                // INTERPOLATION_BILINEAR
                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = min(min(neighbors[0], neighbors[1]), min(neighbors[2], neighbors[3]));
                    float depth_max = max(max(neighbors[0], neighbors[1]), max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint16_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                                         neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                                         0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}

void PrintUsage()
{
    printf("Usage: kinfu_example.exe [Optional]<Mode>\n");
    printf("    Mode: nfov_unbinned(default), wfov_2x2binned, wfov_unbinned, nfov_2x2binned\n");
    printf("    Keys:   q - Quit\n");
    printf("            r - Reset KinFu\n");
    printf("            v - Enable Viz Render Cloud (default is OFF, enable it will slow down frame rate)\n");
    printf("            w - Write out the kf_output.ply point cloud file in the running folder\n");
    printf("    * Please ensure to uncomment HAVE_OPENCV pound define to enable the opencv code that runs kinfu\n");
    printf("    * Please ensure to copy opencv/opencv_contrib/vtk dlls to the running folder\n\n");
}
void initialize_kinfu_params(cv::kinfu::Params &params,
                             const int width,
                             const int height,
                             const float fx,
                             const float fy,
                             const float cx,
                             const float cy)
{
    const Matx33f camera_matrix = Matx33f(fx, 0.0f, cx, 0.0f, fy, cy, 0.0f, 0.0f, 1.0f);
    params.frameSize = Size(width, height);
    params.intr = camera_matrix;
    params.depthFactor = 1000.0f;
}


int _stricmp(const char *a, const char *b) {
    int ca, cb;
    do {
        ca = (unsigned char) *a++;
        cb = (unsigned char) *b++;
        ca = tolower(toupper(ca));
        cb = tolower(toupper(cb));
    } while (ca == cb && ca != '\0');
    return ca - cb;
}


Mat decof(Mat image,double k1, double k2,double k3,double k4,double k5,
          double k6,double p1,double p2,double cx,double cy){
    // 畸变参数
//    double k1 = 0.197412, k2 = -2.37519, p1 = 0.000679696, p2 = -0.000651019;
//// 内参
    double fx = 605.465, fy = 605.431;
    int rows = image.rows, cols = image.cols;
    cv::Mat    image_undistort = cv::Mat(rows, cols, CV_8UC4);   // 去畸变以后的图

// 计算去畸变后图像的内容
    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {

            double u_distorted = 0, v_distorted = 0;
            // TODO 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted, v_distorted) (~6 lines)
            // start your code here
            //image_undistort中含有非畸变的图像坐标
            //将image_undistort的坐标通过内参转换到归一化坐标系下，此时得到的归一化坐标是对的
            //将得到的归一化坐标系进行畸变处理
            //将畸变处理后的坐标通过内参转换为图像坐标系下的坐标
            //这样就相当于是在非畸变图像的图像坐标和畸变图像的图像坐标之间建立了一个对应关系
            //相当于是非畸变图像坐标在畸变图像中找到了映射
            //对畸变图像进行遍历之后，然后赋值（一般需要线性插值，因为畸变后图像的坐标不一定是整数的），即可得到矫正之后的图像
            double x1,y1,x2,y2;
            x1 = (u-cx)/fx;
            y1 = (v-cy)/fy;
            double r2;
            r2 = pow(x1,2)+pow(y1,2);
            x2  = x1*(1+k1*r2+k2*pow(r2,2))+2*p1*x1*y1+p2*(r2+2*x1*x1);
            y2 = y1*(1+k1*r2+k2*pow(r2,2))+p1*(r2+2*y1*y1)+2*p2*x1*y1;

            u_distorted = fx*x2+cx;
            v_distorted = fy*y2+cy;

            // end your code here

            // 赋值 (最近邻插值)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }

// 画图去畸变后图像
//    cv::imshow("image undistorted", image_undistort);
//    cv::waitKey();

    return image_undistort;
}

int main(int argc, char * argv[]) {
    k4a_device_t device = NULL;
    // Configure the depth mode and fps
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
//    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.synchronized_images_only = true;
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 1;
    }


    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
        k4a_device_close(device);
        return 1;
    }

    // Retrive calibration
    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        k4a_device_close(device);
        return 1;
    }

    // Start cameras
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start device\n");
        k4a_device_close(device);
        return 1;
    }
    k4a_transformation_t transformation = k4a_transformation_create(&calibration);
    // Generate a pinhole model for depth camera
    pinhole_t pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);
    interpolation_t interpolation_type = INTERPOLATION_BILINEAR_DEPTH;

    setUseOptimized(true);
// Retrieve calibration parameters
    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.depth_camera_calibration.intrinsics.parameters;
    const int width = calibration.depth_camera_calibration.resolution_width;
    const int height = calibration.depth_camera_calibration.resolution_height;
    float k1 = intrinsics->param.k1;
    float k2 = intrinsics->param.k2;
    float k3 = intrinsics->param.k3;
    float k4 = intrinsics->param.k4;
    float k5 = intrinsics->param.k5;
    float k6 = intrinsics->param.k6;

    float cx = intrinsics->param.cx;
    float cy = intrinsics->param.cy;

    float p1 = intrinsics->param.p1;
    float p2 = intrinsics->param.p2;
    float codx = intrinsics->param.codx;
    float cody = intrinsics->param.cody;
    // Initialize kinfu parameters
    Ptr<kinfu::Params> params;
    params = kinfu::Params::defaultParams();
    bool useHashTSDF = false;
    bool coarse = false;
    if(coarse){
        params = Params::coarseParams();
    }else{
        params = Params::defaultParams();
    }
    if(useHashTSDF){

        params = Params::hashTSDFParams(coarse);
    }
//    Ptr<DepthWriter> depthWriter;
//    std::string recordPath = "/home/benebot/baihao/recordPath";
//    if(!recordPath.empty())
//        depthWriter = makePtr<DepthWriter>(recordPath);
//#ifdef HAVE_OPENCV_VIZ
//    cout<<"we have we have we have"<<endl;
//    const std::string vizWindowName = "cloud";
//    cv::viz::Viz3d window(vizWindowName);
//    window.setViewerPose(Affine3f::Identity());
//    bool pause = false;
//#endif

    initialize_kinfu_params(*params, width, height, pinhole.fx, pinhole.fy, pinhole.px, pinhole.py);
// Distortion coefficients
    Matx<float, 1, 8> distCoeffs;
    distCoeffs(0) = intrinsics->param.k1;
    distCoeffs(1) = intrinsics->param.k2;
    distCoeffs(2) = intrinsics->param.p1;
    distCoeffs(3) = intrinsics->param.p2;
    distCoeffs(4) = intrinsics->param.k3;
    distCoeffs(5) = intrinsics->param.k4;
    distCoeffs(6) = intrinsics->param.k5;
    distCoeffs(7) = intrinsics->param.k6;

    k4a_image_t lut = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     pinhole.width,
                     pinhole.height,
                     pinhole.width * (int)sizeof(coordinate_t),
                     &lut);

    create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_DEPTH, &pinhole, lut, interpolation_type);

    // Create KinectFusion module instance
    Ptr<kinfu::KinFu> kf;
    kf = kinfu::KinFu::create(params);
//    namedWindow("AzureKinect KinectFusion Example");
//    viz::Viz3d visualization("AzureKinect KinectFusion Example");

    bool stop = false;
    bool renderViz = false;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t undistorted_depth_image = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    int count = 0;
    while (!stop ) {
        // Get a depth frame
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                printf("Timed out waiting for a capture\n");
                continue;
                break;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a capture\n");
                k4a_device_close(device);
                return 1;
        }
        // Retrieve depth image
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == NULL)
        {
            printf("Depth16 None\n");
            k4a_capture_release(capture);
            continue;
        }
        k4a_image_t rgbImage = k4a_capture_get_color_image(capture);

        // Retrieve depth image
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == NULL)
        {
            printf("Depth16 None\n");
            k4a_capture_release(capture);
            continue;
        }
        k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                         pinhole.width,
                         pinhole.height,
                         pinhole.width * (int)sizeof(uint16_t),
                         &undistorted_depth_image);
        remap(depth_image, lut, undistorted_depth_image, interpolation_type);

        // Create frame from depth buffer
        uint8_t *buffer = k4a_image_get_buffer(undistorted_depth_image);
        uint16_t *depth_buffer = reinterpret_cast<uint16_t *>(buffer);
        UMat undistortedFrame;
//        create_mat_from_buffer<uint16_t>(depth_buffer, width, height, 1).copyTo(undistortedFrame);
        Mat cv_depth = cv::Mat(k4a_image_get_height_pixels(depth_image),k4a_image_get_width_pixels(depth_image),CV_16U,k4a_image_get_buffer(depth_image), static_cast<size_t>(k4a_image_get_stride_bytes(depth_image)));
        std::string name = "/home/haobai/Desktop/capturedimage/rgb_image.png";
//        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
        Mat cv_rgb = cv::Mat(k4a_image_get_height_pixels(rgbImage),k4a_image_get_width_pixels(rgbImage),CV_8UC4,k4a_image_get_buffer(rgbImage), static_cast<size_t>(k4a_image_get_stride_bytes(rgbImage)));
//        cv_rgb = decof(cv_rgb, k1,k2,k3,k4,k5,k6,p1,p2,codx,cody);
        cv::imwrite(name, cv_rgb);
        cv_depth.copyTo(undistortedFrame);
        if (undistortedFrame.empty())
        {
            k4a_image_release(depth_image);
            k4a_image_release(undistorted_depth_image);
            k4a_capture_release(capture);
            continue;
        }
        // Update KinectFusion
//        kf->update(undistortedFrame);
        if (!kf->update(undistortedFrame))        {
            printf("failed KinectFusion \n");
            kf->reset();
            k4a_image_release(depth_image);
            k4a_image_release(undistorted_depth_image);
            k4a_capture_release(capture);
            continue;
        }else{
            printf("kinect update succeed\n");
        }
//         Retrieve rendered TSDF
//        UMat tsdfRender;
//        kf->render(tsdfRender);
//        name = "/home/benebot/baihao/tsdfRender.jpg";
////        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
//        cv::imwrite(name, tsdfRender);
        UMat points;
        UMat normals;

        kf->getCloud(points, normals);

//        int width = k4a_image_get_width_pixels(points);
//        int height = k4a_image_get_height_pixels(points);

//        cout<<width<< endl;

        if (!points.empty() && !normals.empty())
        {
            printf("not empty point cloud\n");
            viz::WCloud cloud(points, viz::Color::white());
            viz::WCloudNormals cloudNormals(points, normals, 1, 0.01, viz::Color::cyan());
//            visualization.showWidget("cloud", cloud);
//            visualization.showWidget("normals", cloudNormals);
//            visualization.showWidget("worldAxes", viz::WCoordinateSystem());
            //            Vec3d volSize = kf->getParams().voxelSize * kf->getParams().volumeDims;
            Vec3d volSize = kf->getParams().voxelSize * kf->getParams().volumeDims;


            int32_t key = waitKey(20);

            if (key == 'r')
            {
//                printf("Reset KinectFusion\n");
//                kf->reset();
            }
            else if (key == 'v')
            {
                renderViz = true;
            }
            else if (key == 'w' or count == 1) {

                printf("Saving fused point cloud into ply file ...\n");
                Mat out_points;
                Mat out_normals;
                points.copyTo(out_points);
                normals.copyTo(out_normals);
#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"
                std::string output_file_name = "/home/haobai/Desktop/capturedimage/kf_output.ply";
                std::ofstream ofs(output_file_name); // text mode first
                ofs << PLY_START_HEADER << std::endl;
                ofs << PLY_ASCII << std::endl;
                ofs << PLY_ELEMENT_VERTEX << " " << out_points.rows << std::endl;
                ofs << "property float x" << std::endl;
                ofs << "property float y" << std::endl;
                ofs << "property float z" << std::endl;
                ofs << "property float nx" << std::endl;
                ofs << "property float ny" << std::endl;
                ofs << "property float nz" << std::endl;

                //            // add by my self
                //            ofs << "property uchar red" << endl;
                //            ofs << "property uchar green" << endl;
                //            ofs << "property float blue" << endl;

                ofs << PLY_END_HEADER << std::endl;
                ofs.close();

                std::stringstream ss;
                //            cout<< " rgb "<< out_points.at<float>(0, 1)<<endl;
                for (int i = 0; i < out_points.rows; ++i) {
                    //  can use
                    ss << out_points.at<float>(i, 0) << " "
                       << out_points.at<float>(i, 1) << " "
                       << out_points.at<float>(i, 2) << " "
                       << out_normals.at<float>(i, 0) << " "
                       << out_normals.at<float>(i, 1) << " "
                       << out_normals.at<float>(i, 2) << std::endl;

                    // add by my self
                    //                ss << out_points.at<float>(i, 0) << " "
                    //                   << out_points.at<float>(i, 1) << " "
                    //                   << out_points.at<float>(i, 2) << " "
                    //                   << out_normals.at<float>(i, 0) << " "
                    //                   << out_normals.at<float>(i, 1) << " "
                    //                   << out_normals.at<float>(i, 2) << " "
                    //                   << out_points.at<float>(i, 3) << " "
                    //                   << out_points.at<float>(i, 4) << " "
                    //                   << out_points.at<float>(i, 5) <<endl;
                }


                std::ofstream ofs_text(output_file_name, std::ios::out | std::ios::app);
                ofs_text.write(ss.str().c_str(), (std::streamsize) ss.str().length());

                stop = true;
                break;
            }
            else if (key == 'q')
            {
                stop = true;
//                break;
            }
//            else
//            {
//                printf("Reset KinectFusion\n");
//                kf->reset();
//                break;
//            }
        }
        k4a_image_release(depth_image);
        k4a_image_release(undistorted_depth_image);
        k4a_capture_release(capture);
        count++;
//        break;
    }

    return 0;
}