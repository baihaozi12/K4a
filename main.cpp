////
//// Created by Zero on 2020/8/17.
////
//
#include "lenz_camera.h"
#include <iostream>
#include <fstream>
#include "io_utils.hpp"
//#define STB_IMAGE_WRITE_IMPLEMENTATION
//
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <k4a/k4a.h>
#include <k4ainternal/common.h>

#include "generate_ply.h"
//
////#include "generate_ply.h"
////LenzCamera camera = LenzCamera();
//
//
using  namespace  std;
using namespace cv::kinfu;
using namespace cv::io_utils;
///*
// multi device:
//
//    for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
//    {
//        if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &device))
//        {
//            printf("%d: Failed to open device\n", deviceIndex);
//            continue;
//        }
//
//
//
//        k4a_device_close(device);
//    }
// */
//
//LenzCamera::LenzCamera() {
//    m_deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//    unsigned int deviceCount = k4a_device_get_installed_count();
//    if (deviceCount == 0)
//    {
//        std::cout << "[Streaming Service] No K4A devices found" << std::endl;
//        return;
//    }
//
//    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device))
//    {
//        std::cout << "[Streaming Service] Failed to open device" << std::endl;
//        return;
//    }
//
//    m_deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; //K4A_IMAGE_FORMAT_COLOR_MJPG  K4A_IMAGE_FORMAT_COLOR_BGRA32
////    m_deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_3072P; //K4A_COLOR_RESOLUTION_2160P K4A_COLOR_RESOLUTION_3072P
//    m_deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;
//    m_deviceConfig.depth_mode = K4A_DEPTH_MODE_PASSIVE_IR; //K4A_DEPTH_MODE_OFF K4A_DEPTH_MODE_NFOV_UNBINNED K4A_DEPTH_MODE_PASSIVE_IR
//    m_deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30; //K4A_FRAMES_PER_SECOND_30 K4A_FRAMES_PER_SECOND_15
//
////    m_deviceConfig.synchronized_images_only = true;
////
////    if (K4A_RESULT_SUCCEEDED !=  k4a_device_get_calibration(m_device, m_deviceConfig.depth_mode, m_deviceConfig.color_resolution, &calibration))
////    {
////        printf("Failed to get calibration\n");
////        k4a_device_close(m_device);
//////        return 1;
////    }
//    printf("init \n");
//}
//
//LenzCamera::~LenzCamera() {
//    this->stop();
//    if (m_device != NULL)
//    {
//        k4a_device_close(m_device);
//    }
//    printf("camera dealloc\n");
//}
//
//int LenzCamera::start() {
//
//    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_device, &m_deviceConfig))
//    {
//        std::cout << "[Streaming Service] Failed to start cameras" << std::endl;
//        return - 1;
//    }
//
//    //手动对焦 K4A_COLOR_CONTROL_MODE_MANUAL K4A_COLOR_CONTROL_MODE_AUTO
//    if (K4A_RESULT_SUCCEEDED != k4a_device_set_color_control(m_device,
//                                                             K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
//                                                             K4A_COLOR_CONTROL_MODE_AUTO,
//                                                             0))
//    {
//        std::cout << "[Streaming Service] Failed to set auto exposure" << std::endl;
//        return -1;
//    }
//    return 0;
//}
//
//int LenzCamera::stop() {
//    if (m_device != NULL)
//    {
//        k4a_device_stop_cameras(m_device);
//    }
//    return 0;
//}
//
//int LenzCamera::k4a_take(std::string (*before_save)()) {
//    // Wait for the first capture before starting streaming.
//    this->start();
//    if (k4a_device_get_capture(m_device, &m_capture, 60000) != K4A_WAIT_RESULT_SUCCEEDED)
//    {
//        std::cerr << "[Streaming Service] Runtime error: k4a_device_get_capture() failed" << std::endl;
//        return -1;
//    }
//    k4a_capture_release(m_capture);
//
//    uint32_t camera_fps = k4a_convert_fps_to_uint(m_deviceConfig.camera_fps);
//    int32_t timeout_ms = HZ_TO_PERIOD_MS(camera_fps);
//
//    for (auto i = 0; i < 15; ++i) {
//        switch (k4a_device_get_capture(m_device, &m_capture, timeout_ms))
//        {
//            case K4A_WAIT_RESULT_SUCCEEDED:
//                break;
//            case K4A_WAIT_RESULT_TIMEOUT:
//                continue;
//            case K4A_WAIT_RESULT_FAILED:
//                continue;
//        }
//        k4a_device_get_capture(m_device, &m_capture, timeout_ms);
//        k4a_capture_release(m_capture);
//    }
//
//
//    // Get the capture
//    switch (k4a_device_get_capture(m_device, &m_capture, 60000))
//    {
//        case K4A_WAIT_RESULT_SUCCEEDED:
//            break;
//        case K4A_WAIT_RESULT_TIMEOUT:
//            std::cout << "[Streaming Service] Timed out waiting for the capture" << std::endl;
//            return -1;
//        case K4A_WAIT_RESULT_FAILED:
//            std::cout << "[Streaming Service] Failed to get the capture" << std::endl;
//            return -1;
//    }
//
//    k4a_image_t color_image = NULL;
//    k4a_image_t depth_image = NULL; //OK
//    color_image = k4a_capture_get_color_image(m_capture);
//    depth_image = k4a_capture_get_depth_image(m_capture);
//    std::cout << "[depth] " << "\n"
//              << "format: " << k4a_image_get_format(depth_image) << "\n"
//              << "height*width: " << k4a_image_get_height_pixels(depth_image) << ", " << k4a_image_get_width_pixels(depth_image)
//              << std::endl;
//    assert(color_image != NULL); // Because m_deviceConfig.synchronized_images_only == true
//    assert(depth_image != NULL);
//    std::string name = "/home/benebot/baihao/caputre-out.jpg";
//    if (before_save) {
//        name = before_save();
//    }
//    printf("name is %s\n",name.c_str());
//    if (k4a_image_get_format(color_image) == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
//        cv::Mat mat = cv::Mat(k4a_image_get_height_pixels(color_image),k4a_image_get_width_pixels(color_image),CV_8UC4,k4a_image_get_buffer(color_image));
//        cv::flip(mat, mat, 1);
//        cv::transpose(mat, mat);
////        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
//        cv::imwrite(name, mat);
//    }
//
//
////    this->writeToFile(name.c_str(),k4a_image_get_buffer(color_image),k4a_image_get_size(color_image));
//
//    this->filepath = name;
//
//    if (color_image)
//    {
//        k4a_image_release(color_image);
//    }
//
//    this->stop();
//
//    k4a_capture_release(m_capture);
//
//    return 0;
//}
//
//int LenzCamera::writeToFile(const char *fileName, void *buffer, size_t bufferSize) {
//
//    assert(buffer != NULL);
//
//    std::ofstream hFile;
//
//    hFile.open(fileName, std::ios::out | std::ios::trunc | std::ios::binary);
//
//    if (hFile.is_open()) {
//        hFile.write((char *) buffer, static_cast<std::streamsize>(bufferSize));
//
//        hFile.close();
//
//        std::cout << "[Streaming Service] Color frame is stored in " << fileName << std::endl;
//
//    } else {
//        std::cout << "[Streaming Service] open fail in" << std::endl;
//    }
//}
//
//int main_(int argc, char * argv[])
//{
//    LenzCamera camera = LenzCamera();
//
////    LenzCamera camera;
////    camera.start();
//
//    camera.k4a_take(NULL);
//
//    //my own code
////    const u_int32_t device_count = k4a_device_get_installed_count();
////    if (K4A_RESULT_SUCCEEDED !=  k4a_device_open(K4A_DEVICE_DEFAULT)){
////        std::cout<<"open not success"<<std::endl;
////    }
////    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
////    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
////    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
////    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
////    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
////    config.synchronized_images_only = true;
//
////    k4a_device_t m_device;
////    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(m_device, &config))
////    {
////        std::cout << "[Streaming Service] Failed to start cameras" << std::endl;
////        return 1;
////    }
////    std::cout << "device ok tp load";
//    return 0;
//}
//
//
int main_(int argc, char * argv[]) {
    const uint32_t device_count = k4a_device_get_installed_count();
    cout << "device count is : " << device_count << endl;
    if (0 == device_count) {
        std::cout << "Error: no K4A devices found. " << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "Found " << device_count << " connected devices. " << std::endl;

        if (1 != device_count)// 超过1个设备，也输出错误信息。
        {
            std::cout << "Error: more than one K4A devices found. " << std::endl;
            return EXIT_FAILURE;
        } else// 该示例代码仅限对1个设备操作
        {
            std::cout << "Done: found 1 K4A device. " << std::endl;
        }
    }
    // 打开（默认）设备
    k4a_device_t device;
    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device)) {
        std::cout << "[Streaming Service] Failed to open device" << std::endl;
        return 1;
    }
    std::cout << "Done: open device. " << std::endl;

    /*
		检索 Azure Kinect 图像数据
	*/
    // 配置并启动设备
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    // config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture
    std::cout << "Done:config setting. " << std::endl;

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config)) {
        std::cout << "[Streaming Service] Failed to start cameras" << std::endl;
        return -1;
    }
    std::cout << "Done:camera start . " << std::endl;

    // 稳定化
    k4a_capture_t capture;
    int iAuto = 0;//用来稳定，类似自动曝光
    int iAutoError = 0;// 统计自动曝光的失败次数
    if (k4a_device_get_capture(device, &capture, 60000) != K4A_WAIT_RESULT_SUCCEEDED) {
        std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;

    }
//    while (true) {
//        if (k4a_device_get_capture(device, &capture, 60000) != K4A_WAIT_RESULT_SUCCEEDED) {
//            std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;
//
//            // 跳过前 n 个（成功的数据采集）循环，用来稳定
//            if (iAuto != 30) {
//                iAuto++;
//                continue;
//            } else {
//                std::cout << "Done: auto-exposure" << std::endl;
//                break;// 跳出该循环，完成相机的稳定过程
//            }
//        }else
//        {
//            std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
//            if (iAutoError != 30)
//            {
//                iAutoError++;
//                continue;
//            }
//            else
//            {
//                std::cout << "Error: failed to give auto-exposure. " << std::endl;
//                return EXIT_FAILURE;
//            }
//        }
//
//
//    }
    // 从设备获取捕获
    k4a_image_t rgbImage;
    k4a_image_t depthImage;
    k4a_image_t irImage;

    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;
    cv::Mat cv_irImage;
    cv::Mat cv_irImage_8U;

    rgbImage = k4a_capture_get_color_image(capture);
    depthImage = k4a_capture_get_depth_image(capture);
    std::cout << "[depth] " << "\n"
              << "format: " << k4a_image_get_format(depthImage) << "\n"
              << "height*width: " << k4a_image_get_height_pixels(depthImage) << ", " << k4a_image_get_width_pixels(depthImage)
              << std::endl;
    cv_depth = cv::Mat(k4a_image_get_height_pixels(depthImage),k4a_image_get_width_pixels(depthImage),CV_16U,k4a_image_get_buffer(depthImage), static_cast<size_t>(k4a_image_get_stride_bytes(depthImage)));
    cv::flip(cv_depth, cv_depth, 1);

    cv::transpose(cv_depth, cv_depth);
    cv_depth.convertTo(cv_depth_8U, CV_8U, 1 );
//        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
    std::string name = "/home/benebot/baihao/depth_image.jpg";
    cv::imwrite(name, cv_depth_8U);

    // ir
    irImage = k4a_capture_get_ir_image(capture);
    std::cout << "[ir] " << "\n"
              << "format: " << k4a_image_get_format(irImage) << "\n"
              << "height*width: " << k4a_image_get_height_pixels(irImage) << ", " << k4a_image_get_width_pixels(irImage)
              << std::endl;
    cv_irImage = cv::Mat(k4a_image_get_height_pixels(irImage), k4a_image_get_width_pixels(irImage), CV_16U, (void *)k4a_image_get_buffer(depthImage), static_cast<size_t>(k4a_image_get_stride_bytes(depthImage)));
    cv_irImage.convertTo(cv_irImage_8U, CV_8U, 1 );
    name = "/home/benebot/baihao/ir_image.jpg";
    cv::imwrite(name, cv_depth_8U);


    // get point cloud
    k4a_calibration_t calibration;
//    k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration));

    k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration);
//    k4a_transformation_t transformation = k4a_transformation_create(calibration);
    k4a_transformation_t transformation;


//    k4a_transformation_depth_image_to_point_cloud()
    k4a_image_t xyzImage;
    cv::Mat cv_xyzImage;
    cv::Mat cv_xyzImage_32F;


// 点云
/*
	Each pixel of the xyz_image consists of three int16_t values, totaling 6 bytes. The three int16_t values are the X, Y, and Z values of the point.
	我们将为每个像素存储三个带符号的 16 位坐标值（以毫米为单位）。 因此，XYZ 图像步幅设置为 width * 3 * sizeof(int16_t)。
	数据顺序为像素交错式，即，X 坐标 – 像素 0，Y 坐标 – 像素 0，Z 坐标 – 像素 0，X 坐标 – 像素 1，依此类推。
	如果无法将某个像素转换为 3D，该函数将为该像素分配值 [0,0,0]。
*/
//    k4a_transformation_depth_image_to_point_cloud(transformation, depthImage, K4A_CALIBRATION_TYPE_DEPTH, xyzImage);


    rgbImage = k4a_capture_get_color_image(capture);
    cv::Mat mat = cv::Mat(k4a_image_get_height_pixels(rgbImage),k4a_image_get_width_pixels(rgbImage),CV_8UC4,k4a_image_get_buffer(rgbImage));
    cv::flip(mat, mat, 1);
    cv::transpose(mat, mat);
    name = "/home/benebot/baihao/rgb_image.jpg";
//        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
    cv::imwrite(name, mat);

    printf(CV_VERSION);

    // Generate a pinhole model for depth camera
//    pinhole_t pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);
    pinhole_t pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);
    interpolation_t interpolation_type = INTERPOLATION_BILINEAR_DEPTH;
    // Retrieve calibration parameters
    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.depth_camera_calibration.intrinsics.parameters;
    const int width = calibration.depth_camera_calibration.resolution_width;
    const int height = calibration.depth_camera_calibration.resolution_height;

    // Initialize kinfu parameters
    Ptr<kinfu::Params> params;
    params = kinfu::Params::defaultParams();
    initialize_kinfu_params(
            *params, width, height, pinhole.fx, pinhole.fy, pinhole.px, pinhole.py);

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

    k4a_image_t undistorted_depth_image = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                     pinhole.width,
                     pinhole.height,
                     pinhole.width * (int)sizeof(uint16_t),
                     &undistorted_depth_image);
    remap(depthImage, lut, undistorted_depth_image, interpolation_type);

    // Create frame from depth buffer
    uint8_t *buffer = k4a_image_get_buffer(undistorted_depth_image);
    uint16_t *depth_buffer = reinterpret_cast<uint16_t *>(buffer);
    UMat undistortedFrame;
//    create_mat_from_buffer<uint16_t>(depth_buffer, width, height).copyTo(undistortedFrame);
    cout<<"depth_buffer is : "<<depth_buffer<<endl;
    name = "/home/benebot/baihao/undistorted_depth_image.jpg";
//        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
//    cv::imwrite(name, undistorted_depth_image);
}
int main_OK(int argc, char * argv[]) {
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
#ifdef HAVE_OPENCV_VIZ
    cout<<"we have we have we have"<<endl;
    const std::string vizWindowName = "cloud";
    cv::viz::Viz3d window(vizWindowName);
    window.setViewerPose(Affine3f::Identity());
    bool pause = false;
#endif

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
        std::string name = "/home/benebot/baihao/rgb_image.jpg";
//        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
        Mat cv_rgb = cv::Mat(k4a_image_get_height_pixels(rgbImage),k4a_image_get_width_pixels(rgbImage),CV_32F,k4a_image_get_buffer(rgbImage), static_cast<size_t>(k4a_image_get_stride_bytes(rgbImage)));
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
            else if (key == 'w' or count == 200) {

                printf("Saving fused point cloud into ply file ...\n");
                Mat out_points;
                Mat out_normals;
                points.copyTo(out_points);
                normals.copyTo(out_normals);
#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"
                string output_file_name = "/home/benebot/baihao/kf_output.ply";
                ofstream ofs(output_file_name); // text mode first
                ofs << PLY_START_HEADER << endl;
                ofs << PLY_ASCII << endl;
                ofs << PLY_ELEMENT_VERTEX << " " << out_points.rows << endl;
                ofs << "property float x" << endl;
                ofs << "property float y" << endl;
                ofs << "property float z" << endl;
                ofs << "property float nx" << endl;
                ofs << "property float ny" << endl;
                ofs << "property float nz" << endl;

                //            // add by my self
                //            ofs << "property uchar red" << endl;
                //            ofs << "property uchar green" << endl;
                //            ofs << "property float blue" << endl;

                ofs << PLY_END_HEADER << endl;
                ofs.close();

                stringstream ss;
                //            cout<< " rgb "<< out_points.at<float>(0, 1)<<endl;
                for (int i = 0; i < out_points.rows; ++i) {
                    //  can use
                    ss << out_points.at<float>(i, 0) << " "
                       << out_points.at<float>(i, 1) << " "
                       << out_points.at<float>(i, 2) << " "
                       << out_normals.at<float>(i, 0) << " "
                       << out_normals.at<float>(i, 1) << " "
                       << out_normals.at<float>(i, 2) << endl;

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


                ofstream ofs_text(output_file_name, ios::out | ios::app);
                ofs_text.write(ss.str().c_str(), (streamsize) ss.str().length());


                // another way
                //            int depthimage_width = k4a_image_get_width_pixels(depth_image);
                //            int depthimage_height = k4a_image_get_height_pixels(depth_image);
                //
                //            k4a_image_t transformation_color_image;
                //            cout << k4a_image_get_format(rgbImage) << endl;
                //            k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                //                             depthimage_width,
                //                             depthimage_height,
                //                             depthimage_width * 4 * (int)sizeof(uint8_t), &transformation_color_image);
                //            k4a_transformation_color_image_to_depth_camera(transformation, depth_image, rgbImage, transformation_color_image);
                //
                //            k4a_image_t point_cloud;
                //            k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depthimage_width, depthimage_height, depthimage_width * 3 * (int)sizeof(uint16_t), &point_cloud);
                //            k4a_transformation_depth_image_to_point_cloud(transformation, depth_image, K4A_CALIBRATION_TYPE_DEPTH, point_cloud);
                //
                //
                ////            vector<color_point_t> points;
                //
                //            int width = k4a_image_get_width_pixels(point_cloud);
                //            int height = k4a_image_get_height_pixels(point_cloud);
                //            int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud);
                //            uint8_t *color_image_data = k4a_image_get_buffer(rgbImage);
                //
                //            for (int i = 0; i < width * height; i++)
                //            {
                ////                color_point_t point;
                ////                point.xyz[0] = point_cloud_image_data[3 * i + 0];
                ////                point.xyz[1] = point_cloud_image_data[3 * i + 1];
                ////                point.xyz[2] = point_cloud_image_data[3 * i + 2];
                ////                if (point.xyz[2] == 0 ||point.xyz[2]>maxValue*0.3)
                ////                    continue;
                ////                point.bgr[0] = color_image_data[4 * i + 0];
                ////                point.bgr[1] = color_image_data[4 * i + 1];
                ////                point.bgr[2] = color_image_data[4 * i + 2];
                ////                uint8_t alpha = color_image_data[4 * i + 3];
                ////                if (point.bgr[0] == 0 && point.bgr[1] == 0 && point.bgr[2] == 0 && alpha == 0)
                ////                    continue;
                ////                points.push_back(point);
                //
                //                ss << point_cloud_image_data[3 * i + 0] << " "
                //                   << point_cloud_image_data[3 * i + 1] << " "
                //                   << point_cloud_image_data[3 * i + 2] << " "
                //                   << out_normals.at<float>(i, 0) << " "
                //                   << out_normals.at<float>(i, 1) << " "
                //                   << out_normals.at<float>(i, 2) << " "
                //
                ////                        << color_image_data[4 * i + 0] << " "
                ////                        << color_image_data[4 * i + 1] << " "
                ////                        << color_image_data[4 * i + 2] << " "
                //                   <<endl;
                //            }

//                viz::WCloud cloudWidget(points, viz::Color::white());
//                viz::WCloudNormals cloudNormals(points, normals, /*level*/1, /*scale*/0.05, viz::Color::gray());
                //            Vec3d volSize = kf->getParams().voxelSize*Vec3d(kf->getParams().volumeDims);
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

template<typename T> Mat create_mat_from_buffer(T *data, int width, int height, int channels = 1)
{
    Mat mat(height, width, CV_MAKETYPE(DataType<T>::type, channels));
    memcpy(mat.data, data, width * height * channels * sizeof(T));
    return mat;
}

int main(int argc, char** argv)
{
//    PrintUsage();

    k4a_device_t device = NULL;

    if (argc > 2)
    {
        printf("Please read the Usage\n");
        return 2;
    }

    // Configure the depth mode and fps
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.synchronized_images_only = true;
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 1;
    }
    if (argc == 2)
    {
        if (!_stricmp(argv[1], "nfov_unbinned"))
        {
            config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (!_stricmp(argv[1], "wfov_2x2binned"))
        {
            config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (!_stricmp(argv[1], "wfov_unbinned"))
        {
            config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
            config.camera_fps = K4A_FRAMES_PER_SECOND_15;
        }
        else if (!_stricmp(argv[1], "nfov_2x2binned"))
        {
            config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
        }
        else if (!_stricmp(argv[1], "/?"))
        {
            return 0;
        }
        else
        {
            printf("Depth mode not supported!\n");
            return 1;
        }
    }

//    uint32_t device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 1;
    }

    printf("set config OK \n");
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
    printf("here here here \n");
    // Generate a pinhole model for depth camera
    pinhole_t pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_DEPTH);
    interpolation_t interpolation_type = INTERPOLATION_BILINEAR_DEPTH;

//#ifdef HAVE_OPENCV
    setUseOptimized(true);

    // Retrieve calibration parameters
    k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.depth_camera_calibration.intrinsics.parameters;
    const int width = calibration.depth_camera_calibration.resolution_width;
    const int height = calibration.depth_camera_calibration.resolution_height;

    // Initialize kinfu parameters
    Ptr<kinfu::Params> params;
    params = kinfu::Params::defaultParams();
    initialize_kinfu_params(
            *params, width, height, pinhole.fx, pinhole.fy, pinhole.px, pinhole.py);

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
    namedWindow("AzureKinect KinectFusion Example");
    viz::Viz3d visualization("AzureKinect KinectFusion Example");

    bool stop = false;
    bool renderViz = false;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t undistorted_depth_image = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    while (!stop && !visualization.wasStopped())
    {
        // Get a depth frame
        stop = true;
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
        create_mat_from_buffer<uint16_t>(depth_buffer, width, height,1).copyTo(undistortedFrame);

        if (undistortedFrame.empty())
        {
            k4a_image_release(depth_image);
            k4a_image_release(undistorted_depth_image);
            k4a_capture_release(capture);
            continue;
        }

        // Update KinectFusion
        if (!kf->update(undistortedFrame))
        {
            printf("Reset KinectFusion\n");
            kf->reset();

            k4a_image_release(depth_image);
            k4a_image_release(undistorted_depth_image);
            k4a_capture_release(capture);
            continue;
        }els90000000=
                         {PU:IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII}}}}}}}}}}}}}}}}}
            printf("update OK\n");
        }

        std::string name = "/home/benebot/baihao/depth_image/depth_image_2.jpg";
//        memcpy(k4a_image_get_buffer(color_image),&mat.ptr<cv::Vec4b>(0)[0], mat.rows*mat.cols  *sizeof(cv::Vec4b));
//        Mat cv_rgb = cv::Mat(k4a_image_get_height_pixels(rgbImage),k4a_image_get_width_pixels(rgbImage),CV_32F,k4a_image_get_buffer(rgbImage), static_cast<size_t>(k4a_image_get_stride_bytes(rgbImage)));
        cv::imwrite(name, undistortedFrame);

//        if (!kf->update(undistortedFrame))
//        {
//            printf("second time update failed\n");
//            kf->reset();
//            k4a_image_release(depth_image);
//            k4a_image_release(undistorted_depth_image);
//            k4a_capture_release(capture);
//            continue;
//        }else{
//            printf("second time update OK\n");
//        }
//        printf("##############################\n");
        // Retrieve rendered TSDF
        UMat tsdfRender;
        kf->render(tsdfRender);

        // Retrieve fused point cloud and normals
        UMat points;
        UMat normals;
        kf->getCloud(points, normals);

        // Show TSDF rendering
        imshow("AzureKinect KinectFusion Example", tsdfRender);

        // Show fused point cloud and normals
        if (!points.empty() && !normals.empty() && renderViz)
        {
            viz::WCloud cloud(points, viz::Color::white());
            viz::WCloudNormals cloudNormals(points, normals, 1, 0.01, viz::Color::cyan());
            visualization.showWidget("cloud", cloud);
            visualization.showWidget("normals", cloudNormals);
            visualization.showWidget("worldAxes", viz::WCoordinateSystem());
            Vec3d volSize = kf->getParams().voxelSize * kf->getParams().volumeDims;
            visualization.showWidget("cube", viz::WCube(Vec3d::all(0), volSize), kf->getParams().volumePose);
            visualization.spinOnce(1, true);
        }

        // Key controls
        const int32_t key = waitKey(5);
        if (key == 'r')
        {
            printf("Reset KinectFusion\n");
            kf->reset();
        }
        else if (key == 'v')
        {
            renderViz = true;
        }
        else if (key == 'w')
        {
            // Output the fuse-d point cloud from KinectFusion
            Mat out_points;
            Mat out_normals;
            points.copyTo(out_points);
            normals.copyTo(out_normals);

            printf("Saving fused point cloud into ply file ...\n");

            // Save to the ply file
#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"
            string output_file_name = "kf_output.ply";
            ofstream ofs(output_file_name); // text mode first
            ofs << PLY_START_HEADER << endl;
            ofs << PLY_ASCII << endl;
            ofs << PLY_ELEMENT_VERTEX << " " << out_points.rows << endl;
            ofs << "property float x" << endl;
            ofs << "property float y" << endl;
            ofs << "property float z" << endl;
            ofs << "property float nx" << endl;
            ofs << "property float ny" << endl;
            ofs << "property float nz" << endl;
            ofs << PLY_END_HEADER << endl;
            ofs.close();

            stringstream ss;
            for (int i = 0; i < out_points.rows; ++i)
            {
                ss << out_points.at<float>(i, 0) << " "
                   << out_points.at<float>(i, 1) << " "
                   << out_points.at<float>(i, 2) << " "
                   << out_normals.at<float>(i, 0) << " "
                   << out_normals.at<float>(i, 1) << " "
                   << out_normals.at<float>(i, 2) << endl;
            }
            ofstream ofs_text(output_file_name, ios::out | ios::app);
            ofs_text.write(ss.str().c_str(), (streamsize)ss.str().length());
        }
        else if (key == 'q')
        {
            stop = true;
        }

        k4a_image_release(depth_image);
        k4a_image_release(undistorted_depth_image);
        k4a_capture_release(capture);
    }

    k4a_image_release(lut);

    destroyAllWindows();
//#endif

    k4a_device_close(device);

    return 0;
}