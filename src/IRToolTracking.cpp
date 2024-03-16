#include "IRToolTracking.h"
#include <iostream>
#include <algorithm>

IRToolTracking::IRToolTracking() {
    // Constructor: Initialize any required variables or state
}

void IRToolTracking::queryDevices() {
	// devices = ctx.query_devices();
    // deviceNames.clear();
	// if (devices.size() == 0) {
	// 	std::cerr << "No RealSense devices found." << std::endl;
	// 	return;
	// }
	// for (size_t i = 0; i < devices.size(); i++) {
    //     std::stringstream ss;
    //     ss << "[" << i << "] " << devices[i].get_info(RS2_CAMERA_INFO_NAME) << " (" << devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ")";
    //     deviceNames.push_back(ss.str());
	// }

    device_count = k4a_device_get_installed_count();
    deviceNames.clear();
    if (device_count == 0)
    {
        std::cerr << "No K4A devices found" << std::endl;
        return;
    }

    for (size_t i = 0; i < device_count; i++)
    {
        std::stringstream ss;
        ss << "[" << i << "] " << "K4A Device";

        //ss << "[" << i << "] " << devices[i].get_info(RS2_CAMERA_INFO_NAME) << " (" << devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << ")";
        deviceNames.push_back(ss.str());
    }
}

void IRToolTracking::initializeFromFile(const std::string& file) {
    // Get the width and height of the frames from the file
    // rs2::pipeline pipe(ctx);
    // rs2::config cfg;
    // cfg.enable_device_from_file(file);
    // pipe.start(cfg);
    // auto profile = pipe.get_active_profile();
    // auto depth_profile = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    // frame_width = depth_profile.width();
    // frame_height = depth_profile.height();
    // pipe.stop();

    // // Load the configuration from file
    // config.enable_device_from_file(file);
    // intrinsics_found = false;
    // Terminated = false;
    // playFromFile = true;
}

void IRToolTracking::initialize(int index, int width, int height) {

    // if (index < 0 || index >= devices.size()) {
    //     std::cerr << "Invalid device index." << std::endl;
    //     Terminated = true;
    //     return;
    // }

    // dev = devices[index];
    // //dev.hardware_reset();
    // std::string model_name = dev.get_info(RS2_CAMERA_INFO_NAME);
    // if (model_name == "Intel RealSense D415") {
    //     irThreshold = 100;
    //     minSize = 10;
    //     maxSize = 300;
    // }
    // else if (model_name == "Intel RealSense D435") {
    //     irThreshold = 180;
    //     minSize = 10;
    //     maxSize = 370;
    // }
    // SetThreshold(irThreshold);
    // SetMinMaxSize(minSize, maxSize);

    // frame_width = width;
    // frame_height = height;
    // intrinsics_found = false;
    // // Configure the RealSense pipeline

    // config.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    // config.enable_stream(RS2_STREAM_INFRARED, 1, frame_width, frame_height, RS2_FORMAT_Y8, 90);
    // config.enable_stream(RS2_STREAM_DEPTH, frame_width, frame_height, RS2_FORMAT_Z16, 90);

    device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << std::endl;
        return;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "Failed to open device" << std::endl;
        k4a_device_close(device);
        return;
    }

    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    // // Retrive calibration
    calibration;
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << std::endl;
        k4a_device_close(device);
        return;
    }

    Terminated = false;
}

void IRToolTracking::StartToolCalibration()
{
    if (m_IRToolTracker == nullptr)
    {
        m_IRToolTracker = std::make_shared<IRToolTracker>(this);
    }
    m_IRToolTracker->StartCalibration();
    Terminated = false;
}

void IRToolTracking::StopToolCalibration()
{
    if (m_IRToolTracker == nullptr)
        return;
    m_IRToolTracker->StopCalibration();
    Terminated = true;
}

void IRToolTracking::setLaserPower(int power)
{
    // if (dev) {
    //     // Check if the device is a depth sensor and supports laser power control
    //     auto depth_sensor = dev.first<rs2::depth_sensor>();
    //     if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
    //         depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
    //         // Ensure the power level is within the allowable range
    //         auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
    //         power = std::min(std::max(power, static_cast<int>(range.min)), static_cast<int>(range.max));

    //         // Set the laser power
    //         depth_sensor.set_option(RS2_OPTION_LASER_POWER, static_cast<float>(power));
    //     } else {
    //         std::cerr << "This RealSense device does not support changing laser power." << std::endl;
    //     }
    // } else {
    //     std::cerr << "RealSense device not initialized." << std::endl;
    // }
}

void IRToolTracking::getLaserPower(int &power, int &min, int &max)
{
    // if (dev) {
    //     // Check if the device is a depth sensor and supports laser power control
    //     auto depth_sensor = dev.first<rs2::depth_sensor>();
    //     if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
    //         // Get the current laser power
    //         power = depth_sensor.get_option(RS2_OPTION_LASER_POWER);
    //         auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
    //         min = static_cast<int>(range.min);
    //         max = static_cast<int>(range.max);
    //     } else {
    //         std::cerr << "This RealSense device does not support laser power option." << std::endl;
    //     }
        
    // } else {
    //     std::cerr << "RealSense device not initialized." << std::endl;
    // }
}

void IRToolTracking::processStreams() {

    if (Terminated)
        return;
    // Start the pipeline
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout<<"Failed to start device"<<std::endl;
        return;
    }

    // if (K4A_RESULT_SUCCEEDED == k4a_device_get_calibration(device.handle(), config.depth_mode, config.color_resolution, &calibration))
    // {
    //     printf("calibrate succeed\n");
    // }
    // else
    // {
    //     printf("calibrate failed\n");
    // }

    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t ir_image = NULL;
    k4a_image_t undistorted_depth_image = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    int frame_width = 0;
    int frame_height = 0;

    // Continuously capture frames and process them
    while (!Terminated) {
        // Get a depth frame
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            std::cout<<"Timed out waiting for a capture"<<std::endl;
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            std::cout<<"Failed to read a capture"<<std::endl;
            k4a_device_close(device);
            return;
        }

        // Retrieve depth image
        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == NULL)
        {
            std::cout<<"Depth16 None"<<std::endl;
            k4a_capture_release(capture);
            continue;
        }

        ir_image = k4a_capture_get_ir_image(capture);
        if (ir_image == NULL)
        {
            std::cout<<"IR16 None"<<std::endl;
            k4a_capture_release(capture);
            continue;
        }

        // Get the width and height of the depth frame
        frame_width = k4a_image_get_width_pixels(depth_image);
        frame_height = k4a_image_get_height_pixels(depth_image);

        // Colorize the depth frame
        // rs2::frame colorized_depth = color_map.colorize(depth_frame);
    

        // Get the timestamp of the current frame
        double timestamp = k4a_image_get_device_timestamp_usec(depth_image);

        // Convert RealSense frame to OpenCV matrix
        cv::Mat left_frame_image(cv::Size(frame_width, frame_height), CV_16UC1, (void *)k4a_image_get_buffer(ir_image), cv::Mat::AUTO_STEP);
        cv::Mat depth_frame_image(cv::Size(frame_width, frame_height), CV_16UC1, (void *)k4a_image_get_buffer(depth_image), cv::Mat::AUTO_STEP);

        // Convert 16-bit depth values to 8-bit
        cv::Mat depthMat8bit;
        depth_frame_image.clone().convertTo(depthMat8bit, CV_8UC1, 255.0 / 3480); // Adjust the scale according to the depth range
        {
            std::lock_guard<std::mutex> lock(mtx_frames);
            cv::applyColorMap(depthMat8bit, depthFrame, cv::COLORMAP_JET);
            cv::cvtColor(depthFrame, depthFrame, cv::COLOR_BGR2RGB);
        }

        if (m_IRToolTracker != nullptr && (m_IRToolTracker->IsTracking() || m_IRToolTracker->IsCalibrating()) && timestamp > m_latestTrackedFrame)
        {
            // Create a 4x4 identity matrix
            cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);
            m_IRToolTracker->AddFrame(left_frame_image.data, depth_frame_image.data, left_frame_image.cols, left_frame_image.rows, pose ,timestamp);
            m_latestTrackedFrame = playFromFile ? -1 : timestamp;
        }
        
        trackingFrame = m_IRToolTracker->GetProcessedFrame();
        k4a_image_release(depth_image);
        k4a_image_release(ir_image);
        k4a_capture_release(capture);
    }
}

void IRToolTracking::shutdown() {
    // Clean up resources as necessary
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    trackingFrame.release();
    depthFrame.release();
}

void IRToolTracking::StartToolTracking()
{
    if (m_IRToolTracker == nullptr)
    {
        m_IRToolTracker = std::make_shared<IRToolTracker>(this);
    }
    m_IRToolTracker->StartTracking();
    Terminated = false;
}

void IRToolTracking::StopToolTracking()
{
    if (m_IRToolTracker == nullptr)
        return;
    m_IRToolTracker->StopTracking();
    Terminated = true;
}

bool IRToolTracking::IsTrackingTools()
{
    if (m_IRToolTracker == nullptr)
    {
        return false;
    }
    return m_IRToolTracker->IsTracking();
}

void IRToolTracking::SetThreshold(int threshold)
{
    if (m_IRToolTracker == nullptr)
        return;
    irThreshold = threshold;
    m_IRToolTracker->SetThreshold(threshold);
}

int IRToolTracking::GetThreshold()
{
    return irThreshold;
}


void IRToolTracking::SetMinMaxSize(int min, int max)
{
    if (m_IRToolTracker == nullptr)
        return;
    minSize = min;
    maxSize = max;
    m_IRToolTracker->SetMinMaxSize(min, max);
}

void IRToolTracking::GetMinMaxSize(int &min, int &max)
{
    min = minSize;
    max = maxSize;
}

bool IRToolTracking::AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier)
{
    return AddToolDefinition(sphere_count, sphere_positions, sphere_radius, identifier, sphere_count, 0.3f, 0.6f);
}

bool IRToolTracking::AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier, int min_visible_spheres)
{
    return AddToolDefinition(sphere_count, sphere_positions, sphere_radius, identifier, min_visible_spheres, 0.3f, 0.6f);
}

bool IRToolTracking::AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier, int min_visible_spheres, float lowpass_rotation, float lowpass_position)
{
    if (m_IRToolTracker == nullptr)
    {
        m_IRToolTracker = std::make_shared<IRToolTracker>(this);
    }
    //Minimum required spheres for a tool is 3
    if (sphere_count < 3) {
        return false;
    }
    if (sphere_positions.size() != 3 * sphere_count)
        return false;

    cv::Mat3f spheres = cv::Mat3f(sphere_count, 1);
    int j = 0;
    for (int i = 0; i < sphere_count; i++) {
        //Flip z and convert from unity meters to millimeters
        spheres.at<cv::Vec3f>(i, 0) = cv::Vec3f(sphere_positions[j], sphere_positions[j + 1], sphere_positions[j + 2]);
        j += 3;
    }
    return m_IRToolTracker->AddTool(spheres, sphere_radius, identifier, min_visible_spheres, std::clamp(lowpass_rotation, 0.f, 1.f), std::clamp(lowpass_position, 0.f, 1.f));
}

bool IRToolTracking::RemoveToolDefinition(std::string identifier)
{
    if (m_IRToolTracker == nullptr)
        return false;
    return m_IRToolTracker->RemoveTool(identifier);
}

bool IRToolTracking::RemoveAllToolDefinitions()
{
    if (m_IRToolTracker == nullptr)
        return false;
    return m_IRToolTracker->RemoveAllTools();
}

bool IRToolTracking::DeprojectPixelToPoint(float(&uvd)[3], float(&xy)[2])
{
    int valid;
    k4a_float3_t point; // = {0.f, 0.f, 0.f};
    //float pixel[2] = {uvd[0], uvd[1]};
    k4a_float2_t pixel = {uvd[0], uvd[1]};
    float depth = uvd[2];
    k4a_calibration_2d_to_3d(&calibration, &pixel, depth, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &point, &valid);
    if (!valid)
    {
        return false;
    }
    xy[0] = point.v[0];
    xy[1] = point.v[1];

    return true;
}

bool IRToolTracking::ProjectPointToPixel(float(&xyz)[3], float(&uv)[2])
{
    int valid;
    k4a_float3_t point = {xyz[0], xyz[1], xyz[2]};
    k4a_float2_t pixel = {0.f, 0.f};
    k4a_calibration_3d_to_2d(&calibration, &point, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &pixel, &valid);
    if (!valid)
    {
        return false;
    }

    uv[0] = pixel.v[0];
    uv[1] = pixel.v[1];

    return true;
}

std::vector<float> IRToolTracking::GetToolTransform(std::string identifier)
{
    if (m_IRToolTracker == nullptr)
        return std::vector<float>(8, 0);

    cv::Mat transform = m_IRToolTracker->GetToolTransform(identifier);
    std::vector<float> array;
    if (transform.isContinuous()) {
        array.assign((float*)transform.data, (float*)transform.data + transform.total() * transform.channels());
    }
    else {
        for (int i = 0; i < transform.rows; ++i) {
            array.insert(array.end(), transform.ptr<float>(i), transform.ptr<float>(i) + transform.cols * transform.channels());
        }
    }
    return array;
}

bool IRToolTracking::IsCalibratingTool()
{
    if (m_IRToolTracker == nullptr)
    {
        return false;
    }
    return m_IRToolTracker->IsCalibrating();
}

std::vector<float> IRToolTracking::GetToolDefinition()
{
    if (m_IRToolTracker == nullptr)
        return std::vector<float>(12, -1);

    std::vector<float> array = m_IRToolTracker->GetToolDefinition();
    return array;
}

