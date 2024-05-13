#include "IRToolTracking.h"
#include <iostream>
#include <algorithm>

IRToolTracking::IRToolTracking() {
    // Constructor: Initialize any required variables or state
}

void IRToolTracking::queryDevices() {

    device_count = k4a_device_get_installed_count();
    deviceNames.clear();
    if (device_count == 0)
    {
        std::cerr << "No K4A devices found" << std::endl;
        return;
    }

    for (size_t i = 0; i < device_count; i++)
    {
        
        k4a_device_open(i, &device);
        char serial_number[256];
        size_t serial_number_length = sizeof(serial_number);
        k4a_device_get_serialnum(device, serial_number, &serial_number_length);
        k4a_device_close(device);
        
        std::stringstream ss;
        ss << "[" << i << "] " << "K4A Device" << " (" << serial_number << ")";
        deviceNames.push_back(ss.str());
    }
}

void IRToolTracking::initializeFromFile(const std::string& file) {
    
    auto result = k4a_playback_open(file.c_str(), &playback);
    if (result  != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to open file " << file << std::endl;
        return;
    }
    result = k4a_playback_get_calibration(playback, &calibration);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        std::cerr << "Failed to get calibration" << std::endl;
        return;
    }
    Terminated = false;
    playFromFile = true;

}

void IRToolTracking::initialize(int index, int width, int height) {

    if (index < 0 || index >= static_cast<int>(device_count)) {
        std::cerr << "Invalid device index." << std::endl;
        Terminated = true;
        return;
    }


    // SetThreshold(irThreshold);
    // SetMinMaxSize(minSize, maxSize);

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(index, &device))
    {
        std::cout << "Failed to open device" << std::endl;
        k4a_device_close(device);
        return;
    }

    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    // Retrive calibration
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
   
}

void IRToolTracking::getLaserPower(int &power, int &min, int &max)
{
    
}

void IRToolTracking::processStreams() {

    if (Terminated)
        return;
    // Start the pipeline
    if (!playFromFile)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
        {
            std::cout << "Failed to start device" << std::endl;
            return;
        }
    }

    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t ir_image = NULL;
    k4a_image_t undistorted_depth_image = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    int frame_width = 0;
    int frame_height = 0;

    // Continuously capture frames and process them
    while (!Terminated) {

		if (playFromFile)
		{
            switch (k4a_playback_get_next_capture(playback, &capture))
            {
			case K4A_STREAM_RESULT_SUCCEEDED:
				break;
			case K4A_STREAM_RESULT_FAILED:
				std::cout << "Timed out waiting for a capture" << std::endl;
				continue;
				break;
			case K4A_STREAM_RESULT_EOF:
				std::cout << "Reached end of the file" << std::endl;
				return;
            }
		}
		else
		{
			// Get a depth frame
			switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				break;
			case K4A_WAIT_RESULT_TIMEOUT:
				std::cout << "Timed out waiting for a capture" << std::endl;
				continue;
				break;
			case K4A_WAIT_RESULT_FAILED:
				std::cout << "Failed to read a capture" << std::endl;
				k4a_device_close(device);
				return;
			}
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

		if (playFromFile)
		{
			// Sleep for 33ms to simulate 30fps
			std::this_thread::sleep_for(std::chrono::milliseconds(33));
		}
    }
}

void IRToolTracking::shutdown() {
    // Clean up resources as necessary
    if (device != NULL)
    {
        k4a_device_close(device);
    }
	if (playback != NULL)
	{
		k4a_playback_close(playback);
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

