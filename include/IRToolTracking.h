#pragma once

#ifndef IRTOOLTRACKING_H
#define IRTOOLTRACKING_H

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <mutex>
#include <queue>
#include <condition_variable>
#include <iostream>

#include "IRToolTrack.h"


/// Join a std::shared_ptr<std::thread>
inline void JoinThread(std::shared_ptr<std::thread>& th)
{
    if (th) {
        try {
            if (th->joinable()) {
                th->join();
            }
        } catch (std::system_error& /*err*/) {}
        th = nullptr;
    }
}



class FrameQueue {
private:
    std::queue<cv::Mat> queue;
    std::mutex mutex;
    std::condition_variable cond;

public:
    cv::Mat lastframe;
    void push(cv::Mat frame) {
        std::lock_guard<std::mutex> lock(mutex);
        queue.push(frame);
        cond.notify_one();
    }

    cv::Mat pop() {
        std::unique_lock<std::mutex> lock(mutex);
        cond.wait(lock, [this]{ return !queue.empty(); });
        while (queue.size() > 1) {
            queue.pop();
        }
        auto frame = std::move(queue.front());
        lastframe = frame.clone();
        queue.pop();
        return frame;
    }

    bool empty() {
        std::lock_guard<std::mutex> lock(mutex);
        return queue.empty();
    }
};


class IRToolTracking {
public:
    IRToolTracking();

    void queryDevices();
    void initialize(int index, int width, int height);
    void initializeFromFile(const std::string& file);
    void StartToolCalibration();
    void StopToolCalibration();
    void processStreams();
    void StartToolTracking();
    void StopToolTracking();
    bool AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier);
    bool AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier, int min_visible_spheres);
    bool AddToolDefinition(int sphere_count, std::vector<float> sphere_positions, float sphere_radius, std::string identifier, int min_visible_spheres, float lowpass_rotation, float lowpass_position);
    bool RemoveToolDefinition(std::string identifier);
    bool RemoveAllToolDefinitions();
    bool IsTrackingTools();
    bool IsCalibratingTool();
    void shutdown();

    void SetThreshold(int threshold);
    int GetThreshold();

    void SetMinMaxSize(int min, int max);
    void GetMinMaxSize(int &min, int &max);

    void setLaserPower(int power);
    void getLaserPower(int &power, int &min, int &max);

    std::vector<std::string>& getDeviceList() { return deviceNames; }
    bool DeprojectPixelToPoint(float(&uvd)[3], float(&xy)[2]);
    bool ProjectPointToPixel(float(&xyz)[3], float(&uv)[2]);
    std::vector<float> GetToolTransform(std::string identifier);
    std::vector<float> GetToolDefinition();

    bool IsTerminated() const
    {
        return Terminated;
    }

    cv::Mat getNextDepthFrame() {
        std::lock_guard<std::mutex> lock(mtx_frames);
        return depthFrame;
    }

    cv::Mat getNextIRFrame() {
        std::lock_guard<std::mutex> lock(mtx_frames);
        return trackingFrame;
    }   

private:

    // int frame_width = 848;
    // int frame_height = 480;
    int irThreshold = 100;
    int minSize = 10;
    int maxSize = 280;

    std::mutex mtx_frames;
    cv::Mat trackingFrame;
    cv::Mat depthFrame;

    std::atomic<bool> Terminated = ATOMIC_VAR_INIT(false);
    std::shared_ptr<std::thread> Thread;

    std::shared_ptr<IRToolTracker> m_IRToolTracker = nullptr;
    double m_latestTrackedFrame = -1;
    bool playFromFile = false;

    bool intrinsics_found = false;

    std::vector<std::string> deviceNames;

    uint32_t device_count = 0;
    k4a_device_t device = NULL;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_calibration_t calibration;

    k4a_playback_t playback = NULL;

};

#endif // IRTOOLTRACKING_HPP
