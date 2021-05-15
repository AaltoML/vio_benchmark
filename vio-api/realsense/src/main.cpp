#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <condition_variable>
#include <thread>
#include <opencv2/opencv.hpp>
#include "jsonl_reader.hpp"
#include "api/vio_implementation.hpp"
#include "api/types.hpp"
#include <accelerated-arrays/opencv_adapter.hpp>
#include <librealsense2/rs.hpp>

template <class T> class ThreadSafeQueue {
public:
    ThreadSafeQueue(void) : q(), m(), c() {}
    ~ThreadSafeQueue(void) {}
    void enqueue(T t) {
        std::lock_guard<std::mutex> lock(m);
        q.push(t);
        c.notify_one();
    }

    T dequeue(void) {
        std::unique_lock<std::mutex> lock(m);
        while(q.empty()) {
            c.wait(lock);
        }
        T v = q.front();
        q.pop();
        return v;
    }
private:
    std::queue<T> q;
    mutable std::mutex m;
    std::condition_variable c;
};

struct ImuSample {
    double time;
    api::Vector3d sample;
};

int main(int argc, char * argv[]) try {
    if(argc < 3) {
        std::cerr << std::endl << "Usage: ./main config calibration [video/pose/both]" << std::endl;
        return 1;
    }

    std::string configPath = argv[1];
    std::string calibPath = argv[2];
    std::string visuMode = argc >= 4 ? argv[3] : "";
    bool visualizeVideo = visuMode == "video" || visuMode == "both";
    bool visualizePose = visuMode == "pose" || visuMode == "both";

    std::unique_ptr<api::VioApi> vioApi;
    {
        std::ifstream configFile(configPath);
        std::ifstream calibrationFile(calibPath);
        vioApi = api::buildVio(calibrationFile, configFile);
    } // Close files


    std::shared_ptr<api::Visualization> visualizationVideo;
    std::shared_ptr<api::Visualization> visualizationPose;
    std::shared_ptr<accelerated::Image> frameVisualizationImage;
    cv::Mat poseImage;

    if (visualizeVideo) visualizationVideo = vioApi->createVisualization("VIDEO");
    if (visualizePose) visualizationPose = vioApi->createVisualization("POSE");

    std::cout << std::setprecision(18);
    ThreadSafeQueue<std::shared_ptr<const api::VioApi::VioOutput>> outputQueue;
    vioApi->onOutput = [&](std::shared_ptr<const api::VioApi::VioOutput> output) {
        outputQueue.enqueue(output);
    };

    ThreadSafeQueue<const ImuSample> accQueue;
    std::thread accThread([&]() {
        while (true) {
            auto imuSample = accQueue.dequeue();
            vioApi->addAcc(imuSample.time, imuSample.sample);
        }
    });

    ThreadSafeQueue<const ImuSample> gyroQueue;
    std::thread gyroThread([&]() {
        while (true) {
            auto imuSample = gyroQueue.dequeue();
            vioApi->addGyro(imuSample.time, imuSample.sample);
        }
    });

    // Attempt hardware reset to ensure device will work properly
    std::cout << "Reseting T265 to ensure smooth operation..." << std::endl;
    {
        rs2::config resetCfg;
        rs2::pipeline resetPipe;
        resetCfg.resolve(resetPipe).get_device().hardware_reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "Reset completed!" << std::endl;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // 6 Degrees of Freedom pose data, calculated by RealSense device
    // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Native stream of gyroscope motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    // Native stream of accelerometer motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    // Native stream of fish-eye (wide) data captured from the dedicate motion camera
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    uint8_t* leftFrame;
    uint8_t* rightFrame;
    int width = -1;
    int height = -1;
    api::CameraParameters leftCamParams;
    api::CameraParameters rightCamParams;

    auto callback = [&](const rs2::frame& frame) {
        // Convert timestamp to seconds
        double timeStamp = frame.get_timestamp() / 1000; // ms to s

        // Cast the frame that arrived to motion frame, accelerometer + gyro
        auto motion = frame.as<rs2::motion_frame>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector gyro_data = motion.get_motion_data();
            gyroQueue.enqueue({timeStamp, api::Vector3d{gyro_data.x, gyro_data.y, gyro_data.z}});
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector accel_data = motion.get_motion_data();
            accQueue.enqueue({timeStamp, api::Vector3d{accel_data.x, accel_data.y, accel_data.z}});
        }

        // Cast to frameset that contains video feed from all cameras
        auto frameset = frame.as<rs2::frameset>();
        if (frameset && frameset.get_profile().stream_type() == RS2_STREAM_FISHEYE && frameset.get_profile().format() == RS2_FORMAT_Y8) {
            // Process feed from both cameras
            // std::vector<recorder::FrameData> frameGroup;
            for (int index = 0; index < 2; index++) {
                rs2::video_frame vf = frameset.get_fisheye_frame(index + 1); // Camera index starts at 1
                // auto vprofile = vf.get_profile().as<rs2::video_stream_profile>();
                // auto intrinsics = vprofile.get_intrinsics();
                uint8_t* imageData = const_cast<uint8_t*>((const uint8_t*)vf.get_data());
                assert(imageData != nullptr);
                width = vf.get_width();
                height = vf.get_height();

                api::CameraParameters camParams;
                // TODO: Could have an option to use calibration from RealSense device, it provides lens distortion etc.
                // but without all of that, it's probably best just use fixed values form calibration
                // camParams.focalLengthX = intrinsics.fx;
                // camParams.focalLengthY = intrinsics.fy;
                // camParams.principalPointX = intrinsics.ppx;
                // camParams.principalPointY = intrinsics.ppy;
                if (index == 0) {
                    leftFrame = imageData;
                    leftCamParams = camParams;
                } else {
                    rightFrame = imageData;
                    rightCamParams = camParams;
                }

                // recorder::FrameData frameData({
                //    .t = timeStamp,
                //    .cameraInd = index,
                //    .focalLengthX = intrinsics.fx,
                //    .focalLengthY = intrinsics.fy,
                //    .px = intrinsics.ppx,
                //    .py = intrinsics.ppy,
                //    .frameData = colorFrames + index
                // });
                // frameGroup.push_back(frameData);
                // if (!videoInitialized[index]) {
                //     recorder->setVideoRecordingFps((float)vprofile.fps());
                //     videoInitialized[index] = true;

                //     // Store lens metadata once
                //     nlohmann::json lensMetadata;
                //     lensMetadata["model"] = distortionToString(intrinsics.model);
                //     auto coeffs = nlohmann::json::array();
                //     for(auto i = 0; i < 5; ++i) {
                //         coeffs.push_back(intrinsics.coeffs[i]);
                //     }
                //     lensMetadata["coeffs"] = coeffs;
                //     lensMetadata["cameraInd"] = index;
                //     recorder->addJson(lensMetadata);
                // }
            }
            int inputTag = 0;
            vioApi->addFrameStereo(
                timeStamp,
                leftCamParams, rightCamParams,
                width,
                height,
                leftFrame,
                rightFrame,
                api::VioApi::ColorFormat::GRAY,
                inputTag);
            // vioApi->addFrameMono(
            //     timeStamp,
            //     leftCamParams,
            //     width,
            //     height,
            //     leftFrame,
            //     api::VioApi::ColorFormat::GRAY,
            //     inputTag);
        }
    };

    std::cout << "Connecting to device...\n";

    // Start pipeline with chosen configuration
    auto profile = pipe.start(cfg, callback);

    if (visualizeVideo || visualizePose)
        std::cout << "!!! Press Ctrl+c or Space to stop !!!\n";
    else
        std::cout << "!!! Press Ctrl+c to stop !!!\n";
    while(true) { // Process output in main thread so it renders on Mac
        auto output = outputQueue.dequeue();
        if (!visualizePose && !visualizeVideo) {
            std::cout
                << "Vio API pose: "
                << output->pose.time << ", "
                << output->pose.position.x << ", "
                << output->pose.position.y << ", "
                << output->pose.position.z << ", "
                << output->pose.orientation.x << ", "
                << output->pose.orientation.y << ", "
                << output->pose.orientation.z << ", "
                << output->pose.orientation.w << std::endl;
        }
        if (visualizePose) {
            if (poseImage.empty())
                poseImage = cv::Mat(640, 480, CV_8UC3);
            constexpr int GRAY = 150;
            poseImage = cv::Scalar(GRAY, GRAY, GRAY);
            visualizationPose->update(output);
            visualizationPose->render(poseImage);
            cv::imshow("Pose", poseImage);
        }
        if (visualizeVideo) {
            if (!frameVisualizationImage)
                frameVisualizationImage = visualizationVideo->createDefaultRenderTarget();
            visualizationVideo->update(output);
            visualizationVideo->render(*frameVisualizationImage);
            cv::imshow("Video", accelerated::opencv::ref(*frameVisualizationImage));
        }

        if (visualizePose || visualizeVideo)
            // TODO: This only works with some visualizatins enabled
            if (cv::waitKey(1) == 32) break; // 32 == Space
    }

    std::cout << "\nExiting.\n";

    pipe.stop();

    std::cout << "Bye!\n";

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
