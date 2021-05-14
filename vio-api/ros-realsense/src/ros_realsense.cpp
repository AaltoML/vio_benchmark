#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <thread>
#include "jsonl_reader.hpp"
#include "api/vio_implementation.hpp"
#include "api/types.hpp"

#include <librealsense2/rs.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char **argv) {
    // TODO: Supports only stereo camera atm, time sync could be stripped for mono

    ros::init(argc, argv, "realsense");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string configPath;
    std::string calibPath;

    n.getParam("config", configPath);
    n.getParam("calibration", calibPath);

    std::string vioOutputTopic = "/odom";

    std::cout << "Creating VioApi instance" << std::endl;
    std::unique_ptr<api::VioApi> vioApi;
    {
        std::ifstream configFile(configPath);
        std::ifstream calibrationFile(calibPath);
        vioApi = api::buildVio(calibrationFile, configFile);
    } // Close files

    ros::Publisher outputPub = n.advertise<nav_msgs::Odometry>(vioOutputTopic, 1000);

    std::cout << std::setprecision(18);
    vioApi->onOutput = [&](std::shared_ptr<const api::VioApi::VioOutput> output) {
        //std::cout
        //    << output->pose.time << ", "
        //    << output->pose.position.x << ", "
        //    << output->pose.position.y << ", "
        //    << output->pose.position.z << ", "
        //    << output->pose.orientation.x << ", "
        //    << output->pose.orientation.y << ", "
        //    << output->pose.orientation.z << ", "
        //    << output->pose.orientation.w << std::endl;

        auto timeStamp = ros::Time(output->pose.time);;
        std::string frameId = "odom";

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = timeStamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = frameId;
        transformStamped.transform.translation.x = output->pose.position.x;;
        transformStamped.transform.translation.y = output->pose.position.y;;
        transformStamped.transform.translation.z = output->pose.position.z;
        transformStamped.transform.rotation.x = output->pose.orientation.x;
        transformStamped.transform.rotation.y = output->pose.orientation.y;
        transformStamped.transform.rotation.z = output->pose.orientation.z;
        transformStamped.transform.rotation.w = output->pose.orientation.w;
        br.sendTransform(transformStamped);

        nav_msgs::Odometry odom;
        odom.header.stamp = timeStamp;
        odom.header.frame_id = frameId;
        odom.pose.pose.position.x = output->pose.position.x;
        odom.pose.pose.position.y = output->pose.position.y;
        odom.pose.pose.position.z = output->pose.position.z;
        odom.pose.pose.orientation.x = output->pose.orientation.x;
        odom.pose.pose.orientation.y = output->pose.orientation.y;
        odom.pose.pose.orientation.z = output->pose.orientation.z;
        odom.pose.pose.orientation.w = output->pose.orientation.w;
        outputPub.publish(odom);
    };

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
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF /*, 15 (fps)*/);
    // Native stream of gyroscope motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);
    // Native stream of accelerometer motion data produced by RealSense device
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F /*, 15 (fps)*/);
    // Native stream of fish-eye (wide) data captured from the dedicate motion camera
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    uint8_t* leftFrame;
    uint8_t* rightFrame;
    api::CameraParameters leftCamParams;
    api::CameraParameters rightCamParams;
    int width = -1;
    int height = -1;
    std::mutex addFrameMutex;
    //int frameNumber = 0;
    auto callback = [&](const rs2::frame& frame) {
        // Convert timestamp to seconds
        double timeStamp = frame.get_timestamp() / 1000; // ms to s

        // Cast the frame that arrived to motion frame, accelerometer + gyro
        auto motion = frame.as<rs2::motion_frame>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector gyro_data = motion.get_motion_data();
            vioApi->addGyro(timeStamp, api::Vector3d{gyro_data.x, gyro_data.y, gyro_data.z});
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            rs2_vector accel_data = motion.get_motion_data();
            vioApi->addAcc(timeStamp, api::Vector3d{accel_data.x, accel_data.y, accel_data.z});
        }

        // Cast to frameset that contains video feed from all cameras
        auto frameset = frame.as<rs2::frameset>();
        if (frameset && frameset.get_profile().stream_type() == RS2_STREAM_FISHEYE && frameset.get_profile().format() == RS2_FORMAT_Y8) {
            // Process feed from both cameras
            std::lock_guard<std::mutex> lock(addFrameMutex);

            // frameNumber++;
            // if (frameNumber % 2 == 0) // Skip every other frame
            //     return;

            for (int index = 0; index < 2; index++) {
                rs2::video_frame vf = frameset.get_fisheye_frame(index + 1); // Camera index starts at 1
                // Save frame metadata
                // auto vprofile = vf.get_profile().as<rs2::video_stream_profile>();
                // auto intrinsics = vprofile.get_intrinsics();

                // Save frame
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
                    //leftFrame = const_cast<void*>(vf.get_data());
                    leftCamParams = camParams;
                } else {
                    rightFrame = imageData;
                    //rightFrame = const_cast<void*>(vf.get_data());
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
        }
    };

    std::cout << "Connecting to device...\n";

    // Start pipeline with chosen configuration
    auto profile = pipe.start(cfg, callback);

    std::cout << "Custom Vio Running!\n";

    ros::spin();

    pipe.stop();

    std::cout << "Bye!\n";

    return EXIT_SUCCESS;

}
