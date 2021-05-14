#include <iostream>
#include <fstream>

#include "api/vio_implementation.hpp"
#include "api/types.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// TODO: Stupid hack to get sync.registerCallback(...) working
boost::function<void (const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1)> onStereoImage;
void callback(const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1) {
  onStereoImage(image0, image1);
}

int main(int argc, char **argv) {
    // TODO: Supports only stereo camera atm, time sync could be stripped for mono

    ros::init(argc, argv, "vio");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::string configPath;
    std::string calibPath;

    n.getParam("config", configPath);
    n.getParam("calibration", calibPath);

    std::cout << "Creating VioApi instance" << std::endl;
    std::unique_ptr<api::VioApi> vioApi;
    {
        std::ifstream configFile(configPath);
        std::ifstream calibrationFile(calibPath);
        vioApi = api::buildVio(calibrationFile, configFile);
    } // Close files

    // TODO: There doesn't seem to be routing in ROS(?), should probably use YAML to configure these
    std::string gyroTopic = "/camera/gyro/sample";
    std::string accTopic = "/camera/accel/sample";
    std::string cam0Topic = "/camera/fisheye1/image_raw";
    std::string cam1Topic = "/camera/fisheye2/image_raw";
    std::string vioOutputTopic = "/vio/odom";

    bool receivedGyro = false;
    bool receivedAcc = false;
    bool receivedStereo = false;

    int lastInputTag = 0;

    ros::Publisher outputPub = n.advertise<nav_msgs::Odometry>(vioOutputTopic, 1000);

    vioApi->onOutput = [&](std::shared_ptr<const api::VioApi::VioOutput> output) {
        std::cout << "Pose: "
            << output->pose.time << ", "
            << output->pose.position.x << ", "
            << output->pose.position.y << ", "
            << output->pose.position.z << ", "
            << output->pose.orientation.x << ", "
            << output->pose.orientation.y << ", "
            << output->pose.orientation.z << ", "
            << output->pose.orientation.w << std::endl;

        auto timeStamp = ros::Time(output->pose.time);;
        std::string frameId = "vio-odom";

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

    std::cout << "Setting up subscribers" << std::endl;

    onStereoImage =
        [&](const sensor_msgs::ImageConstPtr& image0, const sensor_msgs::ImageConstPtr& image1){
            if (!receivedStereo) {
                std::cout << "Received first stereo pair! "
                    << image0->width << "x" << image0->height << std::endl;
                receivedStereo = true;
            }

            // TODO: Could support fetching values for these from ROS topics
            api::CameraParameters leftCamParams;
            api::CameraParameters rightCamParams;

            assert(image0->encoding == image1->encoding);
            assert(image0->width == image1->width);
            assert(image0->height == image1->height);
            auto encoding = image0->encoding;

            api::VioApi::ColorFormat colorFormat;
            if (encoding == sensor_msgs::image_encodings::MONO8) {
                colorFormat = api::VioApi::ColorFormat::GRAY;
            } else if (encoding == sensor_msgs::image_encodings::RGB8 || encoding == sensor_msgs::image_encodings::BGR8) {
                // TODO: Should flip channels
                colorFormat = api::VioApi::ColorFormat::RGB;
            } else {
                ROS_ERROR("Unspported image encoding: %s", encoding.c_str());
                return;
            }

            double time = image0->header.stamp.toSec();
            vioApi->addFrameStereo(
                time,
                leftCamParams, rightCamParams,
                image0->width,
                image0->height,
                const_cast<unsigned char*>(image0->data.data()),
                const_cast<unsigned char*>(image1->data.data()),
                colorFormat,
                lastInputTag);

            lastInputTag++;
        };
    constexpr uint32_t QUEUE_SIZE = 1;
    message_filters::Subscriber<sensor_msgs::Image> camSub0(n, cam0Topic, QUEUE_SIZE);
    message_filters::Subscriber<sensor_msgs::Image> camSub1(n, cam1Topic, QUEUE_SIZE);
    constexpr uint32_t SYNC_QUEUE_SIZE = 2;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(camSub0, camSub1, SYNC_QUEUE_SIZE);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber gyroSub = n.subscribe<sensor_msgs::Imu>(gyroTopic, 1000, [&](const sensor_msgs::ImuConstPtr &imu_msg){
        if (!receivedGyro) {
            std::cout << "Received first gyro sample! "
                << imu_msg->angular_velocity.x << ", "
                << imu_msg->angular_velocity.y << ", "
                << imu_msg->angular_velocity.z << std::endl;
            receivedGyro = true;
        }
        double time = imu_msg->header.stamp.toSec();
        vioApi->addGyro(time, api::Vector3d{
          imu_msg->angular_velocity.x,
          imu_msg->angular_velocity.y,
          imu_msg->angular_velocity.z});
    });

    ros::Subscriber accSub = n.subscribe<sensor_msgs::Imu>(accTopic, 1000, [&](const sensor_msgs::ImuConstPtr &imu_msg){
        if (!receivedAcc) {
            std::cout << "Received first acc sample! "
                << imu_msg->linear_acceleration.x << ", "
                << imu_msg->linear_acceleration.y << ", "
                << imu_msg->linear_acceleration.z << std::endl;
            receivedAcc = true;
        }
        double time = imu_msg->header.stamp.toSec();
        vioApi->addAcc(time, api::Vector3d{
          imu_msg->linear_acceleration.x,
          imu_msg->linear_acceleration.y,
          imu_msg->linear_acceleration.z});
    });

    std::cout << "Init done" << std::endl;

    ros::spin();

    return 0;
}
