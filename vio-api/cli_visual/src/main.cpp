#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "jsonl_reader.hpp"
#include "api/vio_implementation.hpp"
#include "api/types.hpp"
#include <accelerated-arrays/opencv_adapter.hpp>

int main(int argc, char **argv) {
    if(argc < 5) {
        std::cerr << std::endl << "Usage: ./vio_cli_example config calibration data_json left_video [right_video]" << std::endl;
        return 1;
    }

    std::string configPath = argv[1];
    std::string calibPath = argv[2];
    std::string dataJsonPath = argv[3];
    std::string leftVideoPath = argv[4];
    std::string rightVideoPath = argc == 6 ? argv[5] : "";

    bool stereo = !rightVideoPath.empty();

    std::vector<cv::VideoCapture> videoCaptures;
    videoCaptures.push_back(cv::VideoCapture(leftVideoPath));
    if (stereo) videoCaptures.push_back(cv::VideoCapture(rightVideoPath));

    cv::Mat leftFrame;
    cv::Mat rightFrame;

    std::unique_ptr<api::VioApi> vioApi;
    {
        std::ifstream configFile(configPath);
        std::ifstream calibrationFile(calibPath);
        vioApi = api::buildVio(calibrationFile, configFile);
    } // Close files

    std::shared_ptr<api::Visualization> visualizationVideo = vioApi->createVisualization("VIDEO");
    std::shared_ptr<api::Visualization> visualizationPose = vioApi->createVisualization("POSE");
    std::shared_ptr<accelerated::Image> frameVisualizationImage;
    cv::Mat poseImage;

    std::cout << std::setprecision(18);
    vioApi->onOutput = [&vioApi, &visualizationVideo, &visualizationPose, &frameVisualizationImage, &poseImage](std::shared_ptr<const api::VioApi::VioOutput> output) {
        std::cout
            << output->pose.time << ", "
            << output->pose.position.x << ", "
            << output->pose.position.y << ", "
            << output->pose.position.z << ", "
            << output->pose.orientation.x << ", "
            << output->pose.orientation.y << ", "
            << output->pose.orientation.z << ", "
            << output->pose.orientation.w << std::endl;

            if (!frameVisualizationImage) {
                frameVisualizationImage = visualizationVideo->createDefaultRenderTarget();
                poseImage = cv::Mat(640, 480, CV_8UC3);
            }
            visualizationVideo->update(output);
            visualizationVideo->render(*frameVisualizationImage);
            constexpr int GRAY = 150;
            poseImage = cv::Scalar(GRAY, GRAY, GRAY);
            visualizationPose->update(output);
            visualizationPose->render(poseImage);
            cv::imshow("Video", accelerated::opencv::ref(*frameVisualizationImage));
            cv::imshow("Pose", poseImage);
            int key = cv::waitKey(1);
    };

    int lastInputTag = 0;

    JsonlReader reader;
    reader.onAccelerometer = [&vioApi](double time, double x, double y, double z) {
        vioApi->addAcc(time, api::Vector3d{x, y, z});
    };
    reader.onGyroscope = [&vioApi](double time, double x, double y, double z) {
        vioApi->addGyro(time, api::Vector3d{x, y, z});
    };
    reader.onFrames = [&vioApi, &videoCaptures, &leftFrame, &rightFrame, &stereo, &lastInputTag](std::vector<JsonlReader::FrameParameters> frames) {
        assert(frames.size() > 0);
        videoCaptures[0].read(leftFrame);
        if (stereo) videoCaptures[1].read(rightFrame);
        double time = frames[0].time;
        auto colorFormat = leftFrame.type() == CV_8UC1
            ? api::VioApi::ColorFormat::GRAY : api::VioApi::ColorFormat::RGB;
        api::CameraParameters leftCamParams;
        leftCamParams.focalLengthX = frames[0].focalLengthX;
        leftCamParams.focalLengthY = frames[0].focalLengthY;
        leftCamParams.principalPointX = frames[0].principalPointX;
        leftCamParams.principalPointY = frames[0].principalPointY;
        if (!stereo) {
            vioApi->addFrameMono(
                time,
                leftCamParams,
                leftFrame.cols,
                leftFrame.rows,
                leftFrame.data,
                colorFormat,
                lastInputTag);
        } else {
            api::CameraParameters rightCamParams;
            rightCamParams.focalLengthX = frames[1].focalLengthX;
            rightCamParams.focalLengthY = frames[1].focalLengthY;
            rightCamParams.principalPointX = frames[1].principalPointX;
            rightCamParams.principalPointY = frames[1].principalPointY;
            vioApi->addFrameStereo(
                time,
                leftCamParams, rightCamParams,
                leftFrame.cols,
                leftFrame.rows,
                leftFrame.data,
                rightFrame.data,
                colorFormat,
                lastInputTag);
        }
        lastInputTag++;
    };

    reader.read(dataJsonPath);

    return 0;
}
