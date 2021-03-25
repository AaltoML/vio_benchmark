/*
 * @Descripttion: Main function to boost LARVIO.
 * @Author: Xiaochen Qiu
 */


#include <iostream>

#include "utils/DataReader.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include "sensors/ImageData.hpp"

#include "larvio/image_processor.h"
#include "larvio/larvio.h"

#include "Eigen/Dense"

#include "visualization/visualize.hpp"

#include <sys/stat.h>
#include "jsonl_reader.hpp"
#include "imu_sync.hpp"
#include <nlohmann/json.hpp>

using namespace std;

bool fileExists(const std::string filename) {
  struct stat statInfo;
  return (stat(filename.c_str(), &statInfo) == 0);
}

std::string findVideoSuffix(std::string videoPathNoSuffix) {
  for (std::string suffix : { "mov", "avi", "mp4" }) {
    const auto path = videoPathNoSuffix + "." + suffix;
    std::ifstream testFile(videoPathNoSuffix + "." + suffix);
    if (testFile.is_open()) return path;
  }
  return "";
}

inline bool ends_with(std::string const &value, std::string const &ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

int main(int argc, char **argv) {
    if(argc != 5) {
        cerr << endl << "Usage: ./larvio path_to_dataset_folder config_file_path output_file visu/novisu" << endl;
        return 1;
    }

    std::string datasetFolder = argv[1];
    std::string configFile = argv[2];
    std::string outputFile = argv[3];
    std::string mode = argv[4];
    bool visualizerEnabled = "visu" == mode;

    // Read sensors
    vector<larvio::ImuData> allImuData;
    vector<larvio::ImgInfo> allImgInfo;
    JsonlReader reader;
    ImuSync imuSync;
    imuSync.onSyncedLeader = [&allImuData](
        double time,
        double gx, double gy, double gz,
        double ax, double ay, double az) {
        larvio::ImuData imu(time, gx, gy, gz, ax, ay, az);
        allImuData.push_back(imu);
    };
    reader.onAccelerometer = [&imuSync](double time, double x, double y, double z) {
      imuSync.addFollower(time, x, y, z);
    };
    reader.onGyroscope = [&imuSync](double time, double x, double y, double z) {
      imuSync.addLeader(time, x, y, z);
    };
    reader.onFrames = [&allImgInfo](std::vector<JsonlReader::FrameParameters> frames) {
      assert(frames.size() > 0);
      // TODO: Could use just vector<double>
      allImgInfo.push_back(larvio::ImgInfo {
        .timeStampToSec = frames[0].time,
        .imgName = ""
    });
    };
    reader.read(datasetFolder + "/data.jsonl");

    // Setup video input
    std::string camPath = findVideoSuffix(datasetFolder + "/data");
    std::cout << "Reading video file: " << camPath << std::endl;
    cv::VideoCapture videoCapture(camPath);

    // Setup output
    std::ofstream os(outputFile);
    nlohmann::json outputJson = R"({
      "time": 0.0,
      "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      },
      "orientation": {
        "w": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      }
    })"_json;

    // Initialize image processer.
    larvio::ImageProcessorPtr ImgProcesser;
    ImgProcesser.reset(new larvio::ImageProcessor(configFile));
    if (!ImgProcesser->initialize()) {
        cerr << "Image Processer initialization failed!" << endl;
        return 1;
    }

    // Initialize estimator.
    larvio::LarVioPtr Estimator;
    Estimator.reset(new larvio::LarVio(configFile));
    if (!Estimator->initialize()) {
        cerr << "Estimator initialization failed!" << endl;
        return 1;
    }

    // Initialize visualizer
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowActivePoints("menu.Show Active Points",true,true);
    pangolin::Var<bool> menuShowStablePoints("menu.Show Stable Points",true,true);
    pangolin::Var<bool> menuShowSlideWindow("menu.Show Slide Window",true,true);
    pangolin::OpenGlRenderState s_cam(						// Define Camera Render Object (for view / scene browsing)
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,3.5,9, 0,0,0, 0,-1,0)
                );
    pangolin::View* d_cam;
    pangolin::View* d_image;
    bool bFollow = true;
    pangolin::OpenGlMatrix Tbw_pgl;
    std::vector<pangolin::OpenGlMatrix> vPoses;
    std::vector<pangolin::OpenGlMatrix> vPoses_SW;
    std::map<larvio::FeatureIDType,Eigen::Vector3d> mActiveMapPoints;
    std::map<larvio::FeatureIDType,Eigen::Vector3d> mStableMapPoints;
    if (visualizerEnabled) {
        pangolin::CreateWindowAndBind("LARVIO Viewer",1024,768);
        glEnable(GL_DEPTH_TEST);	// 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_BLEND);			// Issue specific OpenGl we might need
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
        d_cam = &pangolin::CreateDisplay()		// Add named OpenGL viewport to window and provide 3D Handler
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
        d_image = &pangolin::CreateDisplay()		// Add named OpenGL viewport to image and provide 3D Handler
            .SetBounds(pangolin::Attach::Pix(0),1.0f,pangolin::Attach::Pix(175),1/3.5f,752.0/480)
            .SetLock(pangolin::LockLeft, pangolin::LockBottom);
    }

    // Main loop
    int k = 0;
    std::vector<larvio::ImuData> imu_msg_buffer;
    cv::Mat tmpImg;
    for(size_t j=0; j<allImgInfo.size(); j++) {
        // get img
        string temp = allImgInfo[j].imgName.substr(0,allImgInfo[j].imgName.size()-1);
        char *fullPath = new char[100];
        memset(fullPath,0,100);
        sprintf(fullPath,"%s/%s",argv[3],temp.c_str());
        larvio::ImageDataPtr imgPtr(new larvio::ImgData);
        imgPtr->timeStampToSec = allImgInfo[j].timeStampToSec;
        // imgPtr->image = cv::imread(fullPath,0);
        videoCapture.read(tmpImg);
        cv::cvtColor(tmpImg, imgPtr->image, cv::COLOR_BGR2GRAY);

        // get imus
        while (allImuData[k].timeStampToSec-imgPtr->timeStampToSec < 0.05 && k<allImuData.size()) {
            imu_msg_buffer.push_back(
                larvio::ImuData(allImuData[k].timeStampToSec, allImuData[k].angular_velocity, allImuData[k].linear_acceleration));
            ++k;
        }

        // process
        larvio::MonoCameraMeasurementPtr features = new larvio::MonoCameraMeasurement;
        int64 start_time_fe = cv::getTickCount();
        bool bProcess = ImgProcesser->processImage(imgPtr, imu_msg_buffer, features);				// process image
        int64 end_time_fe = cv::getTickCount();
        double processing_time_fe = (end_time_fe - start_time_fe)/cv::getTickFrequency();
        double processing_time_be;
        bool bPubOdo = false;
        if (bProcess) {
            int64  start_time_be = cv::getTickCount();
            bPubOdo = Estimator->processFeatures(features, imu_msg_buffer);							// state estimation
            int64 end_time_be = cv::getTickCount();
            processing_time_be = (end_time_be - start_time_be)/cv::getTickFrequency();
        }
        if (bPubOdo) {
            Eigen::Isometry3d pose = Estimator->getTbw();
            Quaterniond rotation(pose.rotation());
            outputJson["time"] = imgPtr->timeStampToSec;
            outputJson["position"]["x"] = pose.translation()(0);
            outputJson["position"]["y"] = pose.translation()(1);
            outputJson["position"]["z"] = pose.translation()(2);
            outputJson["orientation"]["w"] = rotation.w();
            outputJson["orientation"]["x"] = rotation.x();
            outputJson["orientation"]["y"] = rotation.y();
            outputJson["orientation"]["z"] = rotation.z();
            os << outputJson.dump() << std::endl;

            // Visualization
            // draw 3D
            if (visualizerEnabled) {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                larvio::visualize::GetCurrentOpenGLPoseMatrix(Tbw_pgl, pose);
                if(menuFollowCamera && bFollow) {
                    s_cam.Follow(Tbw_pgl);
                }
                else if(menuFollowCamera && !bFollow) {
                    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,3.5,9, 0,0,0, 0,-1,0));
                    s_cam.Follow(Tbw_pgl);
                    bFollow = true;
                }
                else if(!menuFollowCamera && bFollow) {
                    bFollow = false;
                }
                d_cam->Activate(s_cam);
                glClearColor(0.0f,0.0f,0.0f,1.0f);
                larvio::visualize::DrawCurrentPose(Tbw_pgl);
                larvio::visualize::DrawKeyFrames(vPoses);
                if (menuShowSlideWindow) {
                    vector<Eigen::Isometry3d> swPoses;
                    Estimator->getSwPoses(swPoses);
                    larvio::visualize::GetSwOpenGLPoseMatrices(vPoses_SW, swPoses);
                    larvio::visualize::DrawSlideWindow(vPoses_SW);
                }
                if (menuShowActivePoints) {
                    Estimator->getActiveeMapPointPositions(mActiveMapPoints);
                    larvio::visualize::DrawActiveMapPoints(mActiveMapPoints);
                    mActiveMapPoints.clear();
                }
                if (menuShowStablePoints) {
                    Estimator->getStableMapPointPositions(mStableMapPoints);
                    larvio::visualize::DrawStableMapPoints(mStableMapPoints);
                }

                // draw tracking image
                cv::Mat img4Show;
                ImgProcesser->getVisualImg().copyTo(img4Show);
                stringstream ss;
                ss << "fps: " << 1.0/(processing_time_fe+processing_time_be) << " HZ";
                string msg = ss.str();
                int baseLine=0;
                cv::Size textSize = cv::getTextSize(msg, 0, 0.01, 10, &baseLine);
                cv::Point textOrigin(2*baseLine,textSize.height+4*baseLine);
                putText(img4Show, msg, textOrigin, 1, 2, cv::Scalar(0,165,255), 2);
                pangolin::GlTexture imageTexture(img4Show.cols,img4Show.rows);;
                imageTexture.Upload(img4Show.data,GL_BGR,GL_UNSIGNED_BYTE);
                d_image->Activate();
                glColor3f(1.0,1.0,1.0);
                imageTexture.RenderToViewportFlipY();

                pangolin::FinishFrame();

                vPoses.push_back(Tbw_pgl);
            }
        }
    }

    cout << "Totally " << mStableMapPoints.size() << " SLAM points" << endl;

    // keep the final scene in pangolin
    if (visualizerEnabled) {
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            if(menuFollowCamera && bFollow) {
                s_cam.Follow(Tbw_pgl);
            }
            else if(menuFollowCamera && !bFollow) {
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,3.5,9, 0,0,0, 0,-1,0));
                s_cam.Follow(Tbw_pgl);
                bFollow = true;
            }
            else if(!menuFollowCamera && bFollow) {
                bFollow = false;
            }
            d_cam->Activate(s_cam);
            glClearColor(0.0f,0.0f,0.0f,1.0f);
            larvio::visualize::DrawCurrentPose(Tbw_pgl);
            larvio::visualize::DrawKeyFrames(vPoses);
            if (menuShowSlideWindow) {
                larvio::visualize::DrawSlideWindow(vPoses_SW);
            }
            if (menuShowStablePoints) {
                larvio::visualize::DrawStableMapPoints(mStableMapPoints);
            }
            pangolin::FinishFrame();
        }
    }

    cout << "Finished! Exiting..." << endl;
    return 0;
}
