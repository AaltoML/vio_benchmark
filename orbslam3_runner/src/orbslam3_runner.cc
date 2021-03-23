/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>

#include "ORBVocabulary.h"
#include <System.h>
#include "Converter.h"
#include "ImuTypes.h"
#include "Optimizer.h"

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

int main(int argc, char **argv)
{
    if(argc != 5) {
        cerr << endl << "Usage: ./main path_to_vocabulary_folder path_to_settings path_to_dataset_folder output _path" << endl;
        return 1;
    }

    std::string vocabFolder = argv[1];
    std::string settingsFile = argv[2];
    std::string datasetFolder = argv[3];
    std::string outputFile = argv[4];

    std::string vocabBinPath = vocabFolder + "/ORBvoc.bin";
    if (!fileExists(vocabBinPath)) {
        // On first run, convert ORBvoc to a binary file for faster loading in future runs
        ORB_SLAM3::ORBVocabulary* voc = new ORB_SLAM3::ORBVocabulary();
        voc->loadFromTextFile(vocabFolder + "/ORBvoc.txt");
        voc->saveToBinaryFile(vocabBinPath);
    }

    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName) {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;
    int nImages;
    int nImu;
    int first_imu = 0;

    JsonlReader reader;
    ImuSync imuSync;
    imuSync.onSyncedLeader = [&vTimestampsImu, &vGyro, &vAcc](
        double time,
        double gx, double gy, double gz,
        double ax, double ay, double az) {
        vTimestampsImu.push_back(time);
        vGyro.push_back(cv::Point3f(gx, gy, gz));
        vAcc.push_back(cv::Point3f(ax, ay, az));
    };
    reader.onAccelerometer = [&imuSync](double time, double x, double y, double z) {
      imuSync.addFollower(time, x, y, z);
    };
    reader.onGyroscope = [&imuSync](double time, double x, double y, double z) {
      imuSync.addLeader(time, x, y, z);
    };
    reader.onFrames = [&vTimestampsCam](std::vector<JsonlReader::FrameParameters> frames) {
      assert(frames.size() > 0);
      vTimestampsCam.push_back(frames[0].time);
    };
    reader.read(datasetFolder + "/data.jsonl");

    nImages = vTimestampsCam.size();
    nImu = vTimestampsImu.size();

    std::cout << "Frames: " << nImages << ", Imu pairs: " << nImu << std::endl;

    if((nImages<=0) || (nImu<=0)) {
        cerr << "ERROR: Failed to load images or IMU" << endl;
        return 1;
    }

    // Find first imu to be considered, supposing imu measurements start first
    while(vTimestampsImu[first_imu] <= vTimestampsCam[0])
        first_imu++;
    first_imu--; // first imu measurement to be considered

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(
        vocabBinPath,
        settingsFile,
        ORB_SLAM3::System::IMU_STEREO,
        false // Viewer? Was true
    );

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

    std::vector<cv::VideoCapture> videoCaptures;
    std::string leftCamPath = findVideoSuffix(datasetFolder + "/data");
    std::string rightCamPath = findVideoSuffix(datasetFolder + "/data2");
    assert(!leftCamPath.empty() && "Video file missing");
    videoCaptures.push_back(cv::VideoCapture(leftCamPath));
    bool stereo = !rightCamPath.empty();
    std::cout << "Stereo: " << stereo << std::endl;
    if (stereo) videoCaptures.push_back(cv::VideoCapture(rightCamPath));

    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double t_rect = 0;
    int num_rect = 0;
    cv::Mat tm;
    for(int ni = 0; ni < nImages; ni++) {
        // Read left and right images from file
        videoCaptures[0].read(imLeft);
        if (stereo) videoCaptures[1].read(imRight);

        if(imLeft.empty()) {
            cerr << endl << "Failed to load left frame" << endl;
            return 1;
        }

        if(stereo && imRight.empty()) {
            cerr << endl << "Failed to load right frame" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Rect = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_Start_Rect = std::chrono::monotonic_clock::now();
#endif
        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        if (stereo) cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Rect = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_End_Rect = std::chrono::monotonic_clock::now();
#endif

        t_rect = std::chrono::duration_cast<std::chrono::duration<double> >(t_End_Rect - t_Start_Rect).count();
        double tframe = vTimestampsCam[ni];

        // Load imu measurements from previous frame
        vImuMeas.clear();

        if(ni > 0) {
            while (vTimestampsImu[first_imu] <= vTimestampsCam[ni]) {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                    vAcc[first_imu].x,
                    vAcc[first_imu].y,
                    vAcc[first_imu].z,
                    vGyro[first_imu].x,
                    vGyro[first_imu].y,
                    vGyro[first_imu].z,
                    vTimestampsImu[first_imu]
                ));
                first_imu++;
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        if (stereo)
            tm = SLAM.TrackStereo(imLeftRect, imRightRect, tframe, vImuMeas);
        else // TODO: monocular is untested
            tm = SLAM.TrackMonocular(imLeftRect, tframe, vImuMeas);

        // std::cout << "Pose: " << tm << std::endl;
        // Pose: [0.89174962, 0.44998422, -0.047925692, -3.7910242;
        //  0.078495137, -0.25811267, -0.96292061, 1.0970116;
        //  -0.44566926, 0.85492241, -0.26549345, -0.66183448;
        //  0, 0, 0, 1]
        // Pose is empty for first few frames, wait for valid transformation matrix
        if (tm.rows == 4 && tm.cols == 4) {
            cv::Mat Rwc = tm.rowRange(0,3).colRange(0,3).t(); // Rotation information
            cv::Mat twc = -Rwc*tm.rowRange(0,3).col(3); // Translation information
            vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);
            outputJson["time"] = tframe;
            outputJson["position"]["x"] = twc.at<float>(0, 0);
            outputJson["position"]["y"] = twc.at<float>(0, 1);
            outputJson["position"]["z"] = twc.at<float>(0, 2);
            outputJson["orientation"]["w"] = q[3]; // w == 3
            outputJson["orientation"]["x"] = q[0];
            outputJson["orientation"]["y"] = q[1];
            outputJson["orientation"]["z"] = q[2];
            os << outputJson.dump() << std::endl;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // TODO: Is this really necessary?
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsCam[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
    }

    // Stop all threads
    SLAM.Shutdown();

    // TODO: Add option to save this?
    // Save camera trajectory
    // if (bFileName)
    // {
    //     const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    //     const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    //     SLAM.SaveTrajectoryEuRoC(f_file);
    //     SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    // }
    // else
    // {
    //     SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    //     SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    // }

    return 0;
}
