/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef DATASET_IO_JSONL_H
#define DATASET_IO_JSONL_H

#include <iostream>
#include <string>
#include <filesystem>
#include <basalt/io/dataset_io.h>
#include <basalt/imu/imu_types.h>
#include <basalt/utils/filesystem.h>

#include <opencv2/highgui/highgui.hpp>

#include "jsonl_reader.hpp"
#include "imu_sync.hpp"

constexpr int64_t SECONDS_TO_NS = 1e9;

namespace basalt {

std::string stripTrailingSlash(const std::string &s) {
  if (s.empty()) return s;
  return *s.rbegin() == '/' ? s.substr(0, s.size() - 1) : s;
}

std::string joinUnixPath(const std::string &path, const std::string &fn) {
  assert(path.size() > 0);
  return stripTrailingSlash(path) + "/" + fn;
}

std::string findVideoSuffix(std::string videoPathNoSuffix) {
  for (std::string suffix : { "mov", "avi", "mp4" }) {
    const auto path = videoPathNoSuffix + "." + suffix;
    std::ifstream testFile(videoPathNoSuffix + "." + suffix);
    if (testFile.is_open()) return path;
  }
  return "";
}

class JsonlVioDataset : public VioDataset {
  size_t num_cams;

  std::string path;

  bool videoInPngSeries; // When true, assumes video is stored in video/ folder as .png images
  std::string cam0;
  std::string cam1;

  std::vector<int64_t> image_timestamps;
  std::unordered_map<int64_t, std::vector<std::string>> image_paths;

  Eigen::aligned_vector<AccelData> accel_data;
  Eigen::aligned_vector<GyroData> gyro_data;
  Eigen::aligned_vector<ImuData::Ptr> imu_data;

  std::vector<int64_t> gt_timestamps;  // ordered gt timestamps
  Eigen::aligned_vector<Sophus::SE3d>
      gt_pose_data;  // TODO: change to eigen aligned

  int64_t mocap_to_imu_offset_ns = 0;

  std::vector<std::unordered_map<int64_t, double>> exposure_times;

  std::vector<cv::VideoCapture> videoCaptures;

 public:
  ~JsonlVioDataset(){};

  size_t get_num_cams() const { return num_cams; }

  std::vector<int64_t> &get_image_timestamps() { return image_timestamps; }

  const Eigen::aligned_vector<AccelData> &get_accel_data() const {
    return accel_data;
  }
  const Eigen::aligned_vector<GyroData> &get_gyro_data() const {
    return gyro_data;
  }
  const std::vector<int64_t> &get_gt_timestamps() const {
    return gt_timestamps;
  }
  const Eigen::aligned_vector<Sophus::SE3d> &get_gt_pose_data() const {
    return gt_pose_data;
  }

  const Eigen::aligned_vector<ImuData::Ptr> &get_imu_data() const {
      return imu_data;
  }

  int64_t get_mocap_to_imu_offset_ns() const { return mocap_to_imu_offset_ns; }

  size_t requestedImageIndex = 0;
  std::vector<ImageData> get_image_data(int64_t t_ns) {
    if (videoInPngSeries) {
      int64_t currentFrame = image_timestamps[requestedImageIndex++];
      (void)currentFrame;
      assert(t_ns == currentFrame && "get_image_data() must only be called once for each frame in video mode!");
    }
    std::vector<ImageData> res(num_cams);

    if (!videoInPngSeries && videoCaptures.size() == 0) {
      videoCaptures.push_back(cv::VideoCapture(cam0));
      assert(videoCaptures[0].isOpened() && "Failed to open cam0");
      if (!cam1.empty()) {
        videoCaptures.push_back(cv::VideoCapture(cam1));
        assert(videoCaptures[1].isOpened() && "Failed to open cam1");
      }
    }

    cv::Mat img;
    for (size_t i = 0; i < num_cams; i++) {
      if (videoInPngSeries) {
        img = cv::imread(image_paths[t_ns][i], cv::IMREAD_UNCHANGED);
      } else {
        videoCaptures[i].read(img);
      }

      if (img.type() == CV_8UC1) {
        res[i].img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));

        const uint8_t *data_in = img.ptr();
        uint16_t *data_out = res[i].img->ptr;

        size_t full_size = img.cols * img.rows;
        for (size_t i = 0; i < full_size; i++) {
          int val = data_in[i];
          val = val << 8;
          data_out[i] = val;
        }
      } else if (img.type() == CV_8UC3) {
        res[i].img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));

        const uint8_t *data_in = img.ptr();
        uint16_t *data_out = res[i].img->ptr;

        size_t full_size = img.cols * img.rows;
        for (size_t i = 0; i < full_size; i++) {
          int val = data_in[i * 3];
          val = val << 8;
          data_out[i] = val;
        }
      } else if (img.type() == CV_16UC1) {
        res[i].img.reset(new ManagedImage<uint16_t>(img.cols, img.rows));
        std::memcpy(res[i].img->ptr, img.ptr(),
                    img.cols * img.rows * sizeof(uint16_t));

      } else {
        std::cerr << "img.fmt.bpp " << img.type() << std::endl;
        std::abort();
      }

      // TODO: implement
      // auto exp_it = exposure_times[i].find(t_ns);
      // if (exp_it != exposure_times[i].end()) {
      //   res[i].exposure = exp_it->second;
      // }
    }
    return res;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class JsonlIO;
};

class JsonlIO : public DatasetIoInterface {
 public:
  bool allowImages;
  JsonlIO(bool allowImages) : allowImages(allowImages) {}

  bool isPng(std::string const &fullString) {
      if (fullString.length() >= 4) {
          return (0 == fullString.compare(fullString.length() - 4, 4, ".png"));
      } else {
          return false;
      }
  }

  void read(const std::string &path) {

    if (!fs::exists(path))
      std::cerr << "No dataset found in " << path << std::endl;

    data.reset(new JsonlVioDataset);

    data->cam0 = joinUnixPath(path, "video");
    data->cam1 = joinUnixPath(path, "video2");
    if (allowImages && fs::exists(data->cam0)) {
      std::cout << "Using png image series as video" << std::endl;
      data->videoInPngSeries = true;
      data->num_cams = 1;
      if (fs::exists(data->cam1))
        data->num_cams = 2;
    } else {
      std::cout << "Using video file" << std::endl;
      data->videoInPngSeries = false;
      data->cam0 = findVideoSuffix(joinUnixPath(path , "data"));
      data->cam1 = findVideoSuffix(joinUnixPath(path , "data2"));
      if (data->cam0.empty()) assert(false && "No video found");
      data->num_cams = data->cam1.empty() ? 1 : 2;
    }
    data->path = path;

    std::vector<std::string> images0;
    std::vector<std::string> images1;
    if (data->videoInPngSeries) {
        for (const auto & entry : fs::directory_iterator(data->cam0))
          if(isPng(entry.path())) images0.push_back(entry.path());
        sort(images0.begin(), images0.end());

        if (!data->cam1.empty()) {
          for (const auto & entry : fs::directory_iterator(data->cam1))
            if(isPng(entry.path())) images1.push_back(entry.path());
          sort(images1.begin(), images1.end());
        }
    }

    JsonlReader reader;
    ImuSync imuSync;

    imuSync.onSyncedLeader = [this](
      // TODO: Move this out? Now IMU data is stored twice
      double time,
      double gx, double gy, double gz,
      double ax, double ay, double az) {
      int64_t t_ns = time * SECONDS_TO_NS;
      basalt::ImuData::Ptr imu(new basalt::ImuData);
      imu->t_ns = t_ns;
      imu->accel = Eigen::Vector3d(ax, ay, az);
      imu->gyro = Eigen::Vector3d(gx, gy, gz);
      this->data->imu_data.push_back(imu);

      this->data->accel_data.emplace_back();
      this->data->accel_data.back().timestamp_ns = t_ns;
      this->data->accel_data.back().data = imu->accel;

      this->data->gyro_data.emplace_back();
      this->data->gyro_data.back().timestamp_ns = t_ns;
      this->data->gyro_data.back().data = imu->gyro;
    };

    reader.onAccelerometer = [&imuSync](double time, double x, double y, double z) {
      imuSync.addFollower(time, x, y, z);
    };

    reader.onGyroscope = [&imuSync](double time, double x, double y, double z) {
      imuSync.addLeader(time, x, y, z);
    };

    long index = 0;
    reader.onFrames = [this, &images0, &images1, &index](std::vector<JsonlReader::FrameParameters> frames) {
      assert(frames.size() > 0);
      int64_t t_ns = frames[0].time * SECONDS_TO_NS;
      this->data->image_timestamps.emplace_back(t_ns);

      if (this->data->videoInPngSeries) {
        assert(index < images0.size() && "More frames in JSONL than there are PNG images");
        std::vector<std::string> paths;
        paths.push_back(images0[index]);
        if (data->num_cams == 2)
          paths.push_back(images1[index]);
        this->data->image_paths.insert({t_ns, paths});
        index++;
      }
    };

    data->imu_data.clear();
    data->image_timestamps.clear();

    // std::cout << "Reading: " << path << std::endl;
    reader.read(joinUnixPath(path, "data.jsonl"));

    // TODO: Support ground truth
    // if (!load_mocap_as_gt &&
    //     fs::exists(path + "/mav0/state_groundtruth_estimate0/data.csv")) {
    //   read_gt_data_state(path + "/mav0/state_groundtruth_estimate0/");
    // } else if (!load_mocap_as_gt && fs::exists(path + "/mav0/gt/data.csv")) {
    //   read_gt_data_pose(path + "/mav0/gt/");
    // } else if (fs::exists(path + "/mav0/mocap0/data.csv")) {
    //   read_gt_data_pose(path + "/mav0/mocap0/");
    // }

    // TODO: Support exposure
    data->exposure_times.resize(data->num_cams);
    // if (fs::exists(path + "/mav0/cam0/exposure.csv")) {
    //   std::cout << "Loading exposure times for cam0" << std::endl;
    //   read_exposure(path + "/mav0/cam0/", data->exposure_times[0]);
    // }
    // if (fs::exists(path + "/mav0/cam1/exposure.csv")) {
    //   std::cout << "Loading exposure times for cam1" << std::endl;
    //   read_exposure(path + "/mav0/cam1/", data->exposure_times[1]);
    // }
  }

  void reset() { data.reset(); }

  VioDatasetPtr get_data() { return data; }

 private:
  // void read_exposure(const std::string &path,
  //                    std::unordered_map<int64_t, double> &exposure_data) {
  //   exposure_data.clear();

  //   std::ifstream f(path + "exposure.csv");
  //   std::string line;
  //   while (std::getline(f, line)) {
  //     if (line[0] == '#') continue;

  //     std::stringstream ss(line);

  //     char tmp;
  //     int64_t timestamp, exposure_int;
  //     Eigen::Vector3d gyro, accel;

  //     ss >> timestamp >> tmp >> exposure_int;

  //     exposure_data[timestamp] = exposure_int * 1e-9;
  //   }
  // }

  // void read_gt_data_state(const std::string &path) {
  //   data->gt_timestamps.clear();
  //   data->gt_pose_data.clear();

  //   std::ifstream f(path + "data.csv");
  //   std::string line;
  //   while (std::getline(f, line)) {
  //     if (line[0] == '#') continue;

  //     std::stringstream ss(line);

  //     char tmp;
  //     uint64_t timestamp;
  //     Eigen::Quaterniond q;
  //     Eigen::Vector3d pos, vel, accel_bias, gyro_bias;

  //     ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2] >>
  //         tmp >> q.w() >> tmp >> q.x() >> tmp >> q.y() >> tmp >> q.z() >> tmp >>
  //         vel[0] >> tmp >> vel[1] >> tmp >> vel[2] >> tmp >> accel_bias[0] >>
  //         tmp >> accel_bias[1] >> tmp >> accel_bias[2] >> tmp >> gyro_bias[0] >>
  //         tmp >> gyro_bias[1] >> tmp >> gyro_bias[2];

  //     data->gt_timestamps.emplace_back(timestamp);
  //     data->gt_pose_data.emplace_back(q, pos);
  //   }
  // }

  // void read_gt_data_pose(const std::string &path) {
  //   data->gt_timestamps.clear();
  //   data->gt_pose_data.clear();

  //   std::ifstream f(path + "data.csv");
  //   std::string line;
  //   while (std::getline(f, line)) {
  //     if (line[0] == '#') continue;

  //     std::stringstream ss(line);

  //     char tmp;
  //     uint64_t timestamp;
  //     Eigen::Quaterniond q;
  //     Eigen::Vector3d pos;

  //     ss >> timestamp >> tmp >> pos[0] >> tmp >> pos[1] >> tmp >> pos[2] >>
  //         tmp >> q.w() >> tmp >> q.x() >> tmp >> q.y() >> tmp >> q.z();

  //     data->gt_timestamps.emplace_back(timestamp);
  //     data->gt_pose_data.emplace_back(q, pos);
  //   }
  // }

  std::shared_ptr<JsonlVioDataset> data;
};  // namespace basalt

}  // namespace basalt

#endif  // DATASET_IO_H
