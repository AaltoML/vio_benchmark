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

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>

#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>
#include <tbb/global_control.h>

#include <CLI/CLI.hpp>

#include <basalt/io/dataset_io.h>
#include <basalt/io/marg_data_io.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <basalt/calibration/calibration.hpp>

#include <basalt/serialization/headers_serialization.h>

// #include <basalt/utils/vis_utils.h>
#include <Eigen/Dense>
#include <basalt/utils/sophus_utils.hpp>

#include "dataset_io_jsonl.h"
#include <nlohmann/json.hpp>
#include "calibration_loader.h"

// Visualization variables
// std::unordered_map<int64_t, basalt::VioVisualizationData::Ptr> vis_map;

// tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue;
tbb::concurrent_bounded_queue<basalt::PoseVelBiasState::Ptr> out_state_queue;

std::vector<int64_t> vio_t_ns;
Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;
Eigen::aligned_vector<Sophus::SE3d> vio_T_w_i;
Eigen::aligned_vector<Eigen::Vector3d> vio_bg;
Eigen::aligned_vector<Eigen::Vector3d> vio_ba;

std::vector<int64_t> gt_t_ns;
Eigen::aligned_vector<Eigen::Vector3d> gt_t_w_i;

std::string marg_data_path;
size_t last_frame_processed = 0;

tbb::concurrent_unordered_map<int64_t, int, std::hash<int64_t>> timestamp_to_id;

std::mutex m;
std::condition_variable condition;
bool step_by_step = false;

// VIO variables
basalt::Calibration<double> calib;

basalt::VioDatasetPtr vio_dataset;
basalt::VioConfig vio_config;
basalt::OpticalFlowBase::Ptr opt_flow_ptr;
basalt::VioEstimatorBase::Ptr vio;


// Feed functions
void feed_images(int64_t tMax) {
  std::cout << "Started input_data thread " << std::endl;

  for (size_t i = 0; i < vio_dataset->get_image_timestamps().size(); i++) {
    if (step_by_step) {
      std::unique_lock<std::mutex> lk(m);
      condition.wait(lk);
    }

    basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

    data->t_ns = vio_dataset->get_image_timestamps()[i];
    // Hack fix to Basalt crashing if IMU data ends before camera data, as may
    // be case eg with simulated IMU data.
    if (data->t_ns >= tMax) {
      break;
    }

    data->img_data = vio_dataset->get_image_data(data->t_ns);

    timestamp_to_id[data->t_ns] = i;

    opt_flow_ptr->input_queue.push(data);
  }

  // Indicate the end of the sequence
  opt_flow_ptr->input_queue.push(nullptr);

  std::cout << "Finished input_data thread " << std::endl;
}

void feed_imu(std::string inputType) {
  // std::ofstream os("imu_samples.txt");
  if (inputType == "euroc") {
    for (size_t i = 0; i < vio_dataset->get_gyro_data().size(); i++) {
      basalt::ImuData::Ptr imu(new basalt::ImuData);
      imu->t_ns = vio_dataset->get_gyro_data()[i].timestamp_ns;
      imu->accel = vio_dataset->get_accel_data()[i].data;
      imu->gyro = vio_dataset->get_gyro_data()[i].data;
      // os << std::setprecision(18) << imu->t_ns << imu->accel << imu->gyro << std::endl;
      vio->imu_data_queue.push(imu);
    }
  } else {
    basalt::JsonlVioDataset* jsonlDataSet = dynamic_cast<basalt::JsonlVioDataset*>(vio_dataset.get());
    for (size_t i = 0; i < jsonlDataSet->get_imu_data().size(); i++) {
      basalt::ImuData::Ptr imu = jsonlDataSet->get_imu_data()[i];
      // os << std::setprecision(18) << imu->t_ns << imu->accel << imu->gyro << std::endl;
      vio->imu_data_queue.push(imu);
    }
  }
  vio->imu_data_queue.push(nullptr);

  // os.close();
}

int main(int argc, char** argv) {
  bool print_queue = false;
  bool terminate = false;
  std::string cam_calib_path;
  std::string dataset_path;
  std::string config_path;
  std::string result_path;
  std::string trajectory_fmt;
  std::string input_type;
  bool use_png = false;
  int num_threads = 0;
  bool use_imu = true;

  CLI::App app{"App description"};

  app.add_option("--cam-calib", cam_calib_path,
                 "Ground-truth camera calibration used for simulation.")
      ->required();

  app.add_option("--dataset-path", dataset_path, "Path to dataset.")
      ->required();

  app.add_option("--marg-data", marg_data_path,
                 "Path to folder where marginalization data will be stored.");

  app.add_option("--use-png", use_png, "Allows usage of PNG images as video if they exist under video/ folder");
  app.add_option("--input-type", input_type,
                 "Input data format: jsonl, euroc");

  app.add_option("--print-queue", print_queue, "Print queue.");
  app.add_option("--config-path", config_path, "Path to config file.");
  app.add_option("--result-path", result_path,
                 "Path to result file where the system will write RMSE ATE.");
  app.add_option("--num-threads", num_threads, "Number of threads.");
  app.add_option("--step-by-step", step_by_step, "Path to config file.");
  app.add_option("--save-trajectory", trajectory_fmt,
                 "Save trajectory. Supported formats <tum, euroc, kitti>");
  app.add_option("--use-imu", use_imu, "Use IMU.");

  // global thread limit is in effect until global_control object is destroyed
  std::unique_ptr<tbb::global_control> tbb_global_control;
  if (num_threads > 0) {
    tbb_global_control = std::make_unique<tbb::global_control>(
        tbb::global_control::max_allowed_parallelism, num_threads);
  }

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  if (!config_path.empty()) {
    vio_config.load(config_path);

    if (vio_config.vio_enforce_realtime) {
      vio_config.vio_enforce_realtime = false;
      std::cout
          << "The option vio_config.vio_enforce_realtime was enabled, "
             "but it should only be used with the live executables (supply "
             "images at a constant framerate). This executable runs on the "
             "datasets and processes images as fast as it can, so the option "
             "will be disabled. "
          << std::endl;
    }
  }

  load_calibration(cam_calib_path, calib);
  double t0 = 0.0;
  {
    basalt::DatasetIoInterfacePtr dataset_io;
    if (input_type == "euroc") {
      dataset_io = basalt::DatasetIoFactory::getDatasetIo("euroc");
    } else {
      t0 = basalt::JsonlVioDataset::get_t0(dataset_path);
      dataset_io = basalt::DatasetIoInterfacePtr(new basalt::JsonlIO(use_png, false, t0));
    }
    
    dataset_io->read(dataset_path);

    vio_dataset = dataset_io->get_data();

    assert(vio_dataset->get_num_cams() == 2 && "Only stereo supported right now, this might change in future");

    opt_flow_ptr =
        basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);

    // TODO
    // for (size_t i = 0; i < vio_dataset->get_gt_pose_data().size(); i++) {
    //   gt_t_ns.push_back(vio_dataset->get_gt_timestamps()[i]);
    //   gt_t_w_i.push_back(vio_dataset->get_gt_pose_data()[i].translation());
    // }
  }

  int64_t tMax = std::numeric_limits<int64_t>::max();
  if (input_type != "euroc") {
    basalt::JsonlVioDataset* jsonlDataSet = dynamic_cast<basalt::JsonlVioDataset*>(vio_dataset.get());
    tMax = jsonlDataSet->get_imu_data().back()->t_ns;
  }

  // const int64_t start_t_ns = vio_dataset->get_image_timestamps().front();
  {
    vio = basalt::VioEstimatorFactory::getVioEstimator(
        vio_config, calib, basalt::constants::g, use_imu);
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    opt_flow_ptr->output_queue = &vio->vision_data_queue;
    vio->out_state_queue = &out_state_queue;
  }

  basalt::MargDataSaver::Ptr marg_data_saver;

  if (!marg_data_path.empty()) {
    marg_data_saver.reset(new basalt::MargDataSaver(marg_data_path));
    vio->out_marg_queue = &marg_data_saver->in_marg_queue;

    // Save gt.
    {
      std::string p = marg_data_path + "/gt.cereal";
      std::ofstream os(p, std::ios::binary);

      {
        cereal::BinaryOutputArchive archive(os);
        archive(gt_t_ns);
        archive(gt_t_w_i);
      }
      os.close();
    }
  }

  std::thread t1(&feed_images, tMax);
  std::thread t2(&feed_imu, input_type);

  std::shared_ptr<std::thread> t3;

  std::thread t4([&]() {
    basalt::PoseVelBiasState::Ptr data;

    while (true) {
      out_state_queue.pop(data);

      if (!data.get()) break;

      // int64_t t_ns = data->t_ns;
      // std::cerr << "t_ns " << t_ns << std::endl;
      Sophus::SE3d T_w_i = data->T_w_i;
      // Eigen::Vector3d vel_w_i = data->vel_w_i;
      Eigen::Vector3d bg = data->bias_gyro;
      Eigen::Vector3d ba = data->bias_accel;

      vio_t_ns.emplace_back(data->t_ns);
      vio_t_w_i.emplace_back(T_w_i.translation());
      vio_T_w_i.emplace_back(T_w_i);
      vio_bg.emplace_back(bg);
      vio_ba.emplace_back(ba);
    }

    std::cout << "Finished t4" << std::endl;
  });

  std::shared_ptr<std::thread> t5;

  if (print_queue) {
    t5.reset(new std::thread([&]() {
      while (!terminate) {
        std::cout << "opt_flow_ptr->input_queue "
                  << opt_flow_ptr->input_queue.size()
                  << " opt_flow_ptr->output_queue "
                  << opt_flow_ptr->output_queue->size() << " out_state_queue "
                  << out_state_queue.size() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }));
  }

  auto time_start = std::chrono::high_resolution_clock::now();

  terminate = true;

  t1.join();
  t2.join();
  if (t3.get()) t3->join();
  t4.join();
  if (t5.get()) t5->join();

  auto time_end = std::chrono::high_resolution_clock::now();

  if (!trajectory_fmt.empty()) {
    std::cout << "Saving trajectory..." << std::endl;
    std::ofstream os(trajectory_fmt);
    constexpr double NS_TO_SECONDS = 1e-9;
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
      },
      "biasMean": {
        "gyroscopeAdditive": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.0
        },
        "accelerometerAdditive": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.0
        }
      }
    })"_json;
    for (size_t i = 0; i < vio_t_ns.size(); i++) {
      const Sophus::SE3d& pose = vio_T_w_i[i];
      outputJson["time"] = vio_t_ns[i] * NS_TO_SECONDS + t0;
      outputJson["position"]["x"] = pose.translation().x();
      outputJson["position"]["y"] = pose.translation().y();
      outputJson["position"]["z"] = pose.translation().z();
      outputJson["orientation"]["w"] = pose.unit_quaternion().w();
      outputJson["orientation"]["x"] = pose.unit_quaternion().x();
      outputJson["orientation"]["y"] = pose.unit_quaternion().y();
      outputJson["orientation"]["z"] = pose.unit_quaternion().z();
      outputJson["biasMean"]["gyroscopeAdditive"]["x"] = vio_bg[i](0);
      outputJson["biasMean"]["gyroscopeAdditive"]["y"] = vio_bg[i](1);
      outputJson["biasMean"]["gyroscopeAdditive"]["z"] = vio_bg[i](2);
      outputJson["biasMean"]["accelerometerAdditive"]["x"] = vio_ba[i](0);
      outputJson["biasMean"]["accelerometerAdditive"]["y"] = vio_ba[i](1);
      outputJson["biasMean"]["accelerometerAdditive"]["z"] = vio_ba[i](2);
      os << outputJson.dump() << std::endl;
    }
    std::cout << "Saved trajectory to " << trajectory_fmt << std::endl;
  }

  if (!result_path.empty()) {
    double error = basalt::alignSVD(vio_t_ns, vio_t_w_i, gt_t_ns, gt_t_w_i);

    auto exec_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_end - time_start);

    std::ofstream os(result_path);
    {
      cereal::JSONOutputArchive ar(os);
      ar(cereal::make_nvp("rms_ate", error));
      ar(cereal::make_nvp("num_frames",
                          vio_dataset->get_image_timestamps().size()));
      ar(cereal::make_nvp("exec_time_ns", exec_time_ns.count()));
    }
    os.close();
  }

  return 0;
}
