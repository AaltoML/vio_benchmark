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

// Visualization variables
// std::unordered_map<int64_t, basalt::VioVisualizationData::Ptr> vis_map;

// tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> out_vis_queue;
tbb::concurrent_bounded_queue<basalt::PoseVelBiasState::Ptr> out_state_queue;

std::vector<int64_t> vio_t_ns;
Eigen::aligned_vector<Eigen::Vector3d> vio_t_w_i;
Eigen::aligned_vector<Sophus::SE3d> vio_T_w_i;

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

void load_calibration(std::string calib_path) {
  using json = nlohmann::json;
  std::ifstream ifs(calib_path);
  json config = json::parse(ifs);
  auto cameras = config["cameras"].get<json>();
  for (auto it = cameras.begin(); it != cameras.end(); ++it) {
    json &camera = *it;
    if (camera.find("imuToCamera") != camera.end()) {
      Eigen::Matrix4d imuToCamera;
      for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
          imuToCamera(r, c) = camera["imuToCamera"][r][c].get<double>();
      calib.T_i_c.push_back(Sophus::SE3<double>(imuToCamera));
    }
    if (camera.find("imuToCameraQuat") != camera.end()) {
      auto imu = camera["imuToCameraQuat"].get<json>();
      Eigen::Quaternion<double> quat(
          imu["qw"].get<double>(),
          imu["qx"].get<double>(),
          imu["qy"].get<double>(),
          imu["qz"].get<double>()
      );
      Sophus::Vector3<double> trans;
      trans <<
          imu["px"].get<double>(),
          imu["py"].get<double>(),
          imu["pz"].get<double>();
      calib.T_i_c.push_back(Sophus::SE3<double>(quat, trans));
    }
    if (camera.find("models") != camera.end()) {
      auto models = camera["models"].get<json>();
      for (auto mit = models.begin(); mit != models.end(); ++mit) {
        auto &model = *mit;
        std::string name = model["name"].get<std::string>();
        // TODO: Add missing camera models
        if (name == "ds") {
          Eigen::Matrix<double, 6, 1> params; // fx, fy, cx, cy, xi, alpha
          params <<
            model["focalLengthX"].get<double>(),
            model["focalLengthY"].get<double>(),
            model["principalPointX"].get<double>(),
            model["principalPointY"].get<double>(),
            model["xi"].get<double>(),
            model["alpha"].get<double>();
          basalt::GenericCamera<double> genCamera;
          genCamera.variant = basalt::DoubleSphereCamera(params);
          calib.intrinsics.push_back(genCamera);
        }
      }
    }
    if (camera.find("vignette") != camera.end()) {
       // TODO: These numbers shouldn't matter with vignette?
      basalt::RdSpline<1, 4, double> vignette(50000000000L, 0);
      auto values = camera["vignette"].get<json>();
      for (auto vit = values.begin(); vit != values.end(); ++vit) {
        Eigen::Matrix<double, 1, 1> m;
        m(0) = (*vit).get<double>();
        vignette.knots_push_back(m);
      }
      calib.vignette.push_back(vignette);
    }
  }

  if (config.find("gyroscope") != config.end()) {
    auto gyro = config["gyroscope"].get<json>();

    if (gyro.find("updateRate") != gyro.end())
      calib.imu_update_rate = gyro["updateRate"].get<double>();

    if (gyro.find("calibrationBias") != gyro.end())
      for (int i = 0; i < 12; i++)
        calib.calib_gyro_bias.getParam()[i] = gyro["calibrationBias"][i].get<double>();

    if (gyro.find("biasStd") != gyro.end())
          for (int i = 0; i < 3; i++)
            calib.gyro_bias_std(i) = gyro["biasStd"][i].get<double>();

    if (gyro.find("noiseStd") != gyro.end())
          for (int i = 0; i < 3; i++)
            calib.gyro_noise_std(i) = gyro["noiseStd"][i].get<double>();
  }

  if (config.find("accelerometer") != config.end()) {
    auto acc = config["accelerometer"].get<json>();

    if (acc.find("calibrationBias") != acc.end())
      for (int i = 0; i < 9; i++)
        calib.calib_accel_bias.getParam()[i] = acc["calibrationBias"][i].get<double>();

    if (acc.find("biasStd") != acc.end())
          for (int i = 0; i < 3; i++)
            calib.accel_bias_std(i) = acc["biasStd"][i].get<double>();

    if (acc.find("noiseStd") != acc.end())
          for (int i = 0; i < 3; i++)
            calib.accel_noise_std(i) = acc["noiseStd"][i].get<double>();
  }

  // std::ofstream os("./settings_test.json");
  // {
  //   cereal::JSONOutputArchive archive(os);
  //   archive(calib);
  // }
  // os.close();
}

void load_calibration_basalt(const std::string& calib_path) {
  std::ifstream os(calib_path, std::ios::binary);
  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(calib);
    for (size_t i = 0; i < calib.T_i_c.size(); i++) {
      std::cout << std::setprecision(18) << "T_i_c " << calib.T_i_c[i].matrix() << std::endl;
    }
    std::cout << "Loaded camera with " << calib.intrinsics.size() << " cameras"
              << std::endl;
  } else {
    std::cerr << "could not load camera calibration " << calib_path
              << std::endl;
    std::abort();
  }
}

// Feed functions
void feed_images() {
  std::cout << "Started input_data thread " << std::endl;

  for (size_t i = 0; i < vio_dataset->get_image_timestamps().size(); i++) {
    if (step_by_step) {
      std::unique_lock<std::mutex> lk(m);
      condition.wait(lk);
    }

    basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

    data->t_ns = vio_dataset->get_image_timestamps()[i];

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

  if (input_type == "euroc")
    load_calibration_basalt(cam_calib_path);
  else
    load_calibration(cam_calib_path);

  {
    basalt::DatasetIoInterfacePtr dataset_io =
      input_type == "euroc"
      ? basalt::DatasetIoFactory::getDatasetIo("euroc")
      : basalt::DatasetIoInterfacePtr(new basalt::JsonlIO(use_png));

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

  std::thread t1(&feed_images);
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
      Eigen::Vector3d vel_w_i = data->vel_w_i;
      Eigen::Vector3d bg = data->bias_gyro;
      Eigen::Vector3d ba = data->bias_accel;

      vio_t_ns.emplace_back(data->t_ns);
      vio_t_w_i.emplace_back(T_w_i.translation());
      vio_T_w_i.emplace_back(T_w_i);
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
        "zw": 0.0,
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      }
    })"_json;
    for (size_t i = 0; i < vio_t_ns.size(); i++) {
      const Sophus::SE3d& pose = vio_T_w_i[i];
      outputJson["time"] = vio_t_ns[i] * NS_TO_SECONDS;
      outputJson["position"]["x"] = pose.translation().x();
      outputJson["position"]["y"] = pose.translation().y();
      outputJson["position"]["z"] = pose.translation().z();
      outputJson["orientation"]["w"] = pose.unit_quaternion().w();
      outputJson["orientation"]["x"] = pose.unit_quaternion().x();
      outputJson["orientation"]["y"] = pose.unit_quaternion().y();
      outputJson["orientation"]["z"] = pose.unit_quaternion().z();
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
