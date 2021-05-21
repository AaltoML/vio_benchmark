#include <sophus/se3.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <basalt/calibration/calibration.hpp>
#include "dataset_io_jsonl.h"

void load_calibration(std::string calib_path, basalt::Calibration<double> &calib) {
  std::cout << "Loading standard calibration" << std::endl;
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
      calib.T_i_c.push_back(Sophus::SE3<double>(imuToCamera).inverse());
    }
    std::string name = camera["model"].get<std::string>();
    // TODO: Add missing camera models
    if (name == "doublesphere") {
      Eigen::Matrix<double, 6, 1> params; // fx, fy, cx, cy, xi, alpha
      params <<
        camera["focalLengthX"].get<double>(),
        camera["focalLengthY"].get<double>(),
        camera["principalPointX"].get<double>(),
        camera["principalPointY"].get<double>(),
        camera["xi"].get<double>(),
        camera["alpha"].get<double>();
      basalt::GenericCamera<double> genCamera;
      genCamera.variant = basalt::DoubleSphereCamera(params);
      calib.intrinsics.push_back(genCamera);
    } else if (name == "kannala-brandt4") {
      Eigen::Matrix<double, 8, 1> params; // fx, fy, cx, cy, kb0, kb1, kb2, kb3
      params <<
        camera["focalLengthX"].get<double>(),
        camera["focalLengthY"].get<double>(),
        camera["principalPointX"].get<double>(),
        camera["principalPointY"].get<double>(),
        camera["distortionCoefficients"][0].get<double>(),
        camera["distortionCoefficients"][1].get<double>(),
        camera["distortionCoefficients"][2].get<double>(),
        camera["distortionCoefficients"][3].get<double>();
      basalt::GenericCamera<double> genCamera;
      genCamera.variant = basalt::KannalaBrandtCamera4(params);
      calib.intrinsics.push_back(genCamera);
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
