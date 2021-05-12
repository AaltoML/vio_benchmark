#include <CLI/CLI.hpp>

#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <sophus/se3.hpp>

#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>

#include <basalt/utils/filesystem.h>

#include <nlohmann/json.hpp>

#include <sys/stat.h>

using json = nlohmann::json;

json se3ToJson(Sophus::SE3<double> se3) {
  auto tm = se3.matrix();
  json rows;
  for (int r = 0; r < tm.rows(); r++) {
    json row;
    for (int c = 0; c < tm.cols(); c++) {
      row.push_back(tm(r, c));
    }
    rows.push_back(row);
  }
  return rows;
}

int main(int argc, char **argv) {
  std::string calib_path;
  std::string output_path;

  CLI::App app{"Basalt calibration to standard calibration converter"};

  app.add_option("--calib-path", calib_path, "Path to Basalt calibration file")->required();
  app.add_option("--output-path", output_path, "Path to output file")->required();

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }

  if (!basalt::fs::exists(calib_path))
    std::cerr << "No file found " << calib_path << std::endl;

  basalt::Calibration<double> calib;
  std::ifstream is(calib_path, std::ios::binary);
  std::cout << std::setprecision(18);

  if (is.is_open()) {
    cereal::JSONInputArchive archive(is);
    archive(calib);
    for (size_t i = 0; i < calib.T_i_c.size(); i++) {
      std::cout << "T_i_c " << calib.T_i_c[i].matrix() << std::endl;
    }
    std::cout << "Loaded camera with " << calib.intrinsics.size() << " cameras"
              << std::endl;
  } else {
    std::cerr << "could not load camera calibration " << calib_path
              << std::endl;
    std::abort();
  }

  json calibJson;

  bool fisheye = false;
  for (size_t i = 0; i < calib.T_i_c.size(); i++) {
    auto params = calib.intrinsics[i].getParam();
    auto modelName = calib.intrinsics[i].getName();

    json cameraJson;

    if (modelName == "kb4") {
      fisheye = true;
      // fx, fy, cx, cy, k1, k2, k3, k4
      if (i == 0) {
        std::cout
          << "focalLengthX " << params(0) << ";" << std::endl
          << "focalLengthY " << params(1) << ";" << std::endl
          << "principalPointX " << params(2) << ";" << std::endl
          << "principalPointY " << params(3) << ";" << std::endl;
        std::cout
          << "distortionCoeffs ";
        for (int j = 4; j < 8; j++) {
          std::cout << params(j);
          if (j == 7) std::cout << ";" << std::endl;
          else std::cout << ",";
        }
      } else {
        std::cout
          << "secondFocalLengthX " << params(0) << ";" << std::endl
          << "secondFocalLengthY " << params(1) << ";" << std::endl
          << "secondPrincipalPointX " << params(2) << ";" << std::endl
          << "secondPrincipalPointY " << params(3) << ";" << std::endl;
        std::cout
          << "secondDistortionCoeffs ";
        for (int j = 4; j < 8; j++) {
          std::cout << params(j);
          if (j == 7) std::cout << ";" << std::endl;
          else std::cout << ",";
        }
      }
      cameraJson["model"] = "kannala-brandt4";
      cameraJson["focalLengthX"] = params(0);
      cameraJson["focalLengthY"] = params(1);
      cameraJson["principalPointX"] = params(2);
      cameraJson["principalPointY"] = params(3);
      cameraJson["distortionCoefficient"] = {params(4), params(5), params(6), params(7)};
    } else if (modelName == "ds") {
      cameraJson["model"] = "doublesphere";
      cameraJson["focalLengthX"] = params(0);
      cameraJson["focalLengthY"] = params(1);
      cameraJson["principalPointX"] = params(2);
      cameraJson["principalPointY"] = params(3);
      cameraJson["xi"] = params(4);
      cameraJson["alpha"] = params(5);
    }

    if (calib.vignette.size() > 0)
      for (size_t p = 0; p < calib.vignette[i].getKnots().size(); p++)
        cameraJson["vignette"].push_back(calib.vignette[i].getKnots()[p](0));

    if (i == 0)
      std::cout << "imuToCameraMatrix ";
    else
      std::cout << "secondImuToCameraMatrix ";
    auto tm = calib.T_i_c[i].inverse().matrix();
    for (int col = 0; col < tm.cols(); col++) {
      for (int row = 0; row < tm.rows(); row++) {
        std::cout << tm(row, col);
        if (row == 3 && col == 3) std::cout << ";" << std::endl;
        else std::cout << ",";
      }
    }
    cameraJson["imuToCamera"] = se3ToJson(calib.T_i_c[i].inverse());
    calibJson["cameras"].push_back(cameraJson);
  }
  if (fisheye)
    std::cout << "fisheyeCamera true;" << std::endl;

  json gyroJson;
  gyroJson["updateRate"] = calib.imu_update_rate;
  gyroJson["noiseStd"] = {calib.gyro_noise_std(0), calib.gyro_noise_std(1), calib.gyro_noise_std(2)};
  gyroJson["biasStd"] = {calib.gyro_bias_std(0), calib.gyro_bias_std(1), calib.gyro_bias_std(2)};
  for (int i = 0; i < 12; i++)
    gyroJson["calibrationBias"].push_back(calib.calib_gyro_bias.getParam()[i]);
  calibJson["gyroscope"] = gyroJson;

  json accJson;
  // accJson["updateRate"] = calib.imu_update_rate;
  accJson["noiseStd"] = {calib.accel_noise_std(0), calib.accel_noise_std(1), calib.accel_noise_std(2)};
  accJson["biasStd"] = {calib.accel_bias_std(0), calib.accel_bias_std(1), calib.accel_bias_std(2)};
  for (int i = 0; i < 9; i++)
    accJson["calibrationBias"].push_back(calib.calib_accel_bias.getParam()[i]);
  calibJson["accelerometer"] = accJson;

  std::ofstream os(output_path);
  os << calibJson.dump(4) << std::endl;

  return 0;
}
