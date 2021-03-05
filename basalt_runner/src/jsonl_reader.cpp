#include <fstream>
#include <nlohmann/json.hpp>
#include "jsonl_reader.hpp"
#include <iostream>

using json = nlohmann::json;

void JsonlReader::read(std::string jsonlFilePath) {
    std::ifstream dataFile;
    dataFile.open(jsonlFilePath);
    if (!dataFile.is_open()) {
        assert(false && "JSONL file not found");
    }

    double time;
    std::string line;
    std::array<double, 3> sensorValues;
    std::map<int, FrameParameters> frames;
    std::vector<FrameParameters> framesVec;
    // int framesInd;
    while (std::getline(dataFile, line)) {
        // std::cout << "parsing " << line << std::endl;
        json j = json::parse(line);
        if (j.find("sensor") != j.end()) {
            time = j["time"].get<double>();
            sensorValues = j["sensor"]["values"];
            std::string sensorType = j["sensor"]["type"];
            if (sensorType == "gyroscope") {
                if (onGyroscope) onGyroscope(time, sensorValues[0], sensorValues[1], sensorValues[2]);
            } else if (sensorType == "accelerometer") {
                if (onAccelerometer) onAccelerometer(time, sensorValues[0], sensorValues[1], sensorValues[2]);
            }
        } else if (onFrames && j.find("frames") != j.end()) {
            frames.clear();
            time = j["time"].get<double>();
            json jFrames = j["frames"];
            for (json::iterator jFrame = jFrames.begin(); jFrame != jFrames.end(); ++jFrame) {
                FrameParameters frame = {
                    .time = time
                };
                if (!(*jFrame)["cameraParameters"].is_null()) {
#define X(FIELD) \
                    if (!(*jFrame)["cameraParameters"][#FIELD].is_null()) { \
                        frame.FIELD = (*jFrame)["cameraParameters"][#FIELD].get<double>(); \
                    }
                    X(focalLengthX)
                    X(focalLengthY)
                    X(principalPointX)
                    X(principalPointY)
#undef X
                    bool hasDirFocal = frame.focalLengthX > 0.0 && frame.focalLengthY > 0.0;
                    if (!hasDirFocal && !(*jFrame)["cameraParameters"]["focalLength"].is_null()) {
                        double focalLength = (*jFrame)["cameraParameters"]["focalLength"].get<double>();
                        frame.focalLengthX = focalLength;
                        frame.focalLengthY = focalLength;
                    }
                }
                int cameraInd = (*jFrame)["cameraInd"].get<int>();
                // Use map to allow any order of cameraInds in the JSON array.
                frames.insert({cameraInd, frame});
            }
            if (!frames.empty()) {
                size_t n = frames.size();
                framesVec.clear();
                // Assumes the keys of `this->frames` are successive without gaps and start from zero.
                for (size_t i = 0; i < n; ++i) {
                    framesVec.push_back(frames.at(i));
                }
                onFrames(framesVec);
            }
        }
    }
}
