#ifndef JSONL_READER_H
#define JSONL_READER_H

#include <string>
#include <functional>
#include <vector>

class JsonlReader {
public:
    struct FrameParameters {
        double time = -1;
        double focalLengthX = -1;
        double focalLengthY = -1;
        double principalPointX = -1;
        double principalPointY = -1;
    };

    void read(std::string jsonlFilePath);

    std::function<void(double time, double x, double y, double z)> onGyroscope;
    std::function<void(double time, double x, double y, double z)> onAccelerometer;
    // TODO: Add support
    // std::function<void(double time, double x, double y, double z)> onGroundTruth;
    std::function<void(std::vector<FrameParameters>)> onFrames;
};

#endif // JSONL_READER_H
