#ifndef VIO_TYPES_HPP
#define VIO_TYPES_HPP

// Common types, can be included "everywhere"

#include <functional>
#include "jsonl-recorder/types.hpp"

// forward declaration for accelerated::Image from accelerated-arrays
namespace accelerated { struct Image; }

namespace api {
using Vector3d = recorder::Vector3d;
using Quaternion = recorder::Quaternion;
using Pose = recorder::Pose;
using Image = accelerated::Image;

/**
 * A 3x3 matrix, row major (accessed as m[row][col]).
 * Also note that when the matrix is symmetric (like covariance matrices),
 * there is no difference between row-major and column-major orderings.
 */
using Matrix3d = std::array<std::array<double, 3>, 3>;

/** An element of the point cloud */
struct FeaturePoint {
    /**
     * An integer ID to identify same points in different
     * revisions of the point clouds
     */
    int id;

    /** Global position of the feature point */
    Vector3d position;

    /** Implementation-defined status/type */
    int status = 0;
};

using UncertaintyCallback = std::function<void(double t, const Vector3d &pos, const Vector3d &rot)>;

enum class TrackingStatus {
    INIT = 0, // Initial status when tracking starts and is still initializing
    TRACKING = 1, // When tracking is accurate
    LOST_TRACKING = 2 // When tracking fails after having achieved TRACKING state
};
}

#endif
