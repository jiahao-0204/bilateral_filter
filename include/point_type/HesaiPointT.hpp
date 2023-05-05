#pragma once
#include <pcl/point_types.h> // for using macros PCL_ADD_

struct HesaiPointT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    double timestamp;
    std::uint16_t ring;
    PCL_MAKE_ALIGNED_OPERATOR_NEW;

    HesaiPointT()
    {
        x = NAN;
        y = NAN;
        z = NAN;
        intensity = 0;
        timestamp = 0;
        ring = 0;
    };
} EIGEN_ALIGN16;

// need this for pcl/conversion to work
POINT_CLOUD_REGISTER_POINT_STRUCT(HesaiPointT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp, timestamp)
                                  (std::uint16_t, ring, ring)
);


// ================= NOTES =================
// intensity, timestamp, ring are specified as datatype = 7, 8, 4 in sensor_msgs
// document, which corresponds to FLOAT32, FLOAT64, UINT16.

// for pcl conversion to work, we need to register the variable using datatype
// with matching bits for correct memory allocation

// using 'double' instead of 'uint64_t' because pcl doesn't support 'uint64_t'
// ========================================