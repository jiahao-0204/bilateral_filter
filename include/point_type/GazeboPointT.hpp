#pragma once
#include <pcl/point_types.h> // for using macros PCL_ADD_

struct GazeboPointT
{
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    PCL_MAKE_ALIGNED_OPERATOR_NEW;

    GazeboPointT()
    {
        x = NAN;
        y = NAN;
        z = NAN;
        intensity = 0;
        ring = 0;
    };
} EIGEN_ALIGN16;

// need this for pcl/conversion to work
POINT_CLOUD_REGISTER_POINT_STRUCT(GazeboPointT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
);