#ifndef INCLUDE_POINT_TYPES_H_
#define INCLUDE_POINT_TYPES_H_

#pragma once
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>

struct ConePoint {
    PCL_ADD_POINT4D;
    float   covariance[4];
    uint8_t color;  // Colors defined in ConeType
                    // (BLUE=0, YELLOW=1, ORANGE_BIG=2, ORANGE_SMALL=3, UNKNOWN=4)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(ConePoint, (float, x, x)(float, y, y)(float, z, z)(float[4], covariance, covariance)(uint8_t, color, color))

namespace hesai {

struct PointXYZITRB;
struct PointXYZIT {
    PCL_ADD_POINT4D
    float    intensity;
    double   timestamp;
    uint16_t ring;                   ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned

        operator PointXYZITRB() const;

} EIGEN_ALIGN16;

struct PointXYZITRB {
    PCL_ADD_POINT4D
    float    intensity;
    double   timestamp;
    uint16_t ring;  ///< laser ring number
    double   range;
    float    bearing;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned

        operator PointXYZIT() const;
} EIGEN_ALIGN16;

inline PointXYZITRB::operator PointXYZIT() const {
    PointXYZIT p;
    p.x         = x;
    p.y         = y;
    p.z         = z;
    p.intensity = intensity;
    p.timestamp = timestamp;
    p.ring      = ring;
    return p;
}

inline PointXYZIT::operator PointXYZITRB() const {
    PointXYZITRB p;
    p.x         = x;
    p.y         = y;
    p.z         = z;
    p.intensity = intensity;
    p.timestamp = timestamp;
    p.ring      = ring;
    p.range     = std::hypot(x, y);
    p.bearing   = std::atan2(y, x);
    return p;
}

}  // namespace hesai

POINT_CLOUD_REGISTER_POINT_STRUCT(hesai::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai::PointXYZITRB, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(
                                                           uint16_t, ring, ring)(double, range, range)(float, bearing, bearing))
#endif  // INCLUDE_POINT_TYPES_H_
