#ifndef __ODOMETRY_CONFIGURATION_HPP__
#define __ODOMETRY_CONFIGURATION_HPP__

#include <base/time.h>
#include <base/eigen.h>
	
namespace odometry {
struct TranslationWithYaw
{
    base::Vector3d translation;
    double yaw;

    TranslationWithYaw() {}
    TranslationWithYaw( const Eigen::Vector3d& v, double yaw );

    Eigen::Vector4d toVector4d() const;
};

struct Configuration
{
    Configuration();

    unsigned long seed;
    TranslationWithYaw constError;
    TranslationWithYaw distError;
    TranslationWithYaw tiltError;
    TranslationWithYaw dthetaError;
};
}

#endif
