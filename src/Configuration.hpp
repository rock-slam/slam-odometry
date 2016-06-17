#ifndef __ODOMETRY_CONFIGURATION_HPP__
#define __ODOMETRY_CONFIGURATION_HPP__

#include <base/Time.hpp>
#include <base/Eigen.hpp>
	
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

    /** 
     * set this to true, if the odometry should look 
     * for zero velocity updates, in which orientation
     * changes from the imu are ignored.
     */
    bool useZeroVelocity;
};
}

#endif
