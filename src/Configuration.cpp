#include "Configuration.hpp"

using namespace odometry;

TranslationWithYaw::TranslationWithYaw( const Eigen::Vector3d& v, double yaw )
	: translation( v ), yaw( yaw ) 
{}

Eigen::Vector4d TranslationWithYaw::toVector4d() const
{
    Eigen::Vector4d res;
    res << translation, yaw;
    return res;
}

Configuration::Configuration()
    : seed(42u),
    constError( base::Vector3d( 0.0, 0.0, 0.0), 0 ),
    distError( base::Vector3d( 0.0, 0.0, 0), 0 ),
    tiltError( base::Vector3d( 0.0, 0.0, 0), 0 ),
    dthetaError( base::Vector3d( 0.0, 0.0, 0), 0 ),
    useZeroVelocity( true )
    /*constError( base::Vector3d( 0.002, 0.005, 0.001), 1e-4 ),
    distError( base::Vector3d( 0.1, 0.5, 0), 0 ),
    tiltError( base::Vector3d( 0.1, 0.5, 0), 0 ),
    dthetaError( base::Vector3d( 0.2, 0.0, 0), 0 ),
    useZeroVelocity( true )*/
{}
