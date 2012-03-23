#ifndef __ASGUARDCONFIGURATION_HPP__
#define __ASGUARDCONFIGURATION_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <cmath>
#include <vector>

#include <base/time.h>
#include <base/eigen.h>

#include "BodyState.hpp"

//    /** 
//     * configuration class for the asguard robot. 
//     *
//     * this class contains static configuration parameters that are directly
//     * related to the asguard system. 
//     *
//     * Body origin is the centre of the front wheel axle. 
//     * The coordinates are defined as follows:
//     *
//     * x - right direction looking at the robot from the top
//     * y - forward direction
//     * z - positive upward direction
//     */
//    struct Configuration
//    {
//	Configuration() :
//	    feetPerWheel(5),
//	    wheelRadiusMax(0.19), 
//	    wheelRadiusAvg(0.178),
//	    angleBetweenLegs(2.0*M_PI/5.0),
//	    trackWidth(0.515),
//	    wheelBase(0.510),
//	    sensorHeadAngle(-40/180.0*M_PI),
//	    overallWeight(114.73)
//	{}
//
//	/** number of feet per wheel */
//	unsigned int feetPerWheel;
//	/** length of the individual spokes, from the center of rotation to the
//	 * foot contact point. */
//	double wheelRadiusMax;
//	/** avg wheel radius for odometry calculations */
//	double wheelRadiusAvg;
//	/** angle between two consecutive legs */
//	double angleBetweenLegs;
//	/** distance between left and right wheel */
//	double trackWidth;
//	/** distance between front and rear axle */
//	double wheelBase;
//	/** angle of the sensorhead with respect to the main body */
//	double sensorHeadAngle;
//	/** weight of Asguard (average for big/small battery) [N]*/
//	double overallWeight;
//
//	/** body to IMU fixed coordinate system */
//	base::Quaterniond R_b2i;
//	/** body to GPS fixed coordinate system */
//	base::Quaterniond R_b2g;
//	/** body to LaserScan fixed coordinate system */
//	base::Quaterniond R_b2ls;
//
//	/** xsens world to world transformation */
//	base::Quaterniond R_xw2w;
//
//	/** return the position of the foot specified by 
//	 * @param wheel_idx and 
//	 * @param foot_idx 
//	 * in body referenced coordinates.
//	 */
//	Eigen::Vector3d getFootPosition(const odometry::BodyState& state, odometry::wheelIdx wheel_idx, unsigned int foot_idx) const;
//    };
	
    namespace odometry {
    struct TranslationWithYaw
    {
	base::Vector3d translation;
	double yaw;

	TranslationWithYaw() {}
	TranslationWithYaw( const Eigen::Vector3d& v, double yaw )
	    : translation( v ), yaw( yaw ) {}

	Eigen::Vector4d toVector4d() const
	{
	    Eigen::Vector4d res;
	    res << translation, yaw;
	    return res;
	}
    };

    struct Configuration
    {
	Configuration()
	    : seed(42u),
	    constError( base::Vector3d( 0.002, 0.005, 0.001), 1e-4 ),
	    distError( base::Vector3d( 0.1, 0.5, 0), 0 ),
	    tiltError( base::Vector3d( 0.1, 0.5, 0), 0 ),
	    dthetaError( base::Vector3d( 0.2, 0.0, 0), 0 )
	{}

	unsigned long seed;
	TranslationWithYaw constError;
	TranslationWithYaw distError;
	TranslationWithYaw tiltError;
	TranslationWithYaw dthetaError;
    };
    }

#endif
