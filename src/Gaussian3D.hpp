#ifndef __ODOMETRY_GAUSSIAN3D_HPP__
#define __ODOMETRY_GAUSSIAN3D_HPP__

#include <Eigen/Core>
#include <base/Pose.hpp>

namespace odometry
{
    /** 
     * base class of a 3d odometry model with a gaussian error model
     */
    class Gaussian3D
    {
    public:
	/**
	 * return the pose delta in the body fixed frame of the previous
	 * state. 
	 */
	virtual base::Pose getPoseDelta() = 0;
	
	/**
	 * returns the covariance matrix of the linear velocity 
	 */
	virtual Eigen::Matrix3d getPositionError() = 0;

	/**
	 * returns the covariance matrix of the orientation error
	 * as an axis angle vector on a manifold. 
	 */
	virtual Eigen::Matrix3d getOrientationError() = 0;

        base::Matrix6d getPoseError()
        {
            base::Matrix6d cov;
            cov << 
                getOrientationError(), Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Zero(), getPositionError();

            return cov; 
        }

    };
}

#endif

