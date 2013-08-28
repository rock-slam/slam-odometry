#ifndef __ODOMETRY_GAUSSIAN2D_HPP__
#define __ODOMETRY_GAUSSIAN2D_HPP__

#include <Eigen/Core>
#include <base/Pose.hpp>

namespace odometry
{
    class Gaussian2D
    {
    public:
	virtual base::Pose2D getPoseDelta2D() = 0;
	virtual Eigen::Matrix2d getPositionError2D() = 0;
	virtual double getOrientationError2D() = 0;
    };
}

#endif

