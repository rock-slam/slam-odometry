#ifndef __ODOMETRY_SAMPLING2D_HPP__
#define __ODOMETRY_SAMPLING2D_HPP__

#include <base/Pose.hpp>

namespace odometry
{
    /** 
     * base class of a 2d odometry model with an arbitrary error model, which
     * can be sampled from
     */
    class Sampling2D
    {
    public:
	virtual base::Pose2D getPoseDeltaSample2D() = 0;
    };
}

#endif
