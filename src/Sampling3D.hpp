#ifndef __ODOMETRY_SAMPLING3D_HPP__
#define __ODOMETRY_SAMPLING3D_HPP__

#include <base/Pose.hpp>

namespace odometry
{
    /** 
     * base class of a 3d odometry model with an arbitrary error model, which
     * can be sampled from
     */
    class Sampling3D
    {
    public:
    virtual ~Sampling3D(){};
	virtual base::Pose getPoseDeltaSample() = 0;
    };
}

#endif

