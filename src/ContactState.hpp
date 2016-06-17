#ifndef ODOMETRY_CONTACTSTATE_HPP__
#define ODOMETRY_CONTACTSTATE_HPP__

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <vector>

namespace odometry
{

struct BodyContactPoint
{
    /** @brief position in body frame */
    base::Vector3d position;

    /** @brief contact probability in the interval between 0 and 1.0, or NaN if unknown */
    double contact;

    /** @brief slip distance of contact point */
    double slip;

    /** @brief contact group id, or -1 if not part of a group */
    int groupId;
};

struct BodyContactState
{
    /** @brief timestamp */
    base::Time time;
    
    /** @brief vector of all potential contact points of the body and their states.
     * 
     * The order of the contact points in the vector should not change over data items.
     */
    std::vector<BodyContactPoint> points;
};

}

#endif
