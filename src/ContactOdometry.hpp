#ifndef CONTACT_ODODOMETRY_HPP__
#define CONTACT_ODODOMETRY_HPP__

//#include <eslam/ContactModel.hpp>
#include <odometry/ContactState.hpp>
#include <odometry/Configuration.hpp>
#include <odometry/Odometry.hpp>
#include <base/odometry.h>

namespace odometry
{
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,6,1> Vector6d;

class FootContact : 
    public base::odometry::Gaussian3D,
    public base::odometry::Sampling3D,
    public base::odometry::Sampling2D
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FootContact(const odometry::Configuration& config);
    void update(const BodyContactState& state, const Eigen::Quaterniond& orientation);

    base::Pose getPoseDelta();
    Eigen::Matrix3d getPositionError();
    Eigen::Matrix3d getOrientationError();
    Matrix6d getPoseError();

public:
    base::Pose getPoseDeltaSample();
    base::Pose2D getPoseDeltaSample2D();

    Eigen::Quaterniond orientation, prevOrientation;
    base::odometry::State<BodyContactState> state;

private:
    /** Odometry configuration */
    odometry::Configuration config;

    odometry::GaussianSamplingPose3D sampling;
};

}

#endif
