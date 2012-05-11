#ifndef CONTACT_ODODOMETRY_HPP__
#define CONTACT_ODODOMETRY_HPP__

#include <eslam/ContactModel.hpp>
#include <asguard/Configuration.hpp>
#include <asguard/Odometry.hpp>
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
    FootContact(const asguard::odometry::Configuration& config);
    void update(const eslam::BodyContactState& state, const Eigen::Quaterniond& orientation);

    base::Pose getPoseDelta();
    Eigen::Matrix3d getPositionError();
    Eigen::Matrix3d getOrientationError();
    Matrix6d getPoseError();

public:
    base::Pose getPoseDeltaSample();
    base::Pose2D getPoseDeltaSample2D();

    Eigen::Quaterniond orientation, prevOrientation;
    base::odometry::State<eslam::BodyContactState> state;

private:
    /** Odometry configuration */
    asguard::odometry::Configuration config;

    asguard::odometry::GaussianSamplingPose3D sampling;
};

}

#endif
