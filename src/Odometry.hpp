#ifndef __ASGUARDODOMETRY_HPP__
#define __ASGUARDODOMETRY_HPP__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <base/time.h>
#include <base/pose.h>
#include <base/odometry.h>

#include "Configuration.hpp"

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

//namespace asguard
//{
namespace odometry
{
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    typedef Eigen::Matrix<double,6,1> Vector6d;

    base::Pose getPoseFromVector6d( const Vector6d &v );
    Vector6d getVector6dFromPose( const base::Pose& pose );
    base::Pose2D projectPoseDelta( const Eigen::Quaterniond& orientation, const base::Pose& pose );

    /** 
     * @brief helper class that represents a pose (position and orientation)
     * and associated gaussian uncertainty, which can also produce samples
     */
    class GaussianSamplingPose3D
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**
	 * @param odometry configuration struct
	 * @todo not actually used at this point
	 */
	GaussianSamplingPose3D( const Configuration& config );
	/**
	 * @brief set mean and upper triangular matrix of covariance 
	 *
	 * The pose is encoded as a 6 vector, where the first three
	 * elements are the axis/angle notation of rotation, and the last 3
	 * elements are translation
	 *
	 * @param mean mean of pose change
	 * @param llt upper triangular matrix of covariance (through e.g.
	 *        cholesky from cov)
	 */ 
	void update( const Vector6d& mean, const Matrix6d& llt );
	/**
	 * @brief get a pose sample based on mean and covariance provided by update
	 */ 
	Vector6d sample();

    public:
	Matrix6d poseCov, poseLT;
	Vector6d poseMean;

	/** constant model error that is added to the dynamic calculation of the error */
	Matrix6d modelError;

    protected:
	Configuration config;

	/** random number generator */
	boost::minstd_rand rand_gen;
	boost::variate_generator<boost::minstd_rand&, boost::normal_distribution<> > rand_norm;
    };

    /** @brief Class which provides a simple wheel (skid steering) odometry
     * from asguard
     *
     * class is based on a simple skid steering model, where the change in
     * orientation is provided by an IMU, so effectively only the translation
     * comes from a change in wheel position.
     *
     * @note currently this class is specific to asguard, but can be made generic
     * quite easily.
     */
    class Skid4Odometry
	: public base::odometry::Gaussian3D,
	  public base::odometry::Sampling3D,
	  public base::odometry::Sampling2D
    {
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/** 
	 * @brief default constructor for wheel odometry class
	 *
	 * @param config odometry configuration class, mainly setting the error matrix
	 * @param asguardConfig configuration of the asguard body model
	 */

	Skid4Odometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase);
	/** 
	 * @brief update method which is called for each new measurement
	 *
	 * @param bs encodes the change in wheel positions of the asgaurd
	 * @param orientation body to world rotation provided by an IMU
	 */
	void update(const odometry::BodyState &bs, const Eigen::Quaterniond& orientation);

	/** 
	 * @brief provides the mean of the change in pose between the last two
	 *	  update calls
	 *
	 * @result is a body to previous body transform 
	 */
	base::Pose getPoseDelta();
	/**
	 * @brief Provide the error covariance matrix for the current pose delta
	 *
	 * @result position error covariance as 3 matrix
	 */
	Eigen::Matrix3d getPositionError();
	/**
	 * @brief provide the orientation covariance matrix for the current
	 *	  pose delta
	 *
	 * @result orientation error covariance as 3 matrix as axis/angle of
	 *	   rotation
	 */
	Eigen::Matrix3d getOrientationError();

	/**
	 * @brief provide the error covariance matrix for the current pose delta
	 *
	 * @result error covariance as 6 matrix (see @class
	 *	   GaussianSamplingPose3D for explanation)
	 */
	Matrix6d getPoseError();

	/**
	 * @brief flag if the rotation should be performed around the body
	 *        center
	 *
	 * @param compensate if set to true, rotation will be performed around
	 * the geometric center of asguard instead of using the body origin.
	 */
	void setBodyCenterCompensation(bool compensate);

    public:
	/**
	 * @brief get a 3d pose sample based on the current state of the
	 *	  odometry
	 *
	 * Provides a sample of the pose based on the last two update calls.
	 * This is effectively the PoseDelta perturbed by gaussian noise based
	 * on the poseError.
	 *
	 * @result a full body to previous body transform
	 */
	base::Pose getPoseDeltaSample();

	/**
	 * @brief provide a 2d pose sample based on the current state of the
	 * odometry
	 *
	 * Works like the 3d pose sample, but performs a flatting onto a 2d pose.
	 *
	 * @result a body to previous body transform in 2d
	 */
	base::Pose2D getPoseDeltaSample2D();

	// DEPRECATED, PLEASE REMOVE.
	// only use the interface defined in the base class.
	// if interface not sufficient, discuss.
	Eigen::Vector3d getTranslation();
	Eigen::Vector3d getVelocity();
	Eigen::Vector3d getAngularVelocity();
	Eigen::Matrix3d getVelocityError();
	double getDeltaYaw();

    public:
	/** helper member to store the current and previous bodystates
	 */
	base::odometry::State<odometry::BodyState> state;

    private:
	/** Odometry configuration */
	Configuration config;
	/** Asguard configuration */

	/** avg wheel radius for odometry calculations */
	double wheelRadiusAvg;
	/** distance between left and right wheel */
	double trackWidth;
	/** distance between front and rear axle */
	double wheelBase;

	/** compensate for the body center being different to the rotation center */
	bool bodyCenterCompensation;

	GaussianSamplingPose3D sampling;

	Eigen::Quaterniond orientation, prevOrientation;
	base::Pose pose;
    };

}
//}

#endif
