#ifndef __ODOMETRY_ODOMETRY_HPP__
#define __ODOMETRY_ODOMETRY_HPP__

#include <base/Time.hpp>
#include <base/Pose.hpp>

#include <odometry/Gaussian.hpp>
#include <odometry/Configuration.hpp>
#include <odometry/BodyState.hpp>
#include <odometry/State.hpp>
#include <odometry/Gaussian3D.hpp>
#include <odometry/Sampling3D.hpp>
#include <odometry/Sampling2D.hpp>
#include <base/samples/Joints.hpp>

namespace odometry
{
    /** @brief Class which provides a simple wheel (skid steering) odometry
     *
     * class is based on a simple skid steering model, where the change in
     * orientation is provided by an IMU, so effectively only the translation
     * comes from a change in wheel position.
     */
    class SkidOdometry
	: public Gaussian3D,
	  public Sampling3D,
	  public Sampling2D
    {
    public:
	typedef base::Matrix6d Matrix6d;
	typedef base::Vector6d Vector6d;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/** 
	 * @brief default constructor for wheel odometry class
	 *
	 * @param config odometry configuration class, mainly setting the error matrix
	 * @param asguardConfig configuration of the asguard body model
	 */
	SkidOdometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase,
                     const std::vector<std::string> &leftWheelNames, const std::vector<std::string> &rightWheelNames);

	/** 
	 * @brief update method which is called for each new measurement
	 *
	 * @param d average of wheel distance travelled within the time step 
	 * @param orientation body to world rotation provided by an IMU
	 */
	void update( double d, const Eigen::Quaterniond& orientation );

        /** 
         * @brief update method which is called for each new measurement
         *
         * @param jbs encodes the current in wheel positions
         * @param orientation body to world rotation provided by an IMU
         */
        void update(const base::samples::Joints &js, const Eigen::Quaterniond& orientation);

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

    protected:
	/** Odometry configuration */
	Configuration config;

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
        
        State<base::samples::Joints> jointState;
            
        std::vector<std::string> leftWheelNames;
        std::vector<std::string> rightWheelNames;

        /**
         * helper function that computes the mean distance 
         * traveled by the specified actuators
         * */
        double getTranslation(const std::vector< std::string >& actuatorNames);
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
	: public SkidOdometry 
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
	void update(const BodyState &bs, const Eigen::Quaterniond& orientation);

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
        State<BodyState> state;
    };

}

#endif
