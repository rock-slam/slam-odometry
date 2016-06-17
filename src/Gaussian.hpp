#ifndef ODOMETRY_GAUSSIAN_HPP__
#define ODOMETRY_GAUSSIAN_HPP__

#include <odometry/Configuration.hpp>
#include <base/Pose.hpp>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

namespace odometry
{

base::Pose2D projectPoseDelta( const Eigen::Quaterniond& orientation, const base::Pose& pose );

/** 
 * @brief helper class that represents a pose (position and orientation)
 * and associated gaussian uncertainty, which can also produce samples
 */
class GaussianSamplingPose3D
{
    typedef base::Matrix6d Matrix6d;
    typedef base::Vector6d Vector6d;

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

protected:
    Configuration config;

    /** random number generator */
    boost::variate_generator<boost::minstd_rand, boost::normal_distribution<> > rand_norm;
};

}
#endif
