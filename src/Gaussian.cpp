#include "Gaussian.hpp"

using namespace odometry;

/** returns an orientation wich removes the rotation around the z axis in
 * such a way, that the 2d projection on the z axis of the resulting
 * orientation will always have a zero angle. 
 *
 * note: this is not the same as removing the yaw rotation in of the euler angle
 * decomposition.
 */
base::Orientation removeZRotation( const base::Orientation& orientation )
{
    // remove z compenent for pose
    Eigen::Vector3d projy = orientation * Eigen::Vector3d::UnitY(); 
    Eigen::Quaterniond comp( Eigen::AngleAxisd( -atan2( -projy.x(), projy.y() ), Eigen::Vector3d::UnitZ() ) * orientation );

    return comp;
}

/** projects a given 3d pose into 2d on the z axis
*/
base::Pose2D projectPose( const base::Pose& pose )
{
    Eigen::Vector3d projy2d = pose.orientation * Eigen::Vector3d::UnitY(); 
    double rot = atan2( -projy2d.x(), projy2d.y() );

    // return projection on z axis in world frame 
    return base::Pose2D( Eigen::Vector2d( pose.position.head<2>() ), rot );
}

base::Pose2D odometry::projectPoseDelta( const Eigen::Quaterniond& orientation, const base::Pose& delta )
{
    // remove z rotation compenent for pose
    Eigen::Quaterniond comp( removeZRotation( orientation) );

    // rotate delta into world frame
    base::Pose deltaw( comp*delta.position, comp*delta.orientation );

    return projectPose( deltaw );
}

GaussianSamplingPose3D::GaussianSamplingPose3D( const Configuration& config ) : 
    poseCov( Matrix6d::Zero() ),
    poseLT( Matrix6d::Zero() ),
    poseMean( Vector6d::Zero() ),
    modelError( Matrix6d::Identity() * 1e-8 ), 
    config( config ), 
    rand_norm( boost::minstd_rand( config.seed ), boost::normal_distribution<>(0,1.0) )
{
}

void GaussianSamplingPose3D::update( const Vector6d& mean, const Matrix6d& llt )
{
    poseMean = mean;
    poseCov = llt * llt.transpose();
    poseLT = llt;
}

void GaussianSamplingPose3D::updateMean( const Vector6d& mean )
{
    poseMean = mean;
}

void GaussianSamplingPose3D::updateCholesky( const Matrix6d& llt )
{
    poseCov = llt * llt.transpose();
    poseLT = llt;
}

base::Vector6d GaussianSamplingPose3D::sample()
{
    Vector6d n;
    for(int i=0;i<6;i++)
	n[i] = rand_norm();

    return poseLT * n + poseMean;
}



