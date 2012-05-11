#include "ContactOdometry.hpp"

using namespace odometry;

FootContact::FootContact(const Configuration& config)
    : config( config ), sampling(config)
{
}

/** this function will return the transformation from the frame that is spanned
 * up the points p1, p2 and p3 to the global frame.  p1 is the origin of the
 * frame and p1-p2 the x axis.  further, p3 will be on the plane spanned up the
 * the x and y axis of the frame.
 */
/*
bool FootContact::getTransformFromPoints( const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, Eigen::Affine3d &trans )
{
    trans = Eigen::Affine3d::Identity();
    Eigen::Matrix3d rot;
    const Eigen::Vector3d c1(p2-p1), c2(p3-p1);

    rot.col(0) = c1.normalized();
    rot.col(2) = c2.cross( c1 ).normalized();
    if( rot.col(2).norm() < 1e-5 )
	return false; // the rotation is underspecified
    else
	rot.col(1) = c1.cross(rot.col(2)).normalized(); 

    trans.matrix().topLeftCorner<3,3>() = rot;
    trans.pretranslate( p1 );
    return true;
}
*/

Eigen::Matrix3d FootContact::getPositionError()
{
    return Eigen::Matrix3d(sampling.poseCov.bottomRightCorner<3,3>());
}

Eigen::Matrix3d FootContact::getOrientationError()
{
    return Eigen::Matrix3d(sampling.poseCov.topLeftCorner<3,3>());
}

Matrix6d FootContact::getPoseError()
{
    return sampling.poseCov;
}

base::Pose FootContact::getPoseDeltaSample()
{
    return base::Pose( sampling.sample() );
}

base::Pose2D FootContact::getPoseDeltaSample2D()
{
    return projectPoseDelta( orientation, getPoseDeltaSample() );
}

base::Pose FootContact::getPoseDelta()
{
    return base::Pose( sampling.poseMean );
}

void FootContact::update(const odometry::BodyContactState& bs, const Eigen::Quaterniond& orientation)
{
    // update state
    state.update( bs );
    this->orientation = orientation;

    if( !state.isValid() )
    {
	state.update( bs );
	prevOrientation = orientation;
    }

    // get relative rotation between updates
    // this is assumed to be the correct rotation (with error of course)
    Eigen::Quaterniond delta_rotq( prevOrientation.inverse() * orientation );

    // the translation we get from the fact, that we assume that contact points
    // with the environment stay static. We do this by rotating the current 
    // contact points into the previous frame, and look at the difference between
    // points that have been in contact in the current and in the previous frame.
    // Assuming equal slip conditions in all directions (could be changed later)
    // the overal translation should be the mean of those differences

    const double contact_threshold = 0.5;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int count = 0;
    for( size_t i=0; i < state.getPrevious().points.size(); i++ )
    {
	const odometry::BodyContactPoint &prevPoint( state.getPrevious().points[i] );
	const odometry::BodyContactPoint &point( state.getCurrent().points[i] );

	if( prevPoint.contact >= contact_threshold 
		&& point.contact >= contact_threshold )
	{
	    sum += prevPoint.position - delta_rotq * point.position;
	    count++;
	}
    }
    Eigen::Vector3d mean = sum;
    if( count > 0 ) 
	mean /= count;

    base::Pose delta_pose( mean, delta_rotq );
    
    // calculate error matrix
    // TODO this is based on the wheel odometry error model I think it could be
    // more accurate by looking the the covariance of the contact position
    // differences 
    
    double tilt = acos(Eigen::Vector3d::UnitZ().dot(orientation*Eigen::Vector3d::UnitZ()));
    double d = mean.norm();
    double dtheta = Eigen::AngleAxisd( delta_rotq ).angle();

    Eigen::Vector4d vec =
	config.constError.toVector4d() +
	d * config.distError.toVector4d() +
	tilt * config.tiltError.toVector4d() +
	dtheta * config.dthetaError.toVector4d();

    Vector6d var;
    var << 0, 0, vec.w(), vec.head<3>();

    sampling.update( delta_pose.toVector6d(), var.asDiagonal() );

    prevOrientation = orientation;
}
