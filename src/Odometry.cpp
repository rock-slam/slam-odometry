#include "Odometry.hpp"
#include <stdexcept>

#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

using namespace odometry;
using namespace std;

base::Pose odometry::getPoseFromVector6d( const Vector6d &v )
{
    Eigen::Quaterniond qrot = Eigen::Quaterniond::Identity();
    const Eigen::Vector3d saxis = v.head<3>();
    if( saxis.norm() > 1e-9 )
    {
	qrot = Eigen::AngleAxisd( saxis.norm(), saxis.normalized() );
    }

    return base::Pose( v.tail<3>(), qrot );
}

Vector6d odometry::getVector6dFromPose( const base::Pose& pose )
{
    Vector6d diff;
    Eigen::AngleAxisd aa(pose.orientation);
    diff.head<3>() = aa.axis() * aa.angle();
    diff.tail<3>() = pose.position;

    return diff;
}

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
    rand_gen( config.seed ),
    rand_norm(rand_gen, boost::normal_distribution<>(0,1.0) )
{
}

void GaussianSamplingPose3D::update( const Vector6d& mean, const Matrix6d& llt )
{
    poseMean = mean;
    poseCov = llt * llt.transpose();
    poseLT = llt;
}

Vector6d GaussianSamplingPose3D::sample()
{
    Vector6d n;
    for(int i=0;i<6;i++)
	n[i] = rand_norm();

    return poseLT * n + poseMean;
}


Skid4Odometry::Skid4Odometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase)
    : config( config ), wheelRadiusAvg(wheelRadiusAvg),  trackWidth(trackWidth), wheelBase(wheelBase), 
      bodyCenterCompensation( true ), sampling(config)
{
}

void Skid4Odometry::setBodyCenterCompensation(bool compensate)
{
    bodyCenterCompensation = compensate;
}

base::Pose Skid4Odometry::getPoseDelta()
{
    return getPoseFromVector6d( sampling.poseMean );
}

base::Pose Skid4Odometry::getPoseDeltaSample()
{
    return getPoseFromVector6d( sampling.sample() );
}

base::Pose2D Skid4Odometry::getPoseDeltaSample2D()
{
    return projectPoseDelta( orientation, getPoseDeltaSample() );
}


void Skid4Odometry::update(const BodyState &bs, const Eigen::Quaterniond& orientation)
{
    state.update( bs );
    this->orientation = orientation;

    if( !state.isValid() )
    {
	state.update( bs );
	prevOrientation = orientation;
    }

    double wheel_radius = wheelRadiusAvg; 

    const BodyState &state_kp(state.getCurrent());
    const BodyState &state_k(state.getPrevious());

    double d1n = (state_kp.getWheelPos(FRONT_LEFT) - state_k.getWheelPos(FRONT_LEFT) 
	    + state_kp.getWheelPos(REAR_LEFT) - state_k.getWheelPos(REAR_LEFT) )/2.0*wheel_radius; // averaged left side distance 
    double d2n = (state_kp.getWheelPos(FRONT_RIGHT) - state_k.getWheelPos(FRONT_RIGHT) 
	    + state_kp.getWheelPos(REAR_RIGHT) - state_k.getWheelPos(REAR_RIGHT) )/2.0*wheel_radius; // averaged left side distance 

    double dx, dy, dtheta;
    
    double d = (d1n+d2n)/2;
    Eigen::Quaterniond delta_rotq( prevOrientation.inverse() * orientation );
    Eigen::AngleAxisd delta_rot( delta_rotq );
    if( delta_rot.angle() > 1e-8 )
    {
	dtheta = (delta_rot.axis()*delta_rot.angle()).z();
	double r = d/dtheta;

	dx = r*(1-cos(dtheta)); // displacement in x coord
	dy = r*sin(dtheta); // displacement in y coord
    }
    else
    {
	dtheta = 0;
	dx = 0;
	dy = d;
    }

    base::Pose p(Eigen::Vector3d(dx, dy, 0), delta_rotq);
    //base::Pose p(Eigen::Vector3d(dx, dy, 0), Eigen::Quaterniond(Eigen::AngleAxisd( dtheta, Eigen::Vector3d::UnitZ())));
    
    if( bodyCenterCompensation )
    {
	// assume the rotation to be in the center of the body, so we
	// need to account for the fact that the body origin is in the 
	// center of the front axis
	Eigen::Affine3d t( p.toTransform() );
	Eigen::Affine3d C_center2body( Eigen::Translation3d( Eigen::Vector3d(0, -wheelBase/2.0, 0 ) ) );

	pose = base::Pose( Eigen::Affine3d( C_center2body * t * C_center2body.inverse()) );
    }
    else
	pose = p;
    
    // calculate error matrix
    double tilt = acos(Eigen::Vector3d::UnitZ().dot(orientation*Eigen::Vector3d::UnitZ()));

    /*
    Eigen::Vector3d orientation_var(
	    1e-5, 
	    1e-5, 
	    config.yawError
	    );

    Eigen::Vector3d translation_var(
	    0.002+dtheta*0.2+d*0.1,
	    0.005+tilt*0.02+d*0.5,
	    0.001
	    );
	    */

    Eigen::Vector4d vec =
	config.constError.toVector4d() +
	d * config.distError.toVector4d() +
	tilt * config.tiltError.toVector4d() +
	dtheta * config.dthetaError.toVector4d();

    Vector6d var;
    var << 0, 0, vec.w(), vec.head<3>();

    sampling.update( getVector6dFromPose(pose), var.asDiagonal() );

    prevOrientation = orientation;
}

Eigen::Matrix3d Skid4Odometry::getPositionError()
{
    Eigen::Matrix3d error_Covariance;
    Eigen::Vector3d translation = getTranslation();

    Eigen::Vector3d var(translation[1]*sin(2.0/180.0*M_PI) * 2.0, translation[1]*0.5, translation[1]*0.2);
    //var += Eigen::Vector3d(0.01,0.02,0.005);

    return var.array().square().matrix().asDiagonal(); 

}

Eigen::Matrix3d Skid4Odometry::getOrientationError()
{
    Eigen::Vector3d var(1e-5, 1e-5, 1e-4);

    return var.array().square().matrix().asDiagonal(); 
}

Matrix6d Skid4Odometry::getPoseError()
{
    Matrix6d cov;
    cov << 
	getOrientationError(), Eigen::Matrix3d::Zero(),
	Eigen::Matrix3d::Zero(), getPositionError();
    
    return cov; 
}


Eigen::Vector3d Skid4Odometry::getTranslation()
{
    // TODO update this parameter to make the rotation center the defined body center 
    // TODO move the parameter into the Transform class
    
    double dist_cent_rot = -0.05; 
    

    double sumleft = 
	state.getCurrent().getWheelPos(FRONT_LEFT) - state.getPrevious().getWheelPos(FRONT_LEFT)
	+ state.getCurrent().getWheelPos(REAR_LEFT) - state.getPrevious().getWheelPos(REAR_LEFT);
    double sumright = 
	state.getCurrent().getWheelPos(FRONT_RIGHT) - state.getPrevious().getWheelPos(FRONT_RIGHT)
	+ state.getCurrent().getWheelPos(REAR_RIGHT) - state.getPrevious().getWheelPos(REAR_RIGHT);

    double sum = sumleft + sumright;

    // TODO explain the damping better, and maybe move it to somewhere else 
    //parameter do damp de velocity in the x axis
    double w_damp = 2.5;

    return Eigen::Vector3d(
	    ( (sumright-sumleft) * wheelRadiusAvg / 2 ) / trackWidth * dist_cent_rot / w_damp, 
	    sum / 4.0 * wheelRadiusAvg, 
	    0 );
};

Eigen::Vector3d Skid4Odometry::getVelocity()
{
    double d_t = state.getTimeDelta().toSeconds(); 
    if(d_t > 0.05)
	std::cout<< " Dt warning " << d_t << std::endl; 

    if(d_t > 0)
	return getTranslation() / d_t; 
    else
	return Eigen::Vector3d::Zero();
}

Eigen::Vector3d Skid4Odometry::getAngularVelocity()
{
    double d_t = state.getTimeDelta().toSeconds(); 

    if(d_t > 0.05)
	std::cout<< " Dt warning " << d_t << std::endl; 

    if(d_t > 0)
    {

	// TODO update this parameter to make the rotation center the defined body center 
	// TODO move the parameter into the Transform class
	//double dist_cent_rot=0.1743; 

	double sumleft = 
	    state.getCurrent().getWheelPos(FRONT_LEFT) - state.getPrevious().getWheelPos(FRONT_LEFT)
	    + state.getCurrent().getWheelPos(REAR_LEFT) - state.getPrevious().getWheelPos(REAR_LEFT);
	double sumright = 
	    state.getCurrent().getWheelPos(FRONT_RIGHT) - state.getPrevious().getWheelPos(FRONT_RIGHT)
	    + state.getCurrent().getWheelPos(REAR_RIGHT) -state.getPrevious().getWheelPos(REAR_RIGHT);

	  //double sum = sumleft + sumright;

	  // TODO explain the damping better, and maybe move it to somewhere else 
	  //parameter do damp de velocity in the x axis
	  return Eigen::Vector3d( (sumright-sumleft) * wheelRadiusAvg / 2 / trackWidth / 2 / d_t,0, 0 ); 
    
    }
    else
      return Eigen::Vector3d(0, 0, 0); 
}

double Skid4Odometry::getDeltaYaw () 
{ 

    double d_t = state.getTimeDelta().toSeconds(); 
    return getAngularVelocity()(0,0)*d_t;
    
} 


Eigen::Matrix3d Skid4Odometry::getVelocityError() 
{
    Eigen::Matrix3d error_Covariance;
    Eigen::Vector3d _planar_velocity = getVelocity();
    
    error_Covariance.setZero(); 

    
    //Error in X 
    
    //std::cout<<_planar_velocity<<std::endl; 
  //  if(_planar_velocity.cwise().abs()(1)>100) {
   //     std::cout<<  (state_kp.time - state_k.time).toSeconds()<<std::endl;
//	for(int i=0;i<4;i++){
//	  std::cout<<"prev wheel pos"<< i << " =  " <<state_kp.wheelPos[i] << std::endl; 
	//  std::cout<<"curr wheel pos"<< i << " =  " <<state_k.wheelPos[i] << std::endl; 
	//  exit(1);
//	}
  //  }
  
    error_Covariance.setZero(); 
    //error in x 
    if(_planar_velocity.array().abs()(0)<0.08) 
	error_Covariance(0,0)=_planar_velocity.array().abs()(1)*pow(10,-7)+pow(10,-12);
    else 
	if(_planar_velocity.array().abs()(0)>1.1) 
	    error_Covariance(0,0)=_planar_velocity.array().abs()(1)*pow(10,-2)+pow(10,-12);
	else
	    error_Covariance(0,0)=_planar_velocity.array().abs()(1)*2*pow(10,-3)+pow(10,-12);
    //error in y
    error_Covariance(1,1)=_planar_velocity.array().abs()(1)*2*pow(10,-5)+pow(10,-12);
    //error in z 
    error_Covariance(2,2)=pow(10,-5);

    return error_Covariance; 
};






