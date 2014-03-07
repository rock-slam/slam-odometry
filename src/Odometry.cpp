#include "Odometry.hpp"
#include <stdexcept>

#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

using namespace odometry;
using namespace std;

SkidOdometry::SkidOdometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase)
    : config( config ), wheelRadiusAvg(wheelRadiusAvg),  trackWidth(trackWidth), wheelBase(wheelBase), 
      bodyCenterCompensation( true ), sampling(config)
{
}

Skid4Odometry::Skid4Odometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase)
    : SkidOdometry( config, wheelRadiusAvg, trackWidth, wheelBase )
{
}

void SkidOdometry::setBodyCenterCompensation(bool compensate)
{
    bodyCenterCompensation = compensate;
}

base::Pose SkidOdometry::getPoseDelta()
{
    return base::Pose( sampling.poseMean ); 
}

base::Pose SkidOdometry::getPoseDeltaSample()
{
    return base::Pose( sampling.sample() );
}

base::Pose2D SkidOdometry::getPoseDeltaSample2D()
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
    double d = (d1n+d2n)/2;

    SkidOdometry::update( d, orientation );
}

void SkidOdometry::update( double d, const Eigen::Quaterniond& orientation )
{
    double dx, dy, dtheta;
    
    Eigen::Quaterniond delta_rotq( prevOrientation.inverse() * orientation );
    Eigen::AngleAxisd delta_rot( delta_rotq ); 

    if( delta_rot.angle() > 1e-8 && delta_rot.axis().z() > 1e-9)
    {
	dtheta = (delta_rot.axis()*delta_rot.angle()).z();
	double r = d/dtheta;
	
	dx = r*(sin(dtheta)); // displacement in x coord
	dy = -r*(1-cos(dtheta)); // displacement in y coord
    }
    else
    {
    	dtheta = 0;
    	dx = d;
    	dy = 0;
    }

    base::Pose p(Eigen::Vector3d(dx, dy, 0), delta_rotq);
    
    if( bodyCenterCompensation )
    {
	// assume the rotation to be in the center of the body, so we
	// need to account for the fact that the body origin is in the 
	// center of the front axis
	Eigen::Affine3d t( p.toTransform() );
	Eigen::Affine3d C_center2body( Eigen::Translation3d( Eigen::Vector3d(-wheelBase/2.0, 0, 0 ) ) );

	pose = base::Pose( Eigen::Affine3d( C_center2body * t * C_center2body.inverse()) );
    }
    else
	pose = p;
    
    // calculate error matrix
    double tilt = acos(Eigen::Vector3d::UnitZ().dot(orientation*Eigen::Vector3d::UnitZ()));

    Eigen::Vector4d vec =
	config.constError.toVector4d() +
	d * config.distError.toVector4d() +
	tilt * config.tiltError.toVector4d() +
	dtheta * config.dthetaError.toVector4d();

    Vector6d var;
    var << 0, 0, vec.w(), vec.head<3>();

    sampling.update( pose.toVector6d(), var.asDiagonal() );

    prevOrientation = orientation;
}

Eigen::Matrix3d SkidOdometry::getPositionError()
{
    return sampling.poseCov.bottomRightCorner<3,3>();
}

Eigen::Matrix3d SkidOdometry::getOrientationError()
{
    return sampling.poseCov.topLeftCorner<3,3>();
}

base::Matrix6d SkidOdometry::getPoseError()
{
    return sampling.poseCov; 
}

Eigen::Matrix3d Skid4Odometry::getPositionError()
{
    Eigen::Matrix3d error_Covariance;
    Eigen::Vector3d translation = getTranslation();

    Eigen::Vector3d var(translation[0]*0.5, translation[0]*sin(2.0/180.0*M_PI) * 2.0, translation[0]*0.2);
    //var += Eigen::Vector3d(0.01,0.02,0.005);

    return var.array().square().matrix().asDiagonal(); 

}

Eigen::Matrix3d Skid4Odometry::getOrientationError()
{
    Eigen::Vector3d var(1e-5, 1e-5, 1e-4);

    return var.array().square().matrix().asDiagonal(); 
}

base::Matrix6d Skid4Odometry::getPoseError()
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

    return Eigen::Vector3d(sum / 4.0 * wheelRadiusAvg, ((sumright-sumleft) * wheelRadiusAvg / 2 ) / trackWidth * dist_cent_rot / w_damp, 0 );
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

	  //return Eigen::Vector3d( (sumright-sumleft) * wheelRadiusAvg / 2 / trackWidth / 2 / d_t,0, 0 ); 
	  return Eigen::Vector3d( 0, 0, (sumright-sumleft) * wheelRadiusAvg / 2 / trackWidth / 2 / d_t ); 
    
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
    error_Covariance(0,0)=_planar_velocity.array().abs()(1)*2*pow(10,-5)+pow(10,-12);
    //error in y
    if(_planar_velocity.array().abs()(0)<0.08) 
	error_Covariance(1,1)=_planar_velocity.array().abs()(1)*pow(10,-7)+pow(10,-12);
    else 
	if(_planar_velocity.array().abs()(0)>1.1) 
	    error_Covariance(1,1)=_planar_velocity.array().abs()(1)*pow(10,-2)+pow(10,-12);
	else
	    error_Covariance(1,1)=_planar_velocity.array().abs()(1)*2*pow(10,-3)+pow(10,-12);
    //error in z 
    error_Covariance(2,2)=pow(10,-5);

    return error_Covariance; 
};






