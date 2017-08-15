#include "Odometry.hpp"
#include <stdexcept>

#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

using namespace odometry;
using namespace std;

SkidOdometry::SkidOdometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase, const vector< string >& leftWheelNames, const vector< string >& rightWheelNames,
                           const vector< string >& leftSteeringNames, const vector< string >& rightSteeringNames)
                               : config( config ), wheelRadiusAvg(wheelRadiusAvg),  trackWidth(trackWidth), wheelBase(wheelBase), 
                                sampling(config), leftWheelNames(leftWheelNames), rightWheelNames(rightWheelNames),
                                leftSteeringNames(leftSteeringNames), rightSteeringNames(rightSteeringNames){

}


SkidOdometry::SkidOdometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase, const vector< string >& leftWheelNames, const vector< string >& rightWheelNames)
    : config( config ), wheelRadiusAvg(wheelRadiusAvg),  trackWidth(trackWidth), wheelBase(wheelBase), 
      sampling(config), leftWheelNames(leftWheelNames), rightWheelNames(rightWheelNames)
{
}

Skid4Odometry::Skid4Odometry(const Configuration& config, double wheelRadiusAvg, double trackWidth, double wheelBase)
    : SkidOdometry( config, wheelRadiusAvg, trackWidth, wheelBase, std::vector<std::string>(), std::vector<std::string>())
{
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

double SkidOdometry::getTranslation(const vector< string >& actuatorNames)
{
    double posDiff = 0;
    
    for(std::vector<std::string>::const_iterator it = actuatorNames.begin();
        it != actuatorNames.end(); it++)
    {
        base::JointState const &state(jointState.getCurrent()[*it]);
        if(!state.hasPosition())
        {
            if(!state.hasSpeed())
                throw std::runtime_error("Skid: Error, actuator sample did not contain position nor speed reading");
                            
            //Hm, is it the speed of the current time, or the last state ?
            posDiff += state.speed * jointState.getTimeDelta().toSeconds();
        }
        else
        {
            base::JointState const &lastState(jointState.getPrevious()[*it]);
            posDiff += state.position - lastState.position;
        }
    }
    
    return posDiff / actuatorNames.size() * wheelRadiusAvg;
}

double SkidOdometry::getRotation(const vector< string >& actuatorNames)
{
    double rotation = 0;
    for(std::vector<std::string>::const_iterator it = actuatorNames.begin();
    it != actuatorNames.end(); it++)
    {
        base::JointState const &state(steeringJointState.getCurrent()[*it]);
        if(!state.hasPosition())
        {
            throw std::runtime_error("Skid: Error, actuator sample did not contain position reading");
        }
        rotation += state.position;
        
    }
    
    return rotation / actuatorNames.size();
}


void SkidOdometry::update(const base::samples::Joints& js, const Eigen::Quaterniond& orientation)
{
    jointState.update( js );
    this->orientation = orientation;

    if( !jointState.isValid() )
    {
        jointState.update( js );
        prevOrientation = orientation;
    }

    // averaged left side distance 
    double d1n = getTranslation(leftWheelNames);
    
    // averaged left side distance 
    double d2n = getTranslation(rightWheelNames); 
    double d = (d1n+d2n)/2;

    SkidOdometry::update( d, orientation );
    
}

void SkidOdometry::update(const base::samples::Joints& jsw, const base::samples::Joints& jss, const Eigen::Quaterniond& orientation)
{
    jointState.update( jsw );
    this->orientation = orientation;

    if( !jointState.isValid() )
    {
        jointState.update( jsw );
        prevOrientation = orientation;
    }
    steeringJointState.update(jss);
    if( !steeringJointState.isValid()){
        steeringJointState.update(jsw);
        prevOrientation = orientation;
    }
    
    

    // averaged left side distance 
    double dl = getTranslation(leftWheelNames);
    
    // averaged left side distance 
    double dr = getTranslation(rightWheelNames); 
    
    double rl = getRotation(leftSteeringNames);
    double rr = getRotation(rightSteeringNames);
    
    Eigen::Vector2d vl(dl, 0), vr(dr, 0);
    Eigen::Rotation2D<double> rotl(rl), rotr(rr);
    vl = rotl * vl;
    vr = rotr * vr;
    

    SkidOdometry::update( vl + vr , orientation );
    
    std::cout << "L dl: "<< dl << " rotl: " << rotl.angle() << std::endl;
    std::cout << "R dr: "<< dr << " rotr: " << rotr.angle() << std::endl;
    std::cout << "L vl: "<< vl<< std::endl;
    std::cout << "R vr: "<< vr<< std::endl;
    
}

void SkidOdometry::update( double d, const Eigen::Quaterniond& orientation ){
    update(Eigen::Vector2d(d, 0), orientation);
}

void SkidOdometry::update( Eigen::Vector2d diff, const Eigen::Quaterniond& orientation )
{    
    Eigen::Quaterniond delta_rotq( prevOrientation.inverse() * orientation );
    Eigen::Vector3d proj_x (delta_rotq * Eigen::Vector3d::UnitX());
    const double dtheta = atan2(proj_x.y(), proj_x.x());
    Eigen::Rotation2D<double> rot2d(dtheta);
    diff = rot2d * diff;
    
    base::Pose p(Eigen::Vector3d(diff.x(), diff.y(), 0), delta_rotq);
    
    pose = p;
    
    // calculate error matrix
    double tilt = acos(Eigen::Vector3d::UnitZ().dot(orientation*Eigen::Vector3d::UnitZ()));

    Eigen::Vector4d vec =
	config.constError.toVector4d() +
	diff.norm() * config.distError.toVector4d() +
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






