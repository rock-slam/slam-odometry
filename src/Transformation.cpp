#include "Configuration.hpp"
#include "Transformation.hpp"
#include <stdio.h>

#include <stdexcept>
#include <fstream>
#include "yaml.h"

namespace odometry {

void Transformation::updateDerived()
{
    // initialize the configuration values, most of it is tape measured or
    // comes from estimations
    //perhaps a better name tilt head to tilt head axis
    tiltHead2UpperDynamixel.setIdentity();
    //vector from axis to mounting point middle in tilt head frame
    tiltHead2UpperDynamixel.translation() = Eigen::Vector3d(0, 0.0, -0.018);
    
    //head axis to tilt head axis
    lowerDynamixel2Head.setIdentity();
    lowerDynamixel2Head.translation() = Eigen::Vector3d(0.0, 0.0, 0.099);
    
    //laser emitter to midle of tilt head mounting
    laser2TiltHead.setIdentity();
    laser2TiltHead.translation() = Eigen::Vector3d(0.0, 0.006, 0.063);
    
    head2Body.setIdentity();
    head2Body.rotate(Eigen::Quaterniond(Eigen::AngleAxisd(sensorHeadAngle, Eigen::Vector3d::UnitX())));
    //Vector from body to head in body frame	
    head2Body.translation() = Eigen::Vector3d(0, 0.022, 0.1175);
    
    xsens2Head.setIdentity();
    //xsens is 90 degrees rotated to the left in relation to head
    xsens2Head.rotate(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ())));
    //Vector from head to xsens in head frame
    xsens2Head.translation() = Eigen::Vector3d(-0.025, -0.025, -0.007);
    
    laser2Head.setIdentity();
    //Vector from head to laser in head frame
    laser2Head.translation() = Eigen::Vector3d(0.0, 0.0075, .145);
    
    gps2Body.setIdentity();
    //Vector from Body to gps in Body frame
    gps2Body.translation() = Eigen::Vector3d(0, -.428, 0.175);
    
    laser2Body = head2Body * laser2Head;
    xsens2Body = head2Body * xsens2Head;
    
    bodyCenter2Ground.setIdentity();
    //Vector from Ground to body in Ground frame
    bodyCenter2Ground.translation() = Eigen::Vector3d(0,0, 0.18);
    
    body2BodyCenter.setIdentity();
    //Body center is 30 cm behind the body frame
    body2BodyCenter.translation() = Eigen::Vector3d(0,0.3,0);
    
    body2Ground  = bodyCenter2Ground * body2BodyCenter;
    
    nwu2Enu.setIdentity();
    nwu2Enu.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));    
    
    R_xw2w = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()); 
    R_b2i = Eigen::AngleAxisd( sensorHeadAngle, Eigen::Vector3d::UnitY() ) * Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitZ() );
    R_b2ls.setIdentity();
}

Eigen::Vector3d Transformation::getFootPosition(const BodyState& state, wheelIdx wheel_idx, unsigned int foot_idx) const
{
    Eigen::Vector3d f = 
	Eigen::AngleAxisd(-state.getWheelPos(wheel_idx)+foot_idx*(2.0*M_PI/5.0), Eigen::Vector3d::UnitX())
	* Eigen::Vector3d(0, 0, wheelRadiusMax);
    
   if( wheel_idx == FRONT_LEFT || wheel_idx == REAR_LEFT )
       f += Eigen::Vector3d( -trackWidth/2.0, 0, 0 );
   else
       f += Eigen::Vector3d( trackWidth/2.0, 0, 0 );

   if( wheel_idx == REAR_LEFT || wheel_idx == REAR_RIGHT )
   {
       f += Eigen::Vector3d( 0, -wheelBase, 0 );
       f = Eigen::AngleAxisd(state.twistAngle, Eigen::Vector3d::UnitY()) * f;
   }

   return f;
}

void Transformation::updateBody2Ground(const odometry::BodyState& bs)
{
    std::vector<Eigen::Vector3d> footPosWithSmallestZ;
    footPosWithSmallestZ.resize(4);
    
    double biggestZ = std::numeric_limits< double >::min();
    unsigned int unusedOne = 0;
    
    for(unsigned int wheelidx = 0; wheelidx < 4; wheelidx++)
    {
	//FIXME this cast is potentially dangerous
	wheelIdx wIdx = (wheelIdx) (wheelidx);
	for(unsigned int footidx = 0; footidx < 5; footidx++)
	{
	    Eigen::Vector3d curFootPoint = getFootPosition(bs, wIdx, footidx);
	    
	    if(footidx == 0)
		footPosWithSmallestZ[wheelidx] = curFootPoint;
	    else
	    {
		if(curFootPoint.z() < footPosWithSmallestZ[wheelidx].z())
		    footPosWithSmallestZ[wheelidx] = curFootPoint;
	    }
	}

	if(footPosWithSmallestZ[wheelidx].z() > biggestZ)
	    unusedOne = wheelidx;
    }
    
    std::vector<int> validFootPoints;
    
    for(unsigned int wheelidx = 0; wheelidx < 4; wheelidx++)
    {
	if(wheelidx != unusedOne)
	    validFootPoints.push_back(wheelidx);
    }    
    
    Eigen::Vector3d v1, v2, a;
    a = footPosWithSmallestZ[1];
    v1 = footPosWithSmallestZ[validFootPoints[0]] - footPosWithSmallestZ[validFootPoints[1]];
    v2 = footPosWithSmallestZ[validFootPoints[2]] - footPosWithSmallestZ[validFootPoints[1]];
    
    v1.normalize();
    v2.normalize();
    
    Eigen::Vector3d normal = v1.cross(v2);
    
    normal.normalize();
    //ebenengleichung : (x - v1) * normal = 0
    
    //distance from robot to foot plan. 
    //In this case this is the intersection point of the Z-Axis of the 
    //robot coordinate system with the ground plane, build from the foot contact points
    double distanceBodyCenter2Ground = normal.dot(a+-body2BodyCenter.translation()) / normal.dot(Eigen::Vector3d(0,0,-1));
    
    bodyCenter2Ground.translation() = (Eigen::Vector3d::UnitZ() * distanceBodyCenter2Ground);
    
    body2Ground  = bodyCenter2Ground * body2BodyCenter;
}


}
