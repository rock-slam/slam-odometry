#ifndef __ASGUARD__TRANSFORMATION__
#define __ASGUARD__TRANSFORMATION__

#include "Configuration.hpp"

namespace odometry
{
    /** @deprecated This class should not be used for new developments. Use the 
     * Frame Transformation stack instead.
     */
    //class Transformation : public Configuration
    class Transformation
    {
    public:
	Transformation() { updateDerived(); };
	//explicit Transformation(){ 
	//updateDerived(); 
	//}
	//explicit Transformation( const Configuration& conf )
	//    : Configuration( conf ) { updateDerived(); }

	Eigen::Vector3d getFootPosition(const BodyState& state, wheelIdx wheel_idx, unsigned int foot_idx) const;

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	///Transformation from the center of the tilt head to the roatiation axis of it
	Eigen::Affine3d tiltHead2UpperDynamixel;
	
	///Transformation from the rotation axis of the tilt head to the head center / head Frame
	Eigen::Affine3d lowerDynamixel2Head;
	
	///Transform from laser to head coordinate frame
	Eigen::Affine3d laser2TiltHead;
	
	///Transform from laser to head coordinate frame
	Eigen::Affine3d laser2Head;
	///Transform from xsens to head coordinate frame
	Eigen::Affine3d xsens2Head;
	///Transform from head to body coordinate frame
	Eigen::Affine3d head2Body;
	///Transform from gps to body coordinate frame
	Eigen::Affine3d gps2Body;
	
	///Transform from laser to body coordinate frame
	Eigen::Affine3d laser2Body;
	///Transform from xsens to body coordinate frame
	Eigen::Affine3d xsens2Body;

	///Transform from body to body center coordinate frame
	Eigen::Affine3d body2BodyCenter;
	
	///Transformation from North West Up to East North Up coordinate frame
	Eigen::Affine3d nwu2Enu;
	
	/**As body is in the center of the front axis, this
	* transform gives us the transfomation to the level
	* of the wheel feet in the middle of the robot.
	* Note this is a static approximation, as the height to the
	* ground changes with wheel position.
	*/
	Eigen::Affine3d body2Ground;

	Eigen::Affine3d bodyCenter2Ground;
	
	void updateBody2Ground(const BodyState &bs);

	/** set default configuration settings */
	void setDefault();

	/** update configuration values that are derived from the base values */
	void updateDerived();

	/** body to IMU fixed coordinate system */
	base::Quaterniond R_b2i;
	/** body to GPS fixed coordinate system */
	base::Quaterniond R_b2g;
	/** body to LaserScan fixed coordinate system */
	base::Quaterniond R_b2ls;

	/** xsens world to world transformation */
	base::Quaterniond R_xw2w;
	/** angle of the sensorhead with respect to the main body */
	double sensorHeadAngle;
	/* foot contact point. */
	double wheelRadiusMax;
	/** distance between left and right wheel */
	double trackWidth;
	/** distance between front and rear axle */
	double wheelBase;
    private:
	Eigen::Quaterniond R_y;
    };

}
#endif
