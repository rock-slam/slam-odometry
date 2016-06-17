#ifndef __ODOMETRY_BODYSTATE_HPP__
#define __ODOMETRY_BODYSTATE_HPP__

#include <base/Time.hpp>

namespace odometry 
{
    static const size_t NUMBER_OF_WHEELS = 4;

    enum wheelIdx
    {
	REAR_LEFT   = 0,
	REAR_RIGHT  = 1,
	FRONT_RIGHT = 2,
	FRONT_LEFT  = 3
    };

    /** @brief convenience method to check if a given wheel index is on the left of the robot
     */
    extern bool isLeft( wheelIdx idx );

    /** @brief convenience method to check if a given wheel index is on the front of the robot 
     */
    extern bool isFront( wheelIdx idx );

    /** @brief get a wheel index from an unsigned long
     *
     * Can be used for enumerating the wheel using an index.
     *
     * will throw if the index is not valid.
     */
    extern wheelIdx wheelIdxEnum( unsigned long idx );

    class BodyState
    {
    public: 
       BodyState(); 
    public:
	/** timestamp of the body state */
	base::Time time;

	double getWheelPos(wheelIdx idx) const;
	void setWheelPos(wheelIdx idx, double value);

	/** angle of the wheel in radians, accumulated over time (not wrapped around). */
	double wheelPos[NUMBER_OF_WHEELS];
    };
}

#endif
