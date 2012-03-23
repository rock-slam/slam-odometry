#ifndef __ASGUARDBODYSTATE_HPP__
#define __ASGUARDBODYSTATE_HPP__

#include <base/time.h>
#include <stdexcept>

namespace odometry 
{
    static const size_t NUMBER_OF_WHEELS = 4;
    static const size_t NUMBER_OF_FEET = 5;

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

    struct WheelContact
    {
	WheelContact() : contact(0.0), slip(0.0) {};
	explicit WheelContact(double contact, double slip = 0.0) 
	    : contact(contact), slip(slip) {}

	/** set to 1.0 if the wheel has contact with the environment */
	double contact;

	/** distance of how much this wheel has slipped in the current contact
	 * phase should be set to 0 after loss of contact */
	double slip;
    };

    class BodyState
    {
    public: 
       BodyState(); 
    public:
	/** timestamp of the body state */
	base::Time time;

	/** angle of the body twist joint */
	double twistAngle;

	double getWheelPos(wheelIdx idx) const;
	void setWheelPos(wheelIdx idx, double value);

	const WheelContact& getWheelContact(wheelIdx idx, unsigned int footNum ) const;
	WheelContact& getWheelContact(wheelIdx idx, unsigned int footNum );

	void setWheelContact(wheelIdx idx, unsigned int footNum, const WheelContact& contact );

	/** angle of the wheel in radians, accumulated over time (not wrapped around). */
	double wheelPos[NUMBER_OF_WHEELS];

	/** wheel contact information for each of the feet */ 
	WheelContact wheelContacts[NUMBER_OF_WHEELS*NUMBER_OF_FEET];
	
	
    };
}

#endif
