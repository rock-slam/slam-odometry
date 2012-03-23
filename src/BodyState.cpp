#include "BodyState.hpp"

#include <stdexcept>

namespace odometry 
{
bool isLeft( wheelIdx idx )
{
    return idx == REAR_LEFT || idx == FRONT_LEFT;
}

bool isFront( wheelIdx idx )
{
    return idx == FRONT_LEFT || idx == FRONT_RIGHT;
}

wheelIdx wheelIdxEnum( unsigned long idx )
{
    if( idx > NUMBER_OF_WHEELS )
	throw std::runtime_error("invalid wheel index given.");

    return static_cast<wheelIdx>( idx );
}
}

using namespace odometry;

BodyState::BodyState()
{
    twistAngle = 0;
    for ( size_t i = 0; i < NUMBER_OF_WHEELS; i++ ) 
      wheelPos[i] = 0; 
}

double BodyState::getWheelPos(wheelIdx idx) const
{ 
    return wheelPos[idx];
}

void BodyState::setWheelPos(wheelIdx idx, double value) 
{ 
    wheelPos[idx] = value; 
}

WheelContact& BodyState::getWheelContact(wheelIdx idx, unsigned int footNum )
{
    if( footNum < NUMBER_OF_FEET )
	return wheelContacts[idx * NUMBER_OF_FEET + footNum];
    else
	throw std::runtime_error("Wrong foot index");
}

const WheelContact& BodyState::getWheelContact(wheelIdx idx, unsigned int footNum ) const
{
    if( footNum < NUMBER_OF_FEET )
	return wheelContacts[idx * NUMBER_OF_FEET + footNum];
    else
	throw std::runtime_error("Wrong foot index");
}

void BodyState::setWheelContact(wheelIdx idx, unsigned int footNum, const WheelContact& contact )
{
    if( footNum < NUMBER_OF_FEET )
	wheelContacts[idx * NUMBER_OF_FEET + footNum] = contact;
}
