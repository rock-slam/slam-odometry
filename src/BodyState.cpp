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

