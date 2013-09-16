#ifndef __ODOMETRY_STATE_HPP__
#define __ODOMETRY_STATE_HPP__

#include <Eigen/Core>
#include <base/Time.hpp>
#include <base/Pose.hpp>

namespace odometry
{
    /** Class which provides common methods for bodystate handling which can be
     * used in the odometry models 
     */
    template <class BodyState_>
    class State
    {
    public:
	State() 
	    : update_counter(0) {}

	/** 
	 * will set the current body state and store the previous state
	 * to perform the odometry calculations.
	 */
	virtual void update( const BodyState_& state )
	{
	    // make the current configuration the previous configuration
	    state_k = state_kp;
	    // set the current configuration with the new values
	    state_kp = state;
	    update_counter++;
	}

	/** 
	 * return true if both a current and a previous Bodystate are set,
	 * so updateBodyState has been called at least twice.
	 */
	bool isValid()
	{
	    return update_counter > 1;
	}

	/** 
	 * returns the time difference between the current and the previous
	 * state
	 */
	base::Time getTimeDelta() const
	{
	    return state_kp.time - state_k.time;
	}

	const BodyState_& getCurrent() const
	{
	    return state_kp;
	}

	const BodyState_& getPrevious() const
	{
	    return state_k;
	}

    protected:
	size_t update_counter;

	/** Body state at time k */ 
	BodyState_ state_k, state_kp;
    };
}

#endif
