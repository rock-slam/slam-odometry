rock_library(odometry
    SOURCES ContactOdometry.cpp
	    Odometry.cpp
	    BodyState.cpp
	    Configuration.cpp
	    Gaussian.cpp
    HEADERS ContactState.hpp
	    Configuration.hpp
	    BodyState.hpp
	    Odometry.hpp
	    ContactOdometry.hpp
	    Gaussian.hpp
            KinematicModel.hpp
            MotionModel.hpp
            State.hpp
            Gaussian2D.hpp
            Gaussian3D.hpp
            Sampling2D.hpp
            Sampling3D.hpp
    DEPS_PKGCONFIG base-types)

