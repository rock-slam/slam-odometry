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
    DEPS_PKGCONFIG base-types)

