rock_library(odometry
    SOURCES ContactOdometry.cpp
	    Odometry.cpp
	    BodyState.cpp
	    Configuration.cpp
    HEADERS ContactState.hpp
	    Configuration.hpp
	    BodyState.hpp
	    Odometry.hpp
	    ContactOdometry.hpp
    DEPS_PKGCONFIG base-types)

