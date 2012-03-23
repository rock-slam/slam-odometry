rock_library(odometry
    SOURCES ContactOdometry.cpp
	    Odometry.cpp
	    BodyState.cpp
    HEADERS ContactState.hpp
	    Configuration.hpp
	    BodyState.hpp
	    Odometry.hpp
	    SysMonTypes.hpp
	    ContactOdometry.hpp
	    Transformation.hpp
    DEPS_PKGCONFIG base-types)
#rock_executable(odometry_bin Main.cpp
#    DEPS odometry)

