rock_library(odometry
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(odometry_bin Main.cpp
    DEPS odometry)

