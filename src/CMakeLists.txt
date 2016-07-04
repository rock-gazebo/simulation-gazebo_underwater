rock_library(gazebo_underwater
    SOURCES GazeboUnderwater.cpp
    HEADERS GazeboUnderwater.hpp
    DEPS_PKGCONFIG gazebo base-types
    DEPS_CMAKE Boost)
