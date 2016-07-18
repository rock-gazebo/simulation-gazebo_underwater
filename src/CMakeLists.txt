rock_library(gazebo_underwater
    SOURCES GazeboUnderwater.cpp
    HEADERS GazeboUnderwater.hpp DataTypes.hpp
    DEPS_PKGCONFIG gazebo
    DEPS_CMAKE Boost)
