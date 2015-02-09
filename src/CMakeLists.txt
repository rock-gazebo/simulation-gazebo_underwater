rock_library(gazebo_underwater
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(gazebo_underwater_bin Main.cpp
    DEPS gazebo_underwater)

