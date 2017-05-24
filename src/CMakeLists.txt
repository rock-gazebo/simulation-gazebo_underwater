find_package(Boost 1.40 COMPONENTS system REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

foreach(ITR ${GAZEBO_INCLUDE_DIRS})
    if(ITR MATCHES ".*gazebo-[0-9]+$")
        set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    endif()
endforeach()

protobuf_generate_cpp(msgs_SRCS msgs_HDR msgs.proto
    ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
    ${PROTOBUF_IMPORT_DIRS}/header.proto
    ${PROTOBUF_IMPORT_DIRS}/time.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

rock_library(gazebo_underwater_msgs
    SOURCES  ${msgs_SRCS}
    HEADERS ${msgs_HDR} DataTypes.hpp
    DEPS_PKGCONFIG protobuf)

rock_library(gazebo_underwater
    SOURCES GazeboUnderwater.cpp
    HEADERS GazeboUnderwater.hpp
    DEPS_PKGCONFIG gazebo
    DEPS gazebo_underwater_msgs
    DEPS_CMAKE Boost)

#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/msgs.pb.h
#    DESTINATION include/${PROJECT_NAME})

install(FILES msgs.proto
    DESTINATION include/${PROJECT_NAME}/proto)
