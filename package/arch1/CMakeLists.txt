include_directories(${ROS_LIB_DIR})

FILE(GLOB_RECURSE ros_src
    "${ROS_LIB_DIR}/*.cpp"
    "${ROS_LIB_DIR}/*.h")
add_library(ros_lib ${ros_src})

add_teensy_executable(target1 main.cpp)
target_link_libraries(target1 ros_lib)