cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)

add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(talkerNodeTest test/test.launch test/main.cpp test/talkerTest.cpp)
  target_link_libraries(talkerNodeTest ${catkin_LIBRARIES})
  add_dependencies(talker talkerNodeTest ${catkin_EXPORTED_TARGETS})
endif()

