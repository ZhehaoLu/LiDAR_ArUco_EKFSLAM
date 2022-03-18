cmake_minimum_required(VERSION 3.9)
project(turtlelib)

# enable C++ 17
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

# CMake also has the ability to generate doxygen documentation
# set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
# doxygen_add_docs(dox include/ src/ README.md ALL)

find_package(catkin REQUIRED COMPONENTS catch_ros)
# find_package(catch_ros REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
#  DEPENDS system_lib
)

if (CATKIN_ENABLE_TESTING)
	add_test(NAME Test_of_Turtlelib COMMAND ${PROJECT_NAME}_test)

	catch_add_test(${PROJECT_NAME}_test
		tests/tests.cpp
	)
	target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

# create the turtlelib library 

add_library(${PROJECT_NAME} src/diff_drive.cpp src/rigid2d.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# The add_library function just added turtlelib as a "target"
# A "target" is a name that CMake uses to refer to some type of output
# In this case it is a library but it could also be an executable or some other items

# Public include directories can be used by other targets that link against turtlelib
# By adding include/ to the include path, this means that files in e.g., include/turtlelib
# can be included with #include"turtlelib/file.hpp"
target_include_directories(${PROJECT_NAME} PUBLIC include/)

# add_library(diff_drive_lib src/diff_drive.cpp)
# add_dependencies(diff_drive_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} $ {catkin_EXPORTED_TARGETS})
# target_include_directories(diff_drive_lib PUBLIC include/)


# create the executable target  and link it with the rigid2d library
# It is also possible specify multiple cpp files and they will be linked
# into a single executable (as long as exactly one of these files includes a main() function).
# However, by creating a library (as we are doing here) the library files
# can be compiled once and used

add_executable(frame_main src/frame_main.cpp)
# add_executable(${PROJECT_NAME}_test tests/tests.cpp)

# Use the cmake testing functionality. A test is just an executable.


target_link_libraries(frame_main ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(TARGETS frame_main RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
