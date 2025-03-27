cmake_minimum_required(VERSION 3.11)
project(ow_fs_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# ROS dependecies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    ow_core
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ow_fs_planner
    CATKIN_DEPENDS
        roscpp
        ow_core
#    DEPENDS ${DEPENDS_VAR}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



set(SOURCES
    src/single_step_planner.cpp
    src/footstep_polynomial.cpp
    src/footstep_planner.cpp
)

# add executables
add_library(${PROJECT_NAME}  ${SOURCES})

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

# link target against libs
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#set(APP_TARGETS
#    ${PROJECT_NAME}_test1
#)

#foreach(target ${APP_TARGETS})
#        add_executable(${target} src/applications/main_${target}.cpp)
#        add_dependencies(${target}
#            ${${PROJECT_NAME}_EXPORTED_TARGETS}
#            ${catkin_EXPORTED_TARGETS})

#        target_link_libraries(${target}
#            ${PROJECT_NAME}
#            ${catkin_LIBRARIES}
#        )
#endforeach()

# don't install private headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*_p.h" EXCLUDE
    PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )



#############
## Testing ##
#############

#set(TEST_TARGETS
#  footstep_planner
#)

#foreach(target ${TEST_TARGETS})
#  catkin_add_gtest(${target} src/tests/test_${target}.cpp)
#  add_dependencies(${target}
#      ${${PROJECT_NAME}_EXPORTED_TARGETS}
#      ${catkin_EXPORTED_TARGETS})

#  set_target_properties(${target} PROPERTIES CXX_STANDARD 11)

#  target_link_libraries(${target}
#      ${PROJECT_NAME}
#      ${catkin_LIBRARIES}
#  )
#endforeach()
