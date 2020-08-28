cmake_minimum_required(VERSION 3.0.2)
project(kinematics)

find_package(catkin REQUIRED COMPONENTS
    pybind11_catkin
    rospy
    trac_ik_lib
    tf_conversions
)

find_package(orocos_kdl REQUIRED)

catkin_python_setup()

include_directories(${PROJECT_SOURCE_DIR}
                    ${PYTHON_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

include_directories(${orocos_kdl_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

set(KDL_SOURCE_DIR "src/kinematics/kdl_pybind")

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${catkin_INCLUDE_DIRS})
include_directories(${KDL_SOURCE_DIR})
include_directories(${TRAC_IK_SOURCE_DIR})

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS pybind11_catkin
                   rospy
                   trac_ik_lib
                   tf_conversions
)

catkin_install_python(PROGRAMS src/kinematics.py
                               src/kinematics/kdl_pybind/__init__.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

pybind_add_module(kdl_pybind MODULE "${KDL_SOURCE_DIR}/kdl_pybind.cpp")

target_link_libraries(kdl_pybind PRIVATE ${catkin_LIBRARIES})

install(TARGETS kdl_pybind
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)