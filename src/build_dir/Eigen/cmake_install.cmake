# Install script for directory: /home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/SuperLUSupport"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Cholesky"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Dense"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/StdDeque"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/QtAlignedMalloc"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Eigenvalues"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/PaStiXSupport"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/UmfPackSupport"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/OrderingMethods"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/PardisoSupport"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Core"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Eigen"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Householder"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/SparseCholesky"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/CholmodSupport"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/SparseCore"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Geometry"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/SPQRSupport"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Jacobi"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/Sparse"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/SparseQR"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/StdList"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/SVD"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/SparseLU"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/IterativeLinearSolvers"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/MetisSupport"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/LU"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/QR"
    "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/StdVector"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/chenrui/catkin_ws/src/build_dir/source_dir/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")

