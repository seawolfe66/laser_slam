#                                               -*- cmake -*-
#
#  Eigen3Config.cmake(.in)

# Use the following variables to compile and link against Eigen:
#  EIGEN3_FOUND              - True if Eigen was found on your system
#  EIGEN3_USE_FILE           - The file making Eigen usable
#  EIGEN3_DEFINITIONS        - Definitions needed to build with Eigen
#  EIGEN3_INCLUDE_DIR        - Directory where signature_of_eigen3_matrix_library can be found
#  EIGEN3_INCLUDE_DIRS       - List of directories of Eigen and it's dependencies
#  EIGEN3_ROOT_DIR           - The base directory of Eigen
#  EIGEN3_VERSION_STRING     - A human-readable string containing the version
#  EIGEN3_VERSION_MAJOR      - The major version of Eigen
#  EIGEN3_VERSION_MINOR      - The minor version of Eigen
#  EIGEN3_VERSION_PATCH      - The patch version of Eigen


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was Eigen3ConfigLegacy.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

set ( EIGEN3_FOUND 1 )
set ( EIGEN3_USE_FILE     "${CMAKE_CURRENT_LIST_DIR}/UseEigen3.cmake" )

set ( EIGEN3_DEFINITIONS  "" )
set ( EIGEN3_INCLUDE_DIR  "${PACKAGE_PREFIX_DIR}/include/eigen3" )
set ( EIGEN3_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/eigen3" )
set ( EIGEN3_ROOT_DIR     "${PACKAGE_PREFIX_DIR}" )

set ( EIGEN3_VERSION_STRING "3.3.3" )
set ( EIGEN3_VERSION_MAJOR  "3" )
set ( EIGEN3_VERSION_MINOR  "3" )
set ( EIGEN3_VERSION_PATCH  "3" )
