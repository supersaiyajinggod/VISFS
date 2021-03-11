# Find the header files

find_path(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  ${G2O_ROOT}/include
  $ENV{G2O_ROOT}/include
  $ENV{G2O_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

macro(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)

  find_library("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ${G2O_ROOT}/lib/Debug
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Debug
    $ENV{G2O_ROOT}/lib
    /usr/lib
    /usr/local/lib
    NO_DEFAULT_PATH
    )

  find_library("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  find_library(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ${G2O_ROOT}/lib/Release
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Release
    $ENV{G2O_ROOT}/lib
    /usr/lib
    /usr/local/lib
    NO_DEFAULT_PATH
    )

  find_library(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )
  
  if(NOT ${MYLIBRARY}_DEBUG)
    if(MYLIBRARY)
      set(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    endif(MYLIBRARY)
  endif( NOT ${MYLIBRARY}_DEBUG)
  
endmacro(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)
FIND_G2O_LIBRARY(G2O_OPENGL_HELPER_LIBRARY opengl_helper)

# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)

# G2O solvers declared found if we found at least one solver
set(G2O_SOLVERS_FOUND "NO")
if(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
  set(G2O_SOLVERS_FOUND "YES")
endif(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)

# G2O itself declared found if we found the core libraries and at least one solver
set(G2O_FOUND "NO")
if(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
  set(G2O_FOUND "YES")
  set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
  set(G2O_LIBRARIES 
	${G2O_CORE_LIBRARY}
  ${G2O_OPENGL_HELPER_LIBRARY}
	${G2O_TYPES_SLAM2D} 
	${G2O_TYPES_SLAM3D}
	${G2O_TYPES_SBA}
  ${G2O_STUFF_LIBRARY})

  IF(CSPARSE_FOUND)
     SET(G2O_INCLUDE_DIRS 
        ${G2O_INCLUDE_DIRS} 
        ${CSPARSE_INCLUDE_DIR})
     SET(G2O_LIBRARIES
        ${G2O_LIBRARIES} 
	${G2O_SOLVER_CSPARSE} 
	${G2O_SOLVER_CSPARSE_EXTENSION}
	${CSPARSE_LIBRARY})
  ENDIF(CSPARSE_FOUND)

  IF(G2O_SOLVER_CHOLMOD)
    SET(G2O_INCLUDE_DIRS 
        ${G2O_INCLUDE_DIRS} 
        ${CHOLMOD_INCLUDE_DIR})
    SET(G2O_LIBRARIES 
	  ${G2O_LIBRARIES}
	  ${G2O_SOLVER_CHOLMOD}
	  ${CHOLMOD_LIB})
  ENDIF(G2O_SOLVER_CHOLMOD)
endif(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
# SET(G2O_FOUND "NO")
# IF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
#   SET(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
#   SET(G2O_LIBRARIES 
# 	${G2O_CORE_LIBRARY}
# 	${G2O_TYPES_SLAM2D} 
# 	${G2O_TYPES_SLAM3D}
# 	${G2O_TYPES_SBA}
#         ${G2O_STUFF_LIBRARY})
  
#   IF(CSPARSE_FOUND)
#      SET(G2O_INCLUDE_DIRS 
#         ${G2O_INCLUDE_DIRS} 
#         ${CSPARSE_INCLUDE_DIR})
#      SET(G2O_LIBRARIES
#         ${G2O_LIBRARIES} 
# 	${G2O_SOLVER_CSPARSE} 
# 	${G2O_SOLVER_CSPARSE_EXTENSION}
# 	${CSPARSE_LIBRARY})
#   ENDIF(CSPARSE_FOUND)

#   IF(G2O_SOLVER_CHOLMOD)
#     SET(G2O_INCLUDE_DIRS 
#         ${G2O_INCLUDE_DIRS} 
#         ${CHOLMOD_INCLUDE_DIR})
#     SET(G2O_LIBRARIES 
# 	  ${G2O_LIBRARIES}
# 	  ${G2O_SOLVER_CHOLMOD}
# 	  ${CHOLMOD_LIB})
#   ENDIF(G2O_SOLVER_CHOLMOD)

#   FILE(READ ${G2O_CONFIG_FILE} TMPTXT)
#   STRING(FIND "${TMPTXT}" "G2O_NUMBER_FORMAT_STR" matchres)
#   IF(${matchres} EQUAL -1)
#     MESSAGE(STATUS "Old g2o version detected with c++03 interface (config file: ${G2O_CONFIG_FILE}).")
#     SET(G2O_CPP11 0)
#   ELSE()
#     MESSAGE(WARNING "Latest g2o version detected with c++11 interface (config file: ${G2O_CONFIG_FILE}). Make sure g2o is built with \"-DBUILD_WITH_MARCH_NATIVE=OFF\" to avoid segmentation faults caused by Eigen.")
#     SET(G2O_CPP11 1)
# ENDIF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)

# set(G2O_FOUND "NO")
# if(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
#   set(G2O_FOUND "YES")
# endif(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
