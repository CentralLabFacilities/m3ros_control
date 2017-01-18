# Antoine Hoarau <hoarau.robotics@gmail.com>
# deduce the libraries suffix from the options
set(FIND_M3RT_LIB_SUFFIX "")

# find the M3RT include directory
message ("!!!M3RT_SRC: ${M3RT_SRC}")
find_path(M3RT_ROOT m3rt/base/toolbox.h
	PATH_SUFFIXES include
		PATHS
    ${M3RT_SRC}
    $ENV{M3_DEVEL}
		/opt/m3
		/usr/local/
		/usr)

message("!!!FindM3RT ROOT IS ${M3RT_ROOT}")

# find the requested modules
#set(M3RT_FOUND TRUE) # will be set to false if one of the required modules is not found
set(FIND_M3RT_LIB_PATHS
    $ENV{M3_DEVEL}
		/opt/m3
		/usr/local
		/usr)

set(FIND_M3RT_COMPONENT "base")
string(TOLOWER ${FIND_M3RT_COMPONENT} FIND_M3RT_COMPONENT_LOWER)
set(FIND_M3RT_COMPONENT_NAME m3${FIND_M3RT_COMPONENT_LOWER}${FIND_M3RT_LIB_SUFFIX})

list(APPEND M3RT_INCLUDE_DIR  ${M3RT_ROOT} ${M3RT_ROOT}/m3rt/${FIND_M3RT_COMPONENT_LOWER})
find_library(M3RT_${FIND_M3RT_COMPONENT} NAMES ${FIND_M3RT_COMPONENT_NAME} PATHS ${FIND_M3RT_LIB_PATHS} PATH_SUFFIXES lib)

list(APPEND M3RT_LIBRARIES ${M3RT_${FIND_M3RT_COMPONENT}})

MARK_AS_ADVANCED(
M3RT_FOUND
M3RT_INCLUDE_DIRS
M3RT_LIBRARIES
)


if(M3RT_FOUND)
  set(M3RT_INCLUDE_DIRS ${M3RT_INCLUDE_DIR})
endif(M3RT_FOUND)
