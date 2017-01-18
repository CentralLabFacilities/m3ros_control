# Antoine Hoarau <hoarau.robotics@gmail.com>
# deduce the libraries suffix from the options
set(FIND_M3_LIB_SUFFIX "")

# find the M3 include directory
find_path(M3_ROOT m3/chains/joint_array.h
	PATH_SUFFIXES include
		PATHS
		${M3_SRC}
    $ENV{M3_DEVEL}
		/opt/m3
		/usr/local/
		/usr)

#set(M3_ROOT ${M3_ROOT}/m3/)
message("!!!FindM3 ROOT IS ${M3_ROOT}")

# find the requested modules
#set(M3_FOUND TRUE) # will be set to false if one of the required modules is not found
set(FIND_M3_LIB_PATHS
    $ENV{M3_DEVEL}
		/opt/m3
		/usr/local
		/usr)

foreach(FIND_M3_COMPONENT ${M3_FIND_COMPONENTS})
    string(TOLOWER ${FIND_M3_COMPONENT} FIND_M3_COMPONENT_LOWER)
    set(FIND_M3_COMPONENT_NAME m3${FIND_M3_COMPONENT_LOWER}${FIND_M3_LIB_SUFFIX})
    set(M3_INCLUDE_DIR ${M3_INCLUDE_DIR} ${M3_ROOT}/m3/${FIND_M3_COMPONENT_LOWER})
    find_library(M3_${FIND_M3_COMPONENT} NAMES ${FIND_M3_COMPONENT_NAME} PATHS ${FIND_M3_LIB_PATHS} PATH_SUFFIXES lib)
    set(M3_LIBRARIES ${M3_LIBRARIES} ${M3_${FIND_M3_COMPONENT}})
endforeach()

if(M3_FOUND)
  list(APPEND M3_INCLUDE_DIR ${M3_INCLUDE_DIR} ${M3_ROOT})
  set(M3_INCLUDE_DIRS ${M3_INCLUDE_DIR})
endif(M3_FOUND)

MARK_AS_ADVANCED(
M3_FOUND
M3_INCLUDE_DIR
M3_LIBRARIES
)



