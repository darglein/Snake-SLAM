# Locates the EigenRecursive library
# This module defines:
# EIGENRECURSIVE_INCLUDE_DIRS


find_path(EIGENRECURSIVE_INCLUDE_DIRS
	NAMES 
                EigenRecursive/Core.h
	PATHS
          /usr/local/include
          /usr/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EigenRecursive DEFAULT_MSG EIGENRECURSIVE_INCLUDE_DIRS)


add_library(EigenRecursive INTERFACE IMPORTED)
set_target_properties(EigenRecursive PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${EIGENRECURSIVE_INCLUDE_DIRS}"
)


mark_as_advanced(EIGENRECURSIVE_INCLUDE_DIRS)
