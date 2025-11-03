#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tinyxml2" for configuration ""
set_property(TARGET tinyxml2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(tinyxml2 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libtinyxml2.so.6.0.0"
  IMPORTED_SONAME_NOCONFIG "libtinyxml2.so.6"
  )

list(APPEND _cmake_import_check_targets tinyxml2 )
list(APPEND _cmake_import_check_files_for_tinyxml2 "${_IMPORT_PREFIX}/lib/libtinyxml2.so.6.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
