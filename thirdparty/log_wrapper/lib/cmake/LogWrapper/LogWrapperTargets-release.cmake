#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LogWrapper::LogWrapper" for configuration "Release"
set_property(TARGET LogWrapper::LogWrapper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(LogWrapper::LogWrapper PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libLogWrapper.so"
  IMPORTED_SONAME_RELEASE "libLogWrapper.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS LogWrapper::LogWrapper )
list(APPEND _IMPORT_CHECK_FILES_FOR_LogWrapper::LogWrapper "${_IMPORT_PREFIX}/lib/libLogWrapper.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
