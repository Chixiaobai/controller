# Containing sdk cmake configuration
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
    "$ENV{GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT}"
)
include(Generic)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
    "$ENV{ETHERCAT_DEVELOPMENT_ENVIRONMENT_ROOT}/include"
    "$ENV{ETHERCAT_DEVELOPMENT_ENVIRONMENT_ROOT}/lib"
)
# Containing subdirectories
include(Ethercat_Inc)
include(Ethercat_Lib)

set(ETHERCAT_BIN_ROOT $ENV{ETHERCAT_RUNTIME_ENVIRONMENT_ROOT}/${GENERIC_LIBRARY_DIR})

