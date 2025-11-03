# Containing sdk cmake configuration
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
    "$ENV{GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT}"
)
include(Generic)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
    "$ENV{AMW_DEVELOPMENT_ENVIRONMENT_ROOT}/include"
    "$ENV{AMW_DEVELOPMENT_ENVIRONMENT_ROOT}/lib"
)
# Containing subdirectories
include(Amw_Inc)
include(Amw_Lib)

file(TO_CMAKE_PATH $ENV{AMW_RUNTIME_ENVIRONMENT_ROOT}/${GENERIC_LIBRARY_DIR} AMW_BIN_ROOT)

include(AmwMsg)
