# Set target platform variable
cmake_minimum_required(VERSION 2.8.12)
option(ARCHITECTURE "define process architecture" OFF)
option(REQUIRE_ADMINISTRATOR_ON_WIN32 "require administrator privileges" OFF)

if (NOT(${ARCHITECTURE} MATCHES OFF))
    set(GENERIC_PROCESSOR_ARCHITECTURE ${ARCHITECTURE})
else ()
    set(GENERIC_PROCESSOR_ARCHITECTURE $ENV{ARCHITECTURE})
endif ()
option(TARGET_PLATFORM "define target platform" OFF)
if (NOT(${TARGET_PLATFORM} MATCHES OFF))
    set(GENERIC_TARGET_PLATFORM ${TARGET_PLATFORM})
else ()
    if (WIN32)
        if (CMAKE_CL_64)
            set(GENERIC_TARGET_PLATFORM x64)
        else ()
            set(GENERIC_TARGET_PLATFORM x86)
        endif ()
    else ()
        set(GENERIC_TARGET_PLATFORM $ENV{PLATFORM})
    endif ()
endif ()

if (WIN32)
    set(CMAKE_OBJECT_PATH_MAX  500)
endif()

# Specify postfix for debug output
set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_RELEASE_POSTFIX "")
set(CMAKE_RELWITHDEBINFO_POSTFIX "rd")
set(CMAKE_MINSIZEREL_POSTFIX "s")

# Remove RPATH in dynamic library, If not do this, the library will be different when it generated from different position.
set(CMAKE_SKIP_BUILD_RPATH TRUE)

# Specify the build postfix extension according to what configuration is being built.
if (CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_RELEASE_POSTFIX}")
elseif (CMAKE_BUILD_TYPE MATCHES "MinSizeRel")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_MINSIZEREL_POSTFIX}")
elseif(CMAKE_BUILD_TYPE MATCHES "RelWithDebInfo")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}")
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_BUILD_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
else()
    set(CMAKE_BUILD_POSTFIX "")
endif()

# add c++11 support,icu required
if (WIN32)
else ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_compile_options(-fvisibility=hidden)
endif ()

# Specify across compile
option(ACROSS_C_COMPILER "Set across c compile" OFF)
if (NOT(${ACROSS_C_COMPILER} MATCHES OFF))
SET(CMAKE_C_COMPILER ${ACROSS_C_COMPILER})
message("CMAKE_C_COMPILER=${ACROSS_C_COMPILER}")
endif()
option(ACROSS_CXX_COMPILER "Set across c++ compile" OFF)
if (NOT(${ACROSS_CXX_COMPILER} MATCHES OFF))
SET(CMAKE_CXX_COMPILER ${ACROSS_CXX_COMPILER})
message("CMAKE_CXX_COMPILER=${ACROSS_CXX_COMPILER}")
endif()
option(SYS_ROOT "define sys root folder" OFF)
if (NOT(${SYS_ROOT} MATCHES OFF))
    set(CMAKE_SYSROOT ${SYS_ROOT})
    message("CMAKE_SYSROOT = ${CMAKE_SYSROOT}")
endif ()

# Detected xenomai
if (UNIX)
    if (NOT DEFINED USE_XENOMAI_ENV)
        if (NOT DEFINED ENV{USE_XENOMAI_ENV})
            set(USE_XENOMAI_ENV 0)
        else ()
            set(USE_XENOMAI_ENV $ENV{USE_XENOMAI_ENV})
        endif()
        if (${USE_XENOMAI_ENV} EQUAL 1)
            if (NOT DEFINED XENOMAI_SKIN)
                set(XENOMAI_SKIN cobalt)
            endif()
            if (NOT DEFINED ENV{XENOMAI_ROOT})
                message(FATAL_ERROR "You do not set XENOMAI_ROOT env!")
            endif()
            if (NOT EXISTS "$ENV{XENOMAI_ROOT}")
                message(FATAL_ERROR "Can not found xenomai root: $ENV{XENOMAI_ROOT}!")
            endif()
            message(STATUS "Xenomai environment has detected!")
            set(XENOMAI_ROOT $ENV{XENOMAI_ROOT})
            add_definitions(-D_XENOMAI_)
            set(XENOMAI_CFLAGS_PARAMS "--skin=${XENOMAI_SKIN}" "--cflags")
            execute_process(
                COMMAND $ENV{XENOMAI_ROOT}/bin/xeno-config ${XENOMAI_CFLAGS_PARAMS}
                OUTPUT_VARIABLE XENOMAI_CFLAGS
                OUTPUT_STRIP_TRAILING_WHITESPACE)
            if ("${XENOMAI_INIT_REQUIRED}" MATCHES "1")
                set(XENOMAI_LDFLAGS_PARAMS "--skin=${XENOMAI_SKIN}" "--ldflags")
            else ()
                set(XENOMAI_LDFLAGS_PARAMS "--skin=${XENOMAI_SKIN}" "--no-auto-init" "--ldflags")
            endif()
            execute_process(
                COMMAND $ENV{XENOMAI_ROOT}/bin/xeno-config ${XENOMAI_LDFLAGS_PARAMS}
                OUTPUT_VARIABLE XENOMAI_LDFLAGS
                OUTPUT_STRIP_TRAILING_WHITESPACE)
            execute_process(
                COMMAND $ENV{XENOMAI_ROOT}/bin/xeno-config --version
                OUTPUT_VARIABLE XENOMAI_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE)
            set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENOMAI_CFLAGS}")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENOMAI_CFLAGS}")
            set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENOMAI_LDFLAGS}")
            set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENOMAI_LDFLAGS}")
        endif()
    endif()
endif()

# Specify flags for compile
if (WIN32)
else ()
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} $ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb ")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} $ENV{CXXFLAGS} -O3 -Wall")
    if (GENERIC_TARGET_PLATFORM MATCHES "x86")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
    endif()
endif ()

file(TO_CMAKE_PATH $ENV{GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT} GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
    "${GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT}/include"
    "${GENERIC_DEVELOPMENT_ENVIRONMENT_ROOT}/lib"
)

# Containing subdirectories
include(Generic_Inc)
include(Generic_Lib)
include(PkgVersion)

file(TO_CMAKE_PATH $ENV{GENERIC_RUNTIME_ENVIRONMENT_ROOT} GENERIC_RUNTIME_ENVIRONMENT_ROOT)
set(GENERIC_BIN_ROOT ${GENERIC_RUNTIME_ENVIRONMENT_ROOT}/${GENERIC_LIBRARY_DIR})

# Specify macro
if (CMAKE_BUILD_TYPE MATCHES "Release")
    add_definitions(-DNDEBUG)
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_definitions(-DDEBUG -D_DEBUG)
else()
endif()
add_definitions(-D__STRICT_ANSI__ -D_USE_MATH_DEFINES)

if(WIN32)
    if(SUBSYSTEM_WINDOWS)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /subsystem:windows")
    else()
        add_definitions(-D_CONSOLE)
    endif(SUBSYSTEM_WINDOWS)
endif(WIN32)

if (${REQUIRE_ADMINISTRATOR} MATCHES ON)
    if(WIN32)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /MANIFESTUAC:level='requireAdministrator'")
    endif(WIN32)
endif()

# define function
function(add_generic_executable source_list)
    add_executable(${PROJECT_NAME} ${ARGV})
    set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
    set_target_properties(${PROJECT_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
    set_target_properties(${PROJECT_NAME} PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
endfunction()

