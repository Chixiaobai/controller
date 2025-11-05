#1. Specify the minimum version of cmake
cmake_minimum_required(VERSION 3.8)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#2. Specify the project name
PROJECT(h10w_controller_test)

#3. Specify the C++17_STANDARD
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#4. Containing h1_sdk_base cmake configuration
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}"
    "${CMAKE_CURRENT_SOURCE_DIR}/Environment"
)
# include(h1_sdk_base)
file(GLOB ENVIRONMENT_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/Environment/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Environment/*.h
)

set(THIRDPART_DIR ${CMAKE_CURRENT_LIST_DIR}/thirdparty)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller REQUIRED)
find_package(GTest REQUIRED)

find_package(absl REQUIRED PATHS "${THIRDPART_DIR}/grpc/lib/cmake/absl" NO_DEFAULT_PATH)
find_package(utf8_range REQUIRED PATHS "${THIRDPART_DIR}/grpc/lib/cmake/utf8_range" NO_DEFAULT_PATH)
find_package(Protobuf REQUIRED PATHS "${THIRDPART_DIR}/grpc/lib/cmake/protobuf" NO_DEFAULT_PATH)
find_package(gRPC REQUIRED PATHS "${THIRDPART_DIR}/grpc/lib/cmake/grpc" NO_DEFAULT_PATH)
set(GRPC_LIB gRPC::grpc++_reflection gRPC::grpc++ protobuf::libprotobuf)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 定义公共依赖集合
set(COMMON_DEPS
  rclcpp
  controller
)


# #7.Specify the head file directories
include_directories(
    ${TEST_BASE_INCLUDE_DIRECTORIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/Environment/
    ${THIRDPART_DIR}/grpc/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include_H10w
    ${CMAKE_CURRENT_SOURCE_DIR}/include_H10w/include()
    ${CMAKE_CURRENT_SOURCE_DIR}/include_H10w/grpc_msg/grpc_ws/monitor/
    ${CMAKE_CURRENT_SOURCE_DIR}/include_H10w/grpc_msg/grpc_ws/common/
    ${CMAKE_CURRENT_SOURCE_DIR}/include_H10w/grpc_msg/fastdds_ws/src/controller/
    ${CMAKE_CURRENT_SOURCE_DIR}/include_H10w/rpc_service
)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-fPIC)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")
endif()

#8. Specify the test source file
file(GLOB_RECURSE Test_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/H10_Controller/grpc/*/*.cpp)

file(GLOB_RECURSE H10w_SRC ${CMAKE_CURRENT_SOURCE_DIR}/include_H10w/*)

#11. Define the source file list variable
set(H10W_CONTROLLER_SRC
    ${ENVIRONMENT_SRC}
    ${H10w_SRC}
    ${Test_SRC}
)


add_executable(${PROJECT_NAME} ${H10W_CONTROLLER_SRC})

ament_target_dependencies(${PROJECT_NAME} ${COMMON_DEPS})
target_link_libraries(${PROJECT_NAME}
  ${GRPC_LIB}
  pugixml
  stdc++fs
  GTest::GTest
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()