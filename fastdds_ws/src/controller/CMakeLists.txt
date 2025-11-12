cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianParams.msg"
  "msg/ErrorMessage.msg"
  "msg/JointAngle.msg"
  "msg/JointParams.msg"
  "msg/LinearMoveParams.msg"
  "msg/Localization.msg"
  "msg/MobileBaseCmd.msg"
  "msg/MobileBaseState.msg"
  "msg/MoveMessage.msg"
  "msg/OdometryMessage.msg"
  "msg/RealTimeBodyJoints.msg"
  "msg/RealTimeBodyTcpCartesian.msg"
  "msg/TcpOffsetParams.msg"
  "msg/TcpPayloadParams.msg"
  "msg/TcpPoseParams.msg"

  "srv/EnableRealtimeCmd.srv"
  "srv/ErrorClear.srv"
  "srv/EnableController.srv"
  "srv/GetChassisMaxVel.srv"
  "srv/GetControlPolicy.srv"
  "srv/GetSafeMode.srv"
  "srv/IsEnabledController.srv"
  "srv/SetChassisMaxVel.srv"
  "srv/SetControlPolicy.srv"
  "srv/SetSafeMode.srv"

  "srv/ForwardKinematics.srv"
  "srv/GetCartesianMechanicalRotationMaxAcc.srv"
  "srv/GetCartesianMechanicalRotationMaxVel.srv"
  "srv/GetCartesianMechanicalTranslationMaxAcc.srv"
  "srv/GetCartesianMechanicalTranslationMaxVel.srv"
  "srv/GetCartesianRotationMaxAcc.srv"
  "srv/GetCartesianRotationMaxVel.srv"
  "srv/GetCartesianTranslationMaxAcc.srv"
  "srv/GetCartesianTranslationMaxVel.srv"
  "srv/GetJointMaxAcc.srv"
  "srv/GetJointMaxVel.srv"
  "srv/GetJointMechanicalLimit.srv"
  "srv/GetJointMechanicalMaxAcc.srv"
  "srv/GetJointMechanicalMaxVel.srv"
  "srv/GetJointSoftLimit.srv"
  "srv/GetTcpOffset.srv"
  "srv/GetTcpPayload.srv"
  "srv/GetVersion.srv"
  "srv/InverseKinematics.srv"
  "srv/LinearMove.srv"
  "srv/MultiJointMove.srv"
  "srv/SetCartesianRotationMaxAcc.srv"
  "srv/SetCartesianRotationMaxVel.srv"
  "srv/SetCartesianTranslationMaxAcc.srv"
  "srv/SetCartesianTranslationMaxVel.srv"
  "srv/SetJointMaxAcc.srv"
  "srv/SetJointMaxVel.srv"
  "srv/SetJointSoftLimit.srv"
  "srv/SetTcpOffset.srv"
  "srv/SetTcpPayload.srv"
  "srv/SingleJointMove.srv"

  DEPENDENCIES std_msgs geometry_msgs
)

if (DEFINED ENV{HUMANOID_DRIVER_ROOT})
  # 禁用系统安装
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Local install dir" FORCE)
  find_package(rosidl_generator_dds_idl REQUIRED)
  rosidl_generate_dds_interfaces(
    ${rosidl_generate_interfaces_TARGET}__dds_fastrtps_idl
    IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
    OUTPUT_SUBFOLDERS dds_fastrtps
  )
  add_dependencies(
    ${PROJECT_NAME}
    ${PROJECT_NAME}__dds_fastrtps_idl
  )

  set(SOURCES_IDL_PATH "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl/${PROJECT_NAME}/msg/dds_fastrtps/")
  set(INSTALL_IDL_PATH "$ENV{HUMANOID_CONTROLLER_ROOT}/idl/${PROJECT_NAME}/msg/dds_fastrtps")
  file(MAKE_DIRECTORY ${INSTALL_IDL_PATH})
  install(
    DIRECTORY ${SOURCES_IDL_PATH}
    DESTINATION ${INSTALL_IDL_PATH}
    FILES_MATCHING PATTERN "*.idl"
  )

  set(SOURCES_SRV_IDL_PATH "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl/${PROJECT_NAME}/srv/dds_fastrtps/")
  set(INSTALL_SRV_IDL_PATH "$ENV{HUMANOID_CONTROLLER_ROOT}/idl/${PROJECT_NAME}/srv/dds_fastrtps")
  file(MAKE_DIRECTORY ${INSTALL_SRV_IDL_PATH})
  install(
    DIRECTORY ${SOURCES_SRV_IDL_PATH}
    DESTINATION ${INSTALL_SRV_IDL_PATH}
    FILES_MATCHING PATTERN "*.idl"
  )
  
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()