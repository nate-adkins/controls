cmake_minimum_required(VERSION 3.8)
project(controls_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanMessage.msg"
  "msg/UartMessage.msg"
  "srv/CanSendRecv.srv"
  "srv/UartSendRecv.srv"
  
  "myactuator_srvs/SystemOperatingModeAcquisitionMsg.srv"
  "myactuator_srvs/CommunicationBaudRateSettingMsg.srv"
  "myactuator_srvs/AbsolutePositionClosedLoopControlMsg.srv"
  "myactuator_srvs/SystemBrakeLockMsg.srv"
  "myactuator_srvs/TorqueClosedLoopControlMsg.srv"
  "myactuator_srvs/SystemRuntimeReadMsg.srv"
  "myactuator_srvs/ReadPIDParamsMsg.srv"
  "myactuator_srvs/WritePIDParamsRAMMsg.srv"
  "myactuator_srvs/IncrementalPositionClosedLoopControlMsg.srv"
  "myactuator_srvs/ReadSingleTurnEncoderMsg.srv"
  "myactuator_srvs/MotorStopMsg.srv"
  "myactuator_srvs/ReadMotorStatus3Msg.srv"
  "myactuator_srvs/ReadAccelerationMsg.srv"
  "myactuator_srvs/WritePIDParamsROMMsg.srv"
  "myactuator_srvs/SystemBrakeReleaseMsg.srv"
  "myactuator_srvs/ReadMultiTurnEncoderOriginalPositionMsg.srv"
  "myactuator_srvs/ReadMultiTurnEncoderPositionMsg.srv"
  "myactuator_srvs/ReadMotorStatus2Msg.srv"
  "myactuator_srvs/MotorPowerAcquisitionMsg.srv"
  "myactuator_srvs/MotorShutdownMsg.srv"
  "myactuator_srvs/CommunicationInterruptionProtectionTimeSettingMsg.srv"
  "myactuator_srvs/SystemSoftwareVersionDateReadMsg.srv"
  "myactuator_srvs/ReadMultiTurnAngleMsg.srv"
  "myactuator_srvs/SystemResetMsg.srv"
  "myactuator_srvs/ReadMotorStatus1Msg.srv"
  "myactuator_srvs/WriteCurrentMultiTurnZeroMsg.srv"
  "myactuator_srvs/MotorModelReadingMsg.srv"
  "myactuator_srvs/CANIDSettingMsg.srv"
  "myactuator_srvs/SpeedClosedLoopControlMsg.srv"
  "myactuator_srvs/WriteAccelerationRAMROMMsg.srv"
  "myactuator_srvs/SingleTurnPositionControlMsg.srv"
  "myactuator_srvs/ReadSingleTurnAngleMsg.srv"
  "myactuator_srvs/ReadMultiTurnEncoderZeroOffsetMsg.srv"
  "myactuator_srvs/WriteEncoderMultiTurnZeroMsg.srv"
  DEPENDENCIES std_msgs 
)

ament_package()
