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
  
  "srv/SystemOperatingModeAcquisitionMsg.srv"
  "srv/CommunicationBaudRateSettingMsg.srv"
  "srv/AbsolutePositionClosedLoopControlMsg.srv"
  "srv/SystemBrakeLockMsg.srv"
  "srv/TorqueClosedLoopControlMsg.srv"
  "srv/SystemRuntimeReadMsg.srv"
  "srv/ReadPIDParamsMsg.srv"
  "srv/WritePIDParamsRAMMsg.srv"
  "srv/IncrementalPositionClosedLoopControlMsg.srv"
  "srv/ReadSingleTurnEncoderMsg.srv"
  "srv/MotorStopMsg.srv"
  "srv/ReadMotorStatus3Msg.srv"
  "srv/ReadAccelerationMsg.srv"
  "srv/WritePIDParamsROMMsg.srv"
  "srv/SystemBrakeReleaseMsg.srv"
  "srv/ReadMultiTurnEncoderOriginalPositionMsg.srv"
  "srv/ReadMultiTurnEncoderPositionMsg.srv"
  "srv/ReadMotorStatus2Msg.srv"
  "srv/MotorPowerAcquisitionMsg.srv"
  "srv/MotorShutdownMsg.srv"
  "srv/CommunicationInterruptionProtectionTimeSettingMsg.srv"
  "srv/SystemSoftwareVersionDateReadMsg.srv"
  "srv/ReadMultiTurnAngleMsg.srv"
  "srv/SystemResetMsg.srv"
  "srv/ReadMotorStatus1Msg.srv"
  "srv/WriteCurrentMultiTurnZeroMsg.srv"
  "srv/MotorModelReadingMsg.srv"
  "srv/CANIDSettingMsg.srv"
  "srv/SpeedClosedLoopControlMsg.srv"
  "srv/WriteAccelerationRAMROMMsg.srv"
  "srv/SingleTurnPositionControlMsg.srv"
  "srv/ReadSingleTurnAngleMsg.srv"
  "srv/ReadMultiTurnEncoderZeroOffsetMsg.srv"
  "srv/WriteEncoderMultiTurnZeroMsg.srv"
  DEPENDENCIES std_msgs 
)

ament_package()
