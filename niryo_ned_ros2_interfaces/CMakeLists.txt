cmake_minimum_required(VERSION 3.8)
project(niryo_ned_ros2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  std_msgs
  geometry_msgs
  trajectory_msgs
  builtin_interfaces
  sensor_msgs
)

# Declare message files
set(ttl_driver_msgs
  "msg/ArrayMotorHardwareStatus.msg"
  "msg/CalibrationStatus.msg"
  "msg/MotorCommand.msg"
  "msg/MotorHardwareStatus.msg"
)

set(arm_commander_msgs
  "msg/ArmMoveCommand.msg"
  "msg/CommandJog.msg"
  "msg/JointLimits.msg"
  "msg/JointTrajectoryPoint.msg"
  "msg/JointTrajectory.msg"
  "msg/PausePlanExecution.msg"
  "msg/ShiftPose.msg"
)

set(robot_status_msgs
  "msg/RobotStatus.msg"
)

set(sound_msgs
  "msg/SoundList.msg"
  "msg/SoundObject.msg"
)

set(end_effector_panel_msgs
  "msg/EEButtonStatus.msg"
  "msg/EEIOState.msg"
)

set(conveyor_msgs
  "msg/ConveyorFeedback.msg"
  "msg/ConveyorFeedbackArray.msg"
)

set(database_msgs
  "msg/FilePath.msg"
  "msg/Setting.msg"
)

set(programs_manager_msgs
  "msg/Program.msg"
  "msg/ProgramList.msg"
)

set(rpi_msgs
  "msg/AnalogIO.msg"
  "msg/AnalogIOState.msg"
  "msg/DigitalIO.msg"
  "msg/DigitalIOState.msg"
  "msg/HotspotButtonStatus.msg"
  "msg/LogStatus.msg"
  "msg/I2CComponent.msg"
  "msg/StorageStatus.msg"
  "msg/WifiButtonStatus.msg"
)

set(tools_msgs
  "msg/Tool.msg"
  "msg/TCP.msg"
  "msg/ToolCommand.msg"
)

set(metrics_msgs
  "msg/Metric.msg"
)

set(reports_msgs
  "msg/Service.msg"
)

set(poses_handlers_msgs
  "msg/DynamicFrame.msg"
  "msg/NiryoPose.msg"
  "msg/Workspace.msg"
)

set(led_ring_msgs
  "msg/LedRingAnimation.msg"
  "msg/LedRingCurrentState.msg"
  "msg/LedRingStatus.msg"
)

set(user_interface_msgs
  "msg/ConnectionState.msg"
)

set(vision_msgs
  "msg/ActiveDebugTopics.msg"
  "msg/ImageParameters.msg"
)

set(program_player_msgs
  "msg/ButtonsStates.msg"
  "msg/DisplayMessage.msg"
)


set(msg_files
  "msg/BasicObject.msg"
  "msg/BasicObjectArray.msg"
  "msg/BusState.msg"
  "msg/CommandStatus.msg"
  "msg/HardwareStatus.msg"
  "msg/MotorHeader.msg"
  "msg/ObjectPose.msg"
  "msg/RobotState.msg"
  "msg/RPY.msg"
  "msg/SoftwareVersion.msg"
  ${ttl_driver_msgs}
  ${arm_commander_msgs}
  ${robot_status_msgs}
  ${sound_msgs}
  ${end_effector_panel_msgs}
  ${conveyor_msgs}
  ${database_msgs}
  ${programs_manager_msgs}
  ${rpi_msgs}
  ${tools_msgs}
  ${tools_commander_msgs}
  ${metrics_msgs}
  ${reports_msgs}
  ${poses_handlers_msgs}
  ${led_ring_msgs}
  ${user_interface_msgs}
  ${vision_msgs}
  ${program_player_msgs}
)

# Declare service files

set(joints_interface_srvs
  "srv/FactoryCalibration.srv"
)

set(ttl_driver_srvs
  "srv/ReadCustomValue.srv"
  "srv/ReadPIDValue.srv"
  "srv/ReadVelocityProfile.srv"
  "srv/WriteCustomValue.srv"
  "srv/WritePIDValue.srv"
  "srv/WriteVelocityProfile.srv"
)

set(arm_commander_srvs
  "srv/ComputeTrajectory.srv"
  "srv/GetFK.srv"
  "srv/GetIK.srv"
  "srv/GetJointLimits.srv"
  "srv/GetTrajectory.srv"
  "srv/JogShift.srv"
  "srv/ManageTrajectory.srv"
)

set(sound_srvs
  "srv/ManageSound.srv"
  "srv/PlaySound.srv"
  "srv/TextToSpeech.srv"
)

set(end_effector_panel_srvs
  "srv/SetEEDigitalOut.srv"
)

set(conveyor_srvs
  "srv/ControlConveyor.srv"
  "srv/GetHardwareId.srv"
  "srv/SetConveyor.srv"
)

set(database_srvs
  "srv/AddFilePath.srv"
  "srv/GetAllByType.srv"
  "srv/GetSettings.srv"
  "srv/RmFilePath.srv"
  "srv/SetSettings.srv"
)

set(programs_manager_srvs
  "srv/CreateProgram.srv"
  "srv/DeleteProgram.srv"
  "srv/GetProgram.srv"
  "srv/GetProgramAutorunInfos.srv"
  "srv/GetProgramList.srv"
  "srv/SetProgramAutorun.srv"
  "srv/UpdateProgram.srv"
)

set(rpi_srvs
  "srv/ChangeMotorConfig.srv"
  "srv/GetAnalogIO.srv"
  "srv/GetDigitalIO.srv"
  "srv/LedBlinker.srv"
  "srv/ScanI2CBus.srv"
  "srv/SetAnalogIO.srv"
  "srv/SetDigitalIO.srv"
  "srv/SetIOMode.srv"
  "srv/SetPullup.srv"
)

set(tools_srvs
  "srv/PingDxlTool.srv"
  "srv/ToolCommand.srv"
  "srv/SetTCP.srv"
)
set(metrics_srvs
  "srv/GetMetric.srv"
)

set(reports_srvs
  "srv/CheckConnection.srv"
  "srv/RunAutoDiagnosis.srv"
)


set(poses_handlers_srvs
  "srv/GetDynamicFrame.srv"
  "srv/GetTransformPose.srv"
  "srv/GetWorkspaceRatio.srv"
  "srv/ManagePose.srv"
  "srv/GetPose.srv"
  "srv/GetWorkspaceMatrixPoses.srv"
  "srv/GetWorkspaceRobotPoses.srv"
  "srv/ManageWorkspace.srv"
  "srv/GetTargetPose.srv"
  "srv/GetWorkspacePoints.srv"
  "srv/ManageDynamicFrame.srv"
)


set(led_ring_srvs
  "srv/LedUser.srv"
  "srv/SetLedColor.srv"
  "srv/GetLedColors.srv"
)

set(cpu_srvs
  "srv/GetCpuTemperature.srv"
)

set(vision_srvs
  "srv/DebugColorDetection.srv"
  "srv/DebugMarkers.srv"
  "srv/ObjDetection.srv"
  "srv/SetImageParameter.srv"
  "srv/TakePicture.srv"
  "srv/Visualization.srv"
  "srv/ActivateDebugTopics.srv"
)

set(srv_files
  "srv/AdvertiseShutdown.srv"
  "srv/GetBool.srv"
  "srv/GetInt.srv"
  "srv/GetNameDescriptionList.srv"
  "srv/GetString.srv"
  "srv/GetStringList.srv"
  "srv/Ping.srv"
  "srv/SetBool.srv"
  "srv/SetFloat.srv"
  "srv/SetInt.srv"
  "srv/SetString.srv"
  "srv/Trigger.srv"
  "srv/GetFloatList.srv"
  "srv/SetFloatList.srv"
  ${joints_interface_srvs}
  ${ttl_driver_srvs}
  ${arm_commander_srvs}
  ${sound_srvs}
  ${end_effector_panel_srvs}
  ${conveyor_srvs}
  ${database_srvs}
  ${programs_manager_srvs}
  ${rpi_srvs}
  ${tools_srvs}
  ${tools_commander_srvs}
  ${metrics_srvs}
  ${reports_srvs}
  ${poses_handlers_srvs}
  ${led_ring_srvs}
  ${cpu_srvs}
  ${vision_srvs}
)

# Declare action files

set(programs_manager_actions
  "action/ExecuteProgram.action"
)

set(arm_commander_actions
  "action/RobotMove.action"
)

set(tools_commander_actions
  "action/Tool.action"
)

set(action_files
  ${arm_commander_actions}
  ${tools_commander_actions}
  ${programs_manager_actions}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES ${dependencies}
)

ament_package()
