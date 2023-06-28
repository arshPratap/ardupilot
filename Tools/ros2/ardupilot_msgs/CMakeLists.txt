cmake_minimum_required(VERSION 3.8)
project(ardupilot_msgs)

# --------------------------------------------------------------------------- #
# Find dependencies.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(builtin_interfaces REQUIRED)
# find_package(std_msgs REQUIRED)

# --------------------------------------------------------------------------- #
# Generate and export message interfaces.

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmMotors.srv"
  "srv/ModeSelect.srv"
  # DEPENDENCIES
  # builtin_interfaces
  # std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

# --------------------------------------------------------------------------- #
# Call last.

ament_package()
