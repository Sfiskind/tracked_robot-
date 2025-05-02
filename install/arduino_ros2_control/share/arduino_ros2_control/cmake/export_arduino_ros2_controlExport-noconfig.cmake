#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "arduino_ros2_control::arduino_ros2_control" for configuration ""
set_property(TARGET arduino_ros2_control::arduino_ros2_control APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(arduino_ros2_control::arduino_ros2_control PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libarduino_ros2_control.so"
  IMPORTED_SONAME_NOCONFIG "libarduino_ros2_control.so"
  )

list(APPEND _cmake_import_check_targets arduino_ros2_control::arduino_ros2_control )
list(APPEND _cmake_import_check_files_for_arduino_ros2_control::arduino_ros2_control "${_IMPORT_PREFIX}/lib/libarduino_ros2_control.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
