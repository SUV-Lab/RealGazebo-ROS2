cmake_minimum_required(VERSION 3.8)
project(realgazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realgazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Build RealGazebo plugin
add_library(RealGazebo SHARED
  plugins/realgazebo/RealGazebo.cpp
)

target_include_directories(RealGazebo PRIVATE
  plugins/realgazebo
)

target_link_libraries(RealGazebo
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
)

ament_target_dependencies(RealGazebo
  rclcpp
  px4_msgs
)

# Build MotorFailureROS2 plugin
add_library(MotorFailureROS2 SHARED
  plugins/motor_failure_ros2/MotorFailureSystem.cpp
)

target_include_directories(MotorFailureROS2 PRIVATE
  plugins/motor_failure_ros2
)

target_link_libraries(MotorFailureROS2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
)

ament_target_dependencies(MotorFailureROS2
  rclcpp
  std_msgs
)

# Build ServoFailureROS2 plugin
add_library(ServoFailureROS2 SHARED
  plugins/servo_failure_ros2/ServoFailureSystem.cpp
)

target_include_directories(ServoFailureROS2 PRIVATE
  plugins/servo_failure_ros2
)

target_link_libraries(ServoFailureROS2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
)

ament_target_dependencies(ServoFailureROS2
  rclcpp
  std_msgs
)

# Build ImuFaultROS2 plugin
add_library(ImuFaultROS2 SHARED
  plugins/imu_fault_ros2/ImuFaultSystem.cpp
)

target_include_directories(ImuFaultROS2 PRIVATE
  plugins/imu_fault_ros2
  plugins/fault_core
)

target_link_libraries(ImuFaultROS2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
)

ament_target_dependencies(ImuFaultROS2
  rclcpp
  std_msgs
  realgazebo_msgs
)

# Get git version tag
execute_process(
  COMMAND git describe --tags --always
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

# Parse to get clean tag (v1.0-5-g1183749 -> v1.0)
string(REGEX REPLACE "-[0-9]+-g[0-9a-f]+$" "" MODEL_VERSION "${GIT_VERSION}")

if(NOT MODEL_VERSION)
  set(MODEL_VERSION "latest")
  message(WARNING "Could not determine git version, using: ${MODEL_VERSION}")
else()
  message(STATUS "Using model version: ${MODEL_VERSION}")
endif()

# --- c-track terrain meshes -------------------------------------------------
# The c-track asset pack (c-track.stl 1.15 GB + vils.stl + urban.stl, ~1.25 GB
# total) is the terrain this project ships. vils/urban are deliberate crops of
# the same c-track site, kept so a small-scale run does not have to load the
# full 1.15 GB mesh into Gazebo; they are selected at launch with `terrain:=`,
# NOT `world:=` (see launch/gazebo.launch.py).
#
# ON  (default): fetch all three. Right for anyone actually flying at c-track.
# OFF: skip the download entirely. Set this when you only want the RealGazebo
#      framework (plugins, manager, PX4 wiring) and bring your own world -
#      drop worlds/<name>.sdf in and launch with `world:=<name>`. Your world
#      never includes model://c-track, so the meshes are dead weight: they cost
#      ~1.25 GB of download, disk, and (via docker/Dockerfile) image size.
#
# Deliberately all-or-nothing rather than a per-mesh list: the three are one
# asset pack for one site, and a partial set only produces a launch that fails
# later, at `terrain:=`, instead of here.
option(WORLD_CTRACK "Download the c-track terrain meshes (OFF: bring your own world)" ON)

# Download large STL files if they don't exist.
# MAKE_DIRECTORY is required, not defensive: every mesh here is gitignored, so
# models/c-track/meshes/ has no tracked file and git does not create it on a
# fresh clone. Without this, curl has nowhere to write and fails with
# "(23) Failure writing output to destination" - which only ever reproduces on
# a clean checkout, never in a working tree that already has the meshes.
set(MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models/c-track/meshes")
file(MAKE_DIRECTORY "${MESH_DIR}")
set(C_TRACK_STL "${MESH_DIR}/c-track.stl")
set(VILS_STL "${MESH_DIR}/vils.stl")
set(URBAN_STL "${MESH_DIR}/urban.stl")
set(BASE_URL "https://realgazebo.cbnu.ac.kr/download/${MODEL_VERSION}")

# Fall back to 'latest' when this version has no mesh directory on the
# server yet (e.g. a fresh tag before its release upload). Meshes change
# rarely, so the newest published set is the right default - and a WARN
# beats every checkout failing FATAL on a missing path. Probed once via a
# known checksum file. Skipped with WORLD_CTRACK=OFF: no download means no
# reason to reach the mesh server at all (and no reason for a build behind a
# firewall to stall on it).
if(WORLD_CTRACK)
  execute_process(
    COMMAND curl -fsI --retry 3 --retry-delay 2 --connect-timeout 10
            "${BASE_URL}/c-track.stl.md5"
    RESULT_VARIABLE version_probe
    OUTPUT_QUIET
    ERROR_QUIET
  )
  if(NOT version_probe EQUAL 0)
    message(WARNING
      "No mesh directory for '${MODEL_VERSION}' on the server; "
      "falling back to 'latest'")
    set(MODEL_VERSION "latest")
    set(BASE_URL "https://realgazebo.cbnu.ac.kr/download/latest")
  endif()
endif()

# Function to download file with MD5 verification (using curl with retry)
function(download_with_verification filename)
  set(FILE_PATH "${MESH_DIR}/${filename}")
  set(MD5_FILE "${CMAKE_CURRENT_BINARY_DIR}/${filename}.md5")
  set(FILE_URL "${BASE_URL}/${filename}")
  set(MD5_URL "${BASE_URL}/${filename}.md5")

  # Download MD5 checksum file using curl with retry
  execute_process(
    COMMAND curl -fsSL --retry 5 --retry-delay 3 --retry-max-time 60 -o ${MD5_FILE} ${MD5_URL}
    RESULT_VARIABLE md5_code
    ERROR_QUIET
  )

  if(NOT md5_code EQUAL 0)
    message(FATAL_ERROR "Failed to download ${filename}.md5 from ${BASE_URL}")
  endif()

  # Check if we need to download
  set(NEED_DOWNLOAD FALSE)
  if(NOT EXISTS ${FILE_PATH})
    message(STATUS "${filename} not found, downloading...")
    set(NEED_DOWNLOAD TRUE)
  else()
    # Compare checksums
    file(MD5 ${FILE_PATH} LOCAL_MD5)
    file(READ ${MD5_FILE} SERVER_MD5)
    string(STRIP "${SERVER_MD5}" SERVER_MD5)
    string(REGEX REPLACE "^([a-f0-9]+).*" "\\1" SERVER_MD5 "${SERVER_MD5}")

    if(NOT "${LOCAL_MD5}" STREQUAL "${SERVER_MD5}")
      message(STATUS "${filename} checksum mismatch (local: ${LOCAL_MD5}, server: ${SERVER_MD5})")
      message(STATUS "Re-downloading ${filename}...")
      set(NEED_DOWNLOAD TRUE)
    else()
      message(STATUS "${filename} checksum verified, skipping download")
    endif()
  endif()

  # Download file if needed using curl with retry
  if(NEED_DOWNLOAD)
    message(STATUS "Downloading ${filename} (with retry)...")
    execute_process(
      COMMAND curl -fSL --retry 5 --retry-delay 5 --retry-max-time 300
                   --connect-timeout 30 -o ${FILE_PATH} ${FILE_URL}
      RESULT_VARIABLE status_code
    )
    if(NOT status_code EQUAL 0)
      message(FATAL_ERROR "Failed to download ${filename} from ${BASE_URL}")
    else()
      message(STATUS "${filename} downloaded successfully")
    endif()
  endif()

  # Clean up temporary MD5 file
  file(REMOVE ${MD5_FILE})
endfunction()

if(WORLD_CTRACK)
  # Download c-track.stl with verification
  download_with_verification("c-track.stl")

  # Download vils.stl with verification
  download_with_verification("vils.stl")

  # Download urban.stl with verification
  download_with_verification("urban.stl")
else()
  message(STATUS
    "WORLD_CTRACK=OFF: skipping the c-track terrain meshes (~1.25 GB). "
    "worlds/c-track.sdf cannot be launched in this build - supply your own "
    "worlds/<name>.sdf and launch with world:=<name>.")
endif()

# Build image_receiver_node
add_executable(image_receiver_node
  nodes/image_receiver/ImageReceiver.cpp
)

target_include_directories(image_receiver_node PRIVATE
  nodes/image_receiver
)

ament_target_dependencies(image_receiver_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
)

target_link_libraries(image_receiver_node
  ${OpenCV_LIBS}
)

install(TARGETS RealGazebo MotorFailureROS2 ServoFailureROS2 ImuFaultROS2 image_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Shared data: launch files, models, worlds, vehicle yaml configs
install(DIRECTORY launch models worlds yaml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins
  DESTINATION lib/${PROJECT_NAME}
)

# Python manager: install the realgazebo python module and the node entry point
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/manager_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Fault waveform maths — pure C++, no simulator needed.
  ament_add_gtest(test_fault_channel_model
    test/test_fault_channel_model.cpp
  )
  target_include_directories(test_fault_channel_model PRIVATE
    plugins/fault_core
  )

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Python unit tests for the manager modules
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_python_tests test
    TIMEOUT 120
  )
endif()

ament_package()
