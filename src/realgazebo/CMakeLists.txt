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
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

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

# Download large STL files if they don't exist
set(MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models/c-track/meshes")
set(C_TRACK_STL "${MESH_DIR}/c-track.stl")
set(VILS_STL "${MESH_DIR}/vils.stl")
set(URBAN_STL "${MESH_DIR}/urban.stl")
set(BASE_URL "https://realgazebo.cbnu.ac.kr/download/${MODEL_VERSION}")

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

# Download c-track.stl with verification
download_with_verification("c-track.stl")

# Download vils.stl with verification
download_with_verification("vils.stl")

# Download urban.stl with verification
download_with_verification("urban.stl")

# Install plugins
install(TARGETS RealGazebo MotorFailureROS2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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
