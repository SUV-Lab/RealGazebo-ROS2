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

# Download large STL files if they don't exist
set(MESH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/models/c-track/meshes")
set(C_TRACK_STL "${MESH_DIR}/c-track.stl")
set(VILS_STL "${MESH_DIR}/vils.stl")

# Download c-track.stl if it doesn't exist
if(NOT EXISTS ${C_TRACK_STL})
  message(STATUS "Downloading c-track.stl...")
  file(DOWNLOAD
    "https://realgazebo.cbnu.ac.kr/file/c-track.stl"
    ${C_TRACK_STL}
    SHOW_PROGRESS
    STATUS download_status
  )
  list(GET download_status 0 status_code)
  if(NOT status_code EQUAL 0)
    message(WARNING "Failed to download c-track.stl")
  endif()
else()
  message(STATUS "c-track.stl already exists, skipping download")
endif()

# Download vils.stl if it doesn't exist
if(NOT EXISTS ${VILS_STL})
  message(STATUS "Downloading vils.stl...")
  file(DOWNLOAD
    "https://realgazebo.cbnu.ac.kr/file/vils.stl"
    ${VILS_STL}
    SHOW_PROGRESS
    STATUS download_status
  )
  list(GET download_status 0 status_code)
  if(NOT status_code EQUAL 0)
    message(WARNING "Failed to download vils.stl")
  endif()
else()
  message(STATUS "vils.stl already exists, skipping download")
endif()

# Install plugin
install(TARGETS RealGazebo
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
