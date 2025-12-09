cmake_minimum_required(VERSION 3.5)
project(cmake_utils)

find_package(ament_cmake REQUIRED)

# Install existing cmake files
install(FILES
  cmake/arch.cmake
  cmake/color.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY cmake_modules/
  DESTINATION share/${PROJECT_NAME}/cmake_modules
)

# Generate extras file
configure_file(cmake/cmake_utils-extras.cmake.in cmake_utils-extras.cmake COPYONLY)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cmake_utils-extras.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(
  CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/cmake_utils-extras.cmake"
)
