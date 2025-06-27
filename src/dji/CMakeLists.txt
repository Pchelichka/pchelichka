cmake_minimum_required(VERSION 3.8)
project(dji)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(dji src/dji.cpp)

target_include_directories(dji PUBLIC
  include
  ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(dji
  ${LIBUSB_LIBRARIES}
)

target_compile_features(dji PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(dji
  rclcpp
)

install(TARGETS dji
  DESTINATION lib/${PROJECT_NAME})

ament_package()
