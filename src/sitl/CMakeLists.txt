cmake_minimum_required(VERSION 3.5)
project(sitl)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

find_program(MAKE NAMES make REQUIRED)

externalproject_add(betaflight-firmware
  SOURCE_DIR
      ${CMAKE_SOURCE_DIR}/src
  PREFIX
      ${PROJECT_BINARY_DIR}/betaflight_sim
  INSTALL_COMMAND
    mkdir -p ${CMAKE_INSTALL_PREFIX}/share/betaflight_sim/bin/ &&
    cp ./obj/main/betaflight_SITL.elf ${CMAKE_INSTALL_PREFIX}/share/betaflight_sim/bin/
  BUILD_COMMAND pwd && ${MAKE} TARGET=SITL -j
  BUILD_IN_SOURCE ON
  CONFIGURE_COMMAND  ${MAKE} arm_sdk_install
)

ament_environment_hooks(cmake/setup.sh.in)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  worlds
  DESTINATION
    share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  config
  DESTINATION
    ${CMAKE_INSTALL_PREFIX}/share/betaflight_sim
)

ament_package()
