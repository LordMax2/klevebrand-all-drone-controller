cmake_minimum_required(VERSION 3.20)

project(klevebrand_all_drone_controller_core
    VERSION 1.9.2
    DESCRIPTION "Klevebrand all-drone controller core"
    LANGUAGES CXX
)

add_library(klevebrand_all_drone_controller_core STATIC
    src/base_drone.cpp
    src/base_drone_gyro.cpp
    src/base_drone_motor.cpp
    src/base_drone_position.cpp
    src/base_hardware_processor.cpp
    src/base_pid_repository.cpp
    src/flight_controller_proxy.cpp
    src/gyro_pid.cpp
    src/pid.cpp
    src/pid_optimizer.cpp
    src/pid_yaw_compass.cpp
)

add_library(Klevebrand::Core ALIAS klevebrand_all_drone_controller_core)

target_include_directories(klevebrand_all_drone_controller_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(klevebrand_all_drone_controller_core PUBLIC cxx_std_17)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

install(
    TARGETS klevebrand_all_drone_controller_core
    EXPORT KlevebrandAllDroneControllerCoreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
    EXPORT KlevebrandAllDroneControllerCoreTargets
    FILE KlevebrandAllDroneControllerCoreTargets.cmake
    NAMESPACE Klevebrand::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/KlevebrandAllDroneControllerCore
)

configure_package_config_file(
    cmake/KlevebrandAllDroneControllerCoreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/KlevebrandAllDroneControllerCoreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/KlevebrandAllDroneControllerCore
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/KlevebrandAllDroneControllerCoreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/KlevebrandAllDroneControllerCoreConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/KlevebrandAllDroneControllerCoreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/KlevebrandAllDroneControllerCore
)
