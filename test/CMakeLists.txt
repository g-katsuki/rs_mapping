cmake_minimum_required(VERSION 2.8)
project(rs_mapping_test)
# Add definitions
if(WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "C:/local" CACHE PATH "Installation directory" FORCE)
endif()
if(MSVC)
  add_definitions(-D_WIN32_WINNT=0x0501)
  set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs")
else()
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build")
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

# find_package(Boost 1.65.0 REQUIRED COMPONENTS system date_time)

# Set dependent libraries
set(HEADER_DIR "" CACHE PATH "The path to dependent headers")
set(LIBRARY_DIR "" CACHE PATH "The path to dependent libraries")
set(DEPENDENT_LIBS rs_mapping)
include_directories(
  ${HEADER_DIR}
)
link_directories(
  ${LIBRARY_DIR}
)

# Build information
set(EXECUTABLES "map_manager_test")     #提出用
#set(EXECUTABLES "scan_matching_test;icp_matching_test;map_manager_test")
foreach(target IN LISTS EXECUTABLES)
  message("${target}")
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} ${DEPENDENT_LIBS})
endforeach()

# Set installation
foreach(target IN LISTS EXECUTABLES)
  install(
    TARGETS ${target}
    RUNTIME DESTINATION bin
  )
endforeach()