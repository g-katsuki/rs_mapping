cmake_minimum_required(VERSION 2.8)
project(rs_mapping)
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

find_package(Boost 1.65.0 REQUIRED COMPONENTS timer filesystem system)

# Set dependent libraries
set(HEADER_DIR "" CACHE PATH "The path to dependent headers")
set(LIBRARY_DIR "" CACHE PATH "The path to dependent libraries")
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${HEADER_DIR}
)
link_directories(
  ${LIBRARY_DIR}
)
set(DEPENDENT_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_flann ${Boost_LIBRARIES})

# Build library
add_library(${PROJECT_NAME} SHARED
  #src/icp_matching.cpp        # 提出用にオフ
  #src/scan_matching.cpp
  src/map_manager.cpp
)
target_link_libraries(${PROJECT_NAME} ${DEPENDENT_LIBS})

# Set installation
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ryusei DESTINATION include)
