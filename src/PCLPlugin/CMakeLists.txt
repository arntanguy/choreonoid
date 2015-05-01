set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
message(STATUS ${CMAKE_MODULE_PATH})
option(BUILD_PCL_PLUGIN "Building PCLPlugin" OFF)

if(NOT BUILD_PCL_PLUGIN)
  return()
endif()

find_package(PCL REQUIRED common io surface features)
find_package(ICP REQUIRED)

set(ICP_INCLUDE_DIRS /usr/local/include/icp/)
set(ICP_LIBRARIES /usr/local/lib/libicp.so)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${ICP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(sources
  PCLPlugin.cpp
  PointCloudUtil.cpp
  MultiPointSetItem.cpp
  )

set(headers
  PointCloudUtil.h
  MultiPointSetItem.h
  exportdecl.h
)

set(target CnoidPCLPlugin)

make_gettext_mofiles(${target} mofiles)
add_cnoid_plugin(${target} SHARED ${sources} ${headers} ${mofiles})
target_link_libraries(${target} CnoidBase ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SURFACE_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${ICP_LIBRARIES})
apply_common_setting_for_plugin(${target} "${headers}")
