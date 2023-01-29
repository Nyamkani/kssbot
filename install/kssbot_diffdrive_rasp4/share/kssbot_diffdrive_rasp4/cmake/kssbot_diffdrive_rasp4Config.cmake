# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_kssbot_diffdrive_rasp4_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED kssbot_diffdrive_rasp4_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(kssbot_diffdrive_rasp4_FOUND FALSE)
  elseif(NOT kssbot_diffdrive_rasp4_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(kssbot_diffdrive_rasp4_FOUND FALSE)
  endif()
  return()
endif()
set(_kssbot_diffdrive_rasp4_CONFIG_INCLUDED TRUE)

# output package information
if(NOT kssbot_diffdrive_rasp4_FIND_QUIETLY)
  message(STATUS "Found kssbot_diffdrive_rasp4: 0.0.1 (${kssbot_diffdrive_rasp4_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'kssbot_diffdrive_rasp4' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${kssbot_diffdrive_rasp4_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(kssbot_diffdrive_rasp4_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_targets-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${kssbot_diffdrive_rasp4_DIR}/${_extra}")
endforeach()
