# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_evaluator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED evaluator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(evaluator_FOUND FALSE)
  elseif(NOT evaluator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(evaluator_FOUND FALSE)
  endif()
  return()
endif()
set(_evaluator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT evaluator_FIND_QUIETLY)
  message(STATUS "Found evaluator: 0.0.0 (${evaluator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'evaluator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${evaluator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(evaluator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${evaluator_DIR}/${_extra}")
endforeach()
