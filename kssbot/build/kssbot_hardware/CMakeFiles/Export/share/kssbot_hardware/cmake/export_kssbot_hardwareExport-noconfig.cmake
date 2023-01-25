#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kssbot_hardware::kssbot_hardware" for configuration ""
set_property(TARGET kssbot_hardware::kssbot_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(kssbot_hardware::kssbot_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkssbot_hardware.so"
  IMPORTED_SONAME_NOCONFIG "libkssbot_hardware.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS kssbot_hardware::kssbot_hardware )
list(APPEND _IMPORT_CHECK_FILES_FOR_kssbot_hardware::kssbot_hardware "${_IMPORT_PREFIX}/lib/libkssbot_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
