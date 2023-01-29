#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kssbot_diffdrive_rasp4::kssbot_diffdrive_rasp4" for configuration ""
set_property(TARGET kssbot_diffdrive_rasp4::kssbot_diffdrive_rasp4 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(kssbot_diffdrive_rasp4::kssbot_diffdrive_rasp4 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkssbot_diffdrive_rasp4.so"
  IMPORTED_SONAME_NOCONFIG "libkssbot_diffdrive_rasp4.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS kssbot_diffdrive_rasp4::kssbot_diffdrive_rasp4 )
list(APPEND _IMPORT_CHECK_FILES_FOR_kssbot_diffdrive_rasp4::kssbot_diffdrive_rasp4 "${_IMPORT_PREFIX}/lib/libkssbot_diffdrive_rasp4.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
