######################################################################################
# Python extension builder
######################################################################################
cmake_minimum_required(VERSION 3.12.4)
set(Python2_EXACTVERSION CACHE STRING "exact version of python2")
set(Python3_EXACTVERSION CACHE STRING "exact version of python3")
macro(Python_BUILD_EXTENSION Python_BUILD_VERSION)
if(NOT ${Python${Python_BUILD_VERSION}_EXACTVERSION} STREQUAL "")
  find_package(Python${Python_BUILD_VERSION} ${Python${Python_BUILD_VERSION}_EXACTVERSION} EXACT REQUIRED COMPONENTS Interpreter Development)
else()
  find_package(Python${Python_BUILD_VERSION} REQUIRED COMPONENTS Interpreter Development)
endif()
find_program(CYTHON_EXECUTABLE NAMES cython cython3)
if(NOT CYTHON_EXECUTABLE)
  message(FATAL_ERROR "Cython not found! Please install Cython.")
endif()
execute_process(
  COMMAND ${CYTHON_EXECUTABLE} --version
  OUTPUT_VARIABLE CYTHON_VERSION_OUTPUT
  ERROR_VARIABLE CYTHON_VERSION_OUTPUT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE)
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" CYTHON_VERSION "${CYTHON_VERSION_OUTPUT}")
if(NOT CYTHON_VERSION)
  message(FATAL_ERROR "Failed to extract Cython version from output: ${CYTHON_VERSION_OUTPUT}")
endif()
string(REGEX MATCHALL "[0-9]+" CYTHON_VERSION_COMPONENTS "${CYTHON_VERSION}")
list(GET CYTHON_VERSION_COMPONENTS 0 CYTHON_VERSION_MAJOR)
list(GET CYTHON_VERSION_COMPONENTS 1 CYTHON_VERSION_MINOR)
list(GET CYTHON_VERSION_COMPONENTS 2 CYTHON_VERSION_PATCH)
if (${CYTHON_VERSION_MAJOR} LESS_EQUAL 0)
  set(FREENECT_PYX "freenect.cython0.pyx")
else()
  set(FREENECT_PYX "freenect.pyx")
endif()
add_custom_command(OUTPUT freenect${Python_BUILD_VERSION}.c
  COMMAND ${CYTHON_EXECUTABLE} -${Python_BUILD_VERSION} -o freenect${Python_BUILD_VERSION}.c "${CMAKE_CURRENT_SOURCE_DIR}/${FREENECT_PYX}")
list(APPEND ADDITIONAL_MAKE_CLEAN_FILES freenect${Python_BUILD_VERSION}.c)
add_library(cython${Python_BUILD_VERSION}_freenect MODULE freenect${Python_BUILD_VERSION}.c)
set_target_properties(cython${Python_BUILD_VERSION}_freenect PROPERTIES
  PREFIX ""
  OUTPUT_NAME "freenect"
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python${Python_BUILD_VERSION})
target_link_libraries(cython${Python_BUILD_VERSION}_freenect
  freenect_sync
  ${Python${Python_BUILD_VERSION}_LIBRARIES})
target_include_directories(cython${Python_BUILD_VERSION}_freenect PRIVATE
  ${Python${Python_BUILD_VERSION}_INCLUDE_DIRS}
  ../c_sync/
  ${Python${Python_BUILD_VERSION}_NumPy_INCLUDE_DIRS})
install(TARGETS cython${Python_BUILD_VERSION}_freenect DESTINATION ${Python${Python_BUILD_VERSION}_SITELIB})
endmacro(Python_BUILD_EXTENSION)
if (BUILD_PYTHON2)
  Python_BUILD_EXTENSION(2)
endif(BUILD_PYTHON2)
if (BUILD_PYTHON3)
  Python_BUILD_EXTENSION(3)
endif(BUILD_PYTHON3)