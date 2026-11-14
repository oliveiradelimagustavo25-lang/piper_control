cmake_minimum_required(VERSION 3.16)
project(yolo_tools VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)
find_package(yaml-cpp REQUIRED)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Extracted ONNX Runtime C/C++ SDK")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h HINTS "${ONNXRUNTIME_ROOT}/include")
find_library(ONNXRUNTIME_LIBRARY onnxruntime HINTS "${ONNXRUNTIME_ROOT}/lib")
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime SDK not found; set ONNXRUNTIME_ROOT to its extracted directory")
endif()

add_library(inference SHARED src/segmenter.cpp)
add_library(yolo_tools::inference ALIAS inference)
target_compile_features(inference PUBLIC cxx_std_17)
target_compile_options(inference PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(inference PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(inference SYSTEM PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}")
target_link_libraries(inference PUBLIC opencv_core PRIVATE opencv_imgproc opencv_dnn yaml-cpp "${ONNXRUNTIME_LIBRARY}")
get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
set_target_properties(inference PROPERTIES OUTPUT_NAME yolo_inference
  INSTALL_RPATH "${ONNXRUNTIME_LIBRARY_DIR}")

add_executable(yolo_predict src/predict.cpp)
target_link_libraries(yolo_predict PRIVATE inference opencv_imgcodecs opencv_imgproc)
set_target_properties(yolo_predict PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
install(TARGETS inference yolo_predict EXPORT yolo_toolsTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT yolo_toolsTargets NAMESPACE yolo_tools:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/yolo_tools)
configure_package_config_file(cmake/yolo_toolsConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/yolo_toolsConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/yolo_tools)
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/yolo_toolsConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yolo_toolsConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/yolo_toolsConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/yolo_tools)
