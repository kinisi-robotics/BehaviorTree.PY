cmake_minimum_required(VERSION 3.8)
project(behaviortree_py)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(fmt REQUIRED)
find_package(py_binding_tools REQUIRED)

ament_python_install_package(behaviortree_py PACKAGE_DIR behaviortree_py)

add_library(${PROJECT_NAME}_headers INTERFACE)

target_link_libraries(${PROJECT_NAME}_headers
                      INTERFACE behaviortree_cpp::behaviortree_cpp fmt::fmt)
target_include_directories(
  ${PROJECT_NAME}_headers
  INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
            "$<INSTALL_INTERFACE:include>")

pybind11_add_module(behaviortree_py src/behaviortree_py.cpp)
target_compile_features(behaviortree_py PRIVATE cxx_std_20)
target_link_libraries(
  behaviortree_py
  PRIVATE behaviortree_cpp::behaviortree_cpp fmt::fmt ${PROJECT_NAME}_headers
          py_binding_tools::py_binding_tools)

add_custom_command(
  TARGET behaviortree_py
  POST_BUILD
  COMMAND stubgen --output $<TARGET_FILE_DIR:behaviortree_py> -p behaviortree_py
          -v --verbose
  WORKING_DIRECTORY $<TARGET_FILE_DIR:behaviortree_py>
  USES_TERMINAL)

install(DIRECTORY include/ DESTINATION include/)
install(TARGETS behaviortree_py
        LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/behaviortree_py)

install(
  TARGETS ${PROJECT_NAME}_headers
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES
  DESTINATION include)
install(
  FILES
    $<TARGET_FILE_DIR:behaviortree_py>/$<TARGET_FILE_BASE_NAME:behaviortree_py>.pyi
  COMPONENT python
  DESTINATION ${PYTHON_INSTALL_DIR}/$<TARGET_FILE_BASE_NAME:behaviortree_py>)

ament_export_targets(${PROJECT_NAME}Targets)
ament_package()
