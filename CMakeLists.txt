cmake_minimum_required(VERSION 3.13)
project(ros2_dvs_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(visp_bridge REQUIRED)
find_package(VISP REQUIRED io)
find_package(VISP REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(ur_rtde REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(CaptureAndSaveDesiredImage src/captureAndSaveDesired/captureAndSaveDesired.cpp src/captureAndSaveDesired/DesiredImageManager.cpp)
target_link_libraries(CaptureAndSaveDesiredImage
                      ${OpenCV_LIBRARIES} 
                      visp_io
                      C_UR
                    )

ament_target_dependencies(CaptureAndSaveDesiredImage rclcpp cv_bridge image_transport visp_bridge)

target_include_directories(CaptureAndSaveDesiredImage PUBLIC
  $<INSTALL_INTERFACE:include>
  ${visp_bridge_INCLUDE_DIRS}
)

add_library(C_UR Robot/C_UR.cpp)
target_link_libraries(C_UR ${Boost_LIBRARIES} ${VISP_LIBRARIES} ur_rtde::rtde)

target_include_directories(C_UR PUBLIC
  $<INSTALL_INTERFACE:include>
  ${visp_bridge_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_ddvsCameraThinLensParameters src/defocusBasedVisualServoing/include/CCameraThinLensParameters.cpp)
add_library(${PROJECT_NAME}_ddvsFeatureDefocusedLuminance src/defocusBasedVisualServoing/include/CFeatureDefocusedLuminance.cpp)
add_library(${PROJECT_NAME}_ddvsFeatureDefocusedLuminanceFC src/defocusBasedVisualServoingFocusControl/include/CFeatureDefocusedLuminanceFC.cpp)
add_library(${PROJECT_NAME}_ddvsImageFilter src/defocusBasedVisualServoing/include/CImageFilter.cpp)

target_include_directories(${PROJECT_NAME}_ddvsCameraThinLensParameters PUBLIC
  $<INSTALL_INTERFACE:include>
  ${visp_bridge_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_ddvsFeatureDefocusedLuminance PUBLIC
  $<INSTALL_INTERFACE:include>
  ${visp_bridge_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_ddvsImageFilter PUBLIC
  $<INSTALL_INTERFACE:include>
  ${visp_bridge_INCLUDE_DIRS}
)
target_include_directories(${PROJECT_NAME}_ddvsFeatureDefocusedLuminanceFC PUBLIC
  $<INSTALL_INTERFACE:include>
  ${visp_bridge_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}_ddvsImageFilter PUBLIC
  $<INSTALL_INTERFACE:include>
  ${visp_bridge_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_ddvsDirectVisualservoing src/defocusBasedVisualServoing/ddvsDefocusBasedVisualServoing.cpp src/defocusBasedVisualServoing/ddvsVisualServoing.cpp)
target_link_libraries(${PROJECT_NAME}_ddvsDirectVisualservoing  ${VISP_LIBRARIES}
                                                         ${PROJECT_NAME}_ddvsCameraThinLensParameters
                                                         ${PROJECT_NAME}_ddvsFeatureDefocusedLuminance
                                                         ${PROJECT_NAME}_ddvsImageFilter
                                                         C_UR)

add_executable(${PROJECT_NAME}_focusControlDirectVisualservoing src/defocusBasedVisualServoingFocusControl/ddvsDefocusBasedVisualServoingFocusControl.cpp src/defocusBasedVisualServoingFocusControl/focusControlVisualServoing.cpp)
target_link_libraries(${PROJECT_NAME}_focusControlDirectVisualservoing  ${VISP_LIBRARIES}
                                                         ${PROJECT_NAME}_ddvsCameraThinLensParameters
                                                         ${PROJECT_NAME}_ddvsFeatureDefocusedLuminance
                                                         ${PROJECT_NAME}_ddvsFeatureDefocusedLuminanceFC
                                                         ${PROJECT_NAME}_ddvsImageFilter
                                                         C_UR)
                                                         

install(TARGETS CaptureAndSaveDesiredImage
        RUNTIME DESTINATION lib/${PROJECT_NAME})
            
            # Install headers
install(TARGETS ${PROJECT_NAME}_ddvsDirectVisualservoing
        RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS ${PROJECT_NAME}_focusControlDirectVisualservoing
RUNTIME DESTINATION lib/${PROJECT_NAME})
                    
ament_target_dependencies(${PROJECT_NAME}_ddvsDirectVisualservoing rclcpp tf2_msgs cv_bridge image_transport visp_bridge )
ament_target_dependencies(${PROJECT_NAME}_focusControlDirectVisualservoing rclcpp tf2_msgs cv_bridge image_transport visp_bridge )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
