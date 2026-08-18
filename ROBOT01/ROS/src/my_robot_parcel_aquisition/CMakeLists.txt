cmake_minimum_required(VERSION 3.8)
project(my_robot_parcel_aquisition)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SearchParcel.action"
  "action/ServoParcel.action"
  "action/RotateBlind.action"
)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  FILES
    my_robot_parcel_aquisition/acquisition_main.py
    my_robot_parcel_aquisition/acquisition_sequencer.py
    my_robot_parcel_aquisition/geometry.py
    my_robot_parcel_aquisition/manager.py
    my_robot_parcel_aquisition/rotate_action.py
    my_robot_parcel_aquisition/search_action.py
    my_robot_parcel_aquisition/servo_action.py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)
install(PROGRAMS scripts/acquisition_main scripts/acquisition_sequencer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md implement.md DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
