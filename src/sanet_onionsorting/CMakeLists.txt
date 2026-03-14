cmake_minimum_required(VERSION 3.8)
project(sanet_onionsorting)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Frcnn.srv"
  "srv/Yolo.srv"
  "srv/YoloDeepsort.srv"
  "msg/OBlobs.msg"
  DEPENDENCIES std_msgs
)

install(
  PROGRAMS
    scripts/client.py
    scripts/frcnn_client.py
    scripts/frcnn_service.py
    scripts/service.py
    scripts/test.py
    scripts/yolo_client.py
    scripts/yolo_service.py
    scripts/yolov7_deepsort_client.py
    scripts/yolov7_deepsort_service.py
    scripts/yolov8_client.py
    scripts/yolov8_service.py
    scripts/rgbd_imgpoint_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    thirdparty/faster_rcnn/detect.py
  DESTINATION lib/python3.12/site-packages/thirdparty/faster_rcnn
)

install(
  FILES
    thirdparty/yolov5/detect.py
  DESTINATION lib/python3.12/site-packages/thirdparty/yolov5
)

install(
  DIRECTORY
    thirdparty/yolov5/models
  DESTINATION lib/python3.12/site-packages/thirdparty/yolov5
)

install(
  FILES
    thirdparty/yolov5/utils/datasets.py
    thirdparty/yolov5/utils/torch_utils.py
    thirdparty/yolov5/utils/utils.py
  DESTINATION lib/python3.12/site-packages/thirdparty/yolov5/utils
)

install(
  FILES
    thirdparty/yolov7_deepsort/detection_helpers.py
    thirdparty/yolov7_deepsort/bridge_wrapper.py
  DESTINATION lib/python3.12/site-packages/thirdparty/yolov7_deepsort
)

install(
  FILES
    thirdparty/yolov8/detect.py
    thirdparty/yolov8/utils.py
  DESTINATION lib/python3.12/site-packages/thirdparty/yolov8
)

# ??
install(
  FILES
    thirdparty/faster_rcnn/keras_retinanet/utils/compute_overlap.c
  DESTINATION lib/python3.12/site-packages/thirdparty/faster_rcnn/keras_retinanet/utils
)

install(DIRECTORY
  networkconfig
  config
  launch
  thirdparty
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
