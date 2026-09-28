cmake_minimum_required(VERSION 3.8)
project(kist_msgs)

# ROS view of the ext-sensor-io wire types. ext-sensor-io (no ROS) declares the
# same structs in idl/ under kist_msgs::msg::dds_ with a trailing underscore,
# which is exactly the DDS type name ROS 2 gives these messages — so the two
# sides match on the wire. Keep the fields identical to that IDL.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioChunk.msg"
  "msg/CompressedColorFrame.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
