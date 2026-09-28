# ext-sensor-io -> anyone. One H.264-encoded color frame.
# Mirrors ext-sensor-io idl/kist_camera_frames.idl field for field; ext-sensor-io
# declares it as kist_msgs::msg::dds_::CompressedColorFrame_ so ROS nodes read it directly.
# Topic: /kist/camera/<name>/color/h264 (DDS rt/kist/camera/<name>/color/h264), BestEffort.

uint32 width
uint32 height
uint64 seq
int64 stamp_ns
bool is_keyframe
string frame_id
uint8[] data         # Annex-B H.264 NAL units
