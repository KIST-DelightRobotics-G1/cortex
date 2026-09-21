# detector_node -> anyone (GUI, rviz, logs). Published per inferred frame.
# The orchestrator does NOT read this; it asks CheckTarget instead.
std_msgs/Header header
string model                   # e.g. "yolo26s"
float32 infer_ms
Detection[] detections
