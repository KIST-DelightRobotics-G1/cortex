# ext-sensor-io -> anyone. One mic chunk (default 100 ms) of raw interleaved PCM.
# Mirrors ext-sensor-io idl/kist_audio_frames.idl field for field; ext-sensor-io
# declares it as kist_msgs::msg::dds_::AudioChunk_ so ROS nodes read it directly.
# Topic: /kist/mic/<name>/audio (DDS rt/kist/mic/<name>/audio), BestEffort.

uint64 seq
int64 stamp_ns       # capture clock at chunk end (epoch ns)
uint32 sample_rate   # Hz
uint32 channels      # interleaved channel count
string format        # PCM sample format, e.g. "S16_LE"
string frame_id
uint8[] data         # interleaved PCM payload
