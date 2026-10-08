#!/usr/bin/env python3
"""Humble rosbag -> timestamp-preserving MP4, source frame map and readable events.
Run in the same sourced ROS environment as Cortex. No ROS graph or motion calls.
"""
import argparse
from fractions import Fraction
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--camera-topic', default='/kist/camera/head/color/h264')
    args = parser.parse_args()
    if not args.bag.is_dir() or args.output.exists():
        parser.error('bag must exist and output must be a new directory')
    import av
    import cv2
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from cortex_perception.ext_sensor import H264Decoder, decoded_stamp_ns
    import yaml
    metadata = yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=metadata['storage_identifier']),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.camera_topic not in types:
        parser.error('camera topic absent from bag; specify --camera-topic')
    supported = ('kist_msgs/msg/CompressedColorFrame', 'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
    if types[args.camera_topic] not in supported:
        parser.error('unsupported camera type: '+types[args.camera_topic])
    classes = {name: get_message(kind) for name, kind in types.items()}
    args.output.mkdir(parents=True)
    decoder, mux, stream = H264Decoder(), None, None
    first, last, index, skipped = None, None, 0, 0
    counts = {}
    try:
        with (args.output/'events.jsonl').open('w') as events, (args.output/'video_frames.jsonl').open('w') as frames:
            while reader.has_next():
                topic, data, received_ns = reader.read_next()
                msg = deserialize_message(data, classes[topic])
                counts[topic] = counts.get(topic, 0)+1
                if topic != args.camera_topic:
                    value = message_to_ordereddict(msg)
                    if types[topic] == 'std_msgs/msg/String':
                        try:
                            value = json.loads(msg.data)
                        except (ValueError, TypeError):
                            pass
                    events.write(json.dumps(dict(topic=topic, received_ns=received_ns, message=value), ensure_ascii=False)+'\n')
                    continue
                kind = types[topic]
                if kind == 'kist_msgs/msg/CompressedColorFrame':
                    image = decoder.decode(msg.data, msg.is_keyframe, msg.stamp_ns)
                    stamp = decoded_stamp_ns(image) if image is not None else None
                else:
                    stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                    if kind == 'sensor_msgs/msg/CompressedImage':
                        array = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
                    elif msg.encoding in ('rgb8', 'bgr8') and msg.step >= msg.width*3:
                        array = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
                        if msg.encoding == 'rgb8':
                            array = array[:, :, ::-1]
                    else:
                        raise ValueError('unsupported raw image layout/encoding')
                    image = av.VideoFrame.from_ndarray(array, format='bgr24') if array is not None else None
                if image is None or stamp is None or stamp <= 0 or (last is not None and stamp <= last):
                    skipped += 1
                    continue
                if mux is None:
                    mux = av.open(str(args.output/'camera.mp4'), 'w')
                    stream = mux.add_stream('libx264', rate=30)
                    stream.width, stream.height = image.width, image.height
                    stream.pix_fmt = 'yuv420p'
                    stream.time_base = Fraction(1, 1_000_000)
                    stream.codec_context.time_base = Fraction(1, 1_000_000)
                    stream.options = {'preset': 'fast', 'crf': '12', 'bf': '0'}
                    first = stamp
                if (image.width, image.height) != (stream.width, stream.height):
                    raise ValueError('camera resolution changed within bag; export separate recordings')
                image.pts, image.time_base = round((stamp-first)/1000), Fraction(1, 1_000_000)
                for packet in stream.encode(image):
                    mux.mux(packet)
                frames.write(json.dumps(dict(frame=index, source_stamp_ns=stamp,
                                             source_s=(stamp-first)/1e9, received_ns=received_ns))+'\n')
                index += 1
                last = stamp
            if stream is not None:
                for packet in stream.encode():
                    mux.mux(packet)
    except Exception as error:
        (args.output/'ERROR.txt').write_text(str(error)+'\nIncomplete export.\n')
        raise
    finally:
        if mux is not None:
            mux.close()
    result = dict(decoded_video_frames=index, skipped_camera_messages=skipped, topic_counts=counts,
                  source_duration_s=(last-first)/1e9 if first is not None else 0,
                  scope='Decoded source PTS preserved relative to first exported frame. MP4 is re-encoded (CRF12), not pixel-identical raw input. '
                        'Initial packets may wait for a keyframe; no frame for every packet is expected. Buffered tail frames are not flushed, matching the live decoder. '
                        'Use source frame map/events for exact stamp correlation; bag-receipt age is not detector callback age.')
    (args.output/'export_summary.json').write_text(json.dumps(result, indent=2))
    if index == 0:
        raise SystemExit('No video frames exported; check camera/keyframes/source timestamps. See events.jsonl.')
    print(f'EXPORT_COMPLETE {args.output.resolve()} ({index} frames)')


if __name__ == '__main__':
    main()
