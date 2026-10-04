#!/usr/bin/env python3
"""CPU-only CI: standard launch + installed YOLO + ROS camera/service wiring.
Uses a random temporary model, never the selected deployment weight or a download.
This is not an accuracy or GPU benchmark.
"""
import hashlib
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from sensor_msgs.msg import Image
from cortex_msgs.msg import DetectionArray
from cortex_msgs.srv import CheckTarget
import torch
from ultralytics import YOLO
import yaml

torch.set_num_threads(2)
with tempfile.TemporaryDirectory(prefix='cortex-yolo-smoke-') as directory:
    root = Path(directory)
    weight = root/'random.pt'
    YOLO('yolo26n.yaml').save(str(weight))
    cfg = yaml.safe_load(Path('/workspace/cortex/install/cortex_bringup/share/'
                             'cortex_bringup/config/cortex_params.yaml').read_text())
    det = cfg['detector_node']['ros__parameters']
    det.update(model=str(weight), model_sha256=hashlib.sha256(weight.read_bytes()).hexdigest(),
               device='cpu', camera_transport='raw', camera_topic='/test/yolo/image',
               imgsz=64)
    params = root/'params.yaml'
    params.write_text(yaml.safe_dump(cfg))
    with (root/'launch.log').open('w+') as log:
        proc = subprocess.Popen(['ros2', 'launch', 'cortex_bringup', 'cortex.launch.py',
                                 f'params_file:={params}', 'detector_only:=true'],
                                stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('yolo_runtime_smoke')
        pub = node.create_publisher(Image, '/test/yolo/image', 10)
        frames = []
        node.create_subscription(DetectionArray, '/cortex/detections', frames.append, 10)
        client = node.create_client(CheckTarget, '/cortex/detector/check')
        try:
            assert client.wait_for_service(timeout_sec=40), 'detector service did not start'
            deadline = time.monotonic()+30
            pending = None
            success = False
            while time.monotonic() < deadline:
                assert proc.poll() is None, 'launch exited unexpectedly'
                msg = Image(height=64, width=64, encoding='rgb8', step=192,
                            data=np.zeros((64, 64, 3), np.uint8).tobytes())
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = 'smoke_camera'
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.15)
                if len(frames) >= 3 and pending is None:
                    pending = client.call_async(CheckTarget.Request(target='fridge'))
                if pending is not None and pending.done():
                    reply = pending.result()
                    assert reply.detail not in ('stub', 'model_not_loaded', 'inference_error')
                    if reply.detail == '':
                        success = True
                        break
                    pending = None
            assert success, 'fresh YOLO inference/presence window did not become available'
            assert frames[-1].header.frame_id == 'smoke_camera'
            assert frames[-1].header.stamp.sec > 0
            print('STANDARD_YOLO_LAUNCH_OK: real CPU inference and ROS service, random weights')
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read())
            raise
        finally:
            node.destroy_node()
            rclpy.shutdown()
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
