# -*- coding: utf-8 -*-
"""speaker_node logic with a fake AudioClient (no Unitree SDK, no robot)."""
import threading
import time

import pytest
import rclpy
from std_msgs.msg import Bool

from cortex_action.speaker_node import SpeakerNode
from g1_onboard_msgs.msg import AudioPCM


class FakeClient:
    def __init__(self, block_first=False):
        self.played, self.stops = [], 0
        self.gate = threading.Event()
        if not block_first:
            self.gate.set()

    def PlayStream(self, app, stream_id, pcm):
        self.gate.wait(2.0)                       # lets a test hold the writer on chunk 1
        self.played.append((stream_id, pcm))
        return 0, None

    def PlayStop(self, app):
        self.stops += 1
        return 0


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def pcm(node, tag: int, stamp_ns=None, rate=16000):
    m = AudioPCM(sample_rate=rate, channels=1, bit_depth=16)
    ns = stamp_ns if stamp_ns is not None else node.get_clock().now().nanoseconds
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    m.data = [tag, 0] * 4
    return m


def wait(cond, t=2.0):
    end = time.time() + t
    while time.time() < end and not cond():
        time.sleep(0.01)
    return cond()


def make(fake):
    return SpeakerNode(client=fake)


def test_plays_in_order_as_one_stream():
    fake = FakeClient()
    n = make(fake)
    try:
        for tag in (1, 2, 3):
            n._on_pcm(pcm(n, tag))
        assert wait(lambda: len(fake.played) == 3)
        assert [p[1][0] for p in fake.played] == [1, 2, 3]
        assert len({p[0] for p in fake.played}) == 1       # one stream id for the sentence
    finally:
        n.destroy_node()


def test_other_formats_are_dropped():
    fake = FakeClient()
    n = make(fake)
    try:
        n._on_pcm(pcm(n, 1, rate=24000))
        time.sleep(0.3)
        assert fake.played == []
    finally:
        n.destroy_node()


def test_barge_in_drops_what_was_said_before_it():
    fake = FakeClient(block_first=True)
    n = make(fake)
    try:
        for tag in (1, 2, 3):
            n._on_pcm(pcm(n, tag))
        assert wait(lambda: len(n._queue) == 2)             # chunk 1 is on the wire, 2 and 3 wait
        n._on_stop(Bool(data=True))
        fake.gate.set()
        n._on_pcm(pcm(n, 9))                               # "멈춥니다." stamped after the stop
        assert wait(lambda: len(fake.played) == 2)
        time.sleep(0.2)
        assert [p[1][0] for p in fake.played] == [1, 9]
        assert fake.stops == 1
        assert fake.played[0][0] != fake.played[1][0]      # the new sentence is a new stream
    finally:
        n.destroy_node()


def test_late_chunk_from_before_the_stop_is_dropped():
    fake = FakeClient()
    n = make(fake)
    try:
        old = n.get_clock().now().nanoseconds
        n._on_stop(Bool(data=True))
        n._on_pcm(pcm(n, 5, stamp_ns=old))                 # was on the wire when stop came
        time.sleep(0.3)
        assert fake.played == []
    finally:
        n.destroy_node()


def test_full_queue_drops_the_oldest():
    fake = FakeClient(block_first=True)
    n = make(fake)
    try:
        n._max_q = 2
        n._queue = type(n._queue)(maxlen=2)
        n._on_pcm(pcm(n, 1))
        assert wait(lambda: len(n._queue) == 0)            # 1 is on the wire
        for tag in (2, 3, 4):
            n._on_pcm(pcm(n, tag))
        fake.gate.set()
        assert wait(lambda: len(fake.played) == 3)
        assert [p[1][0] for p in fake.played] == [1, 3, 4]
    finally:
        n.destroy_node()
