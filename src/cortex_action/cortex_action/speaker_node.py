"""speaker_node — TTS audio -> the robot's own speaker [SYS-REQ-29].

    /cortex/tts/audio (AudioPCM)  -> queue -> AudioClient.PlayStream -> G1 audio service ("voice")
    /cortex/tts/stop  (Bool)      -> drop what was said before it, PlayStop
    /cortex/speaker/state (SpeakerState) -> stt_node (mutes the mic while playing)

Playback goes through the Unitree SDK AudioClient — a DDS RPC to the robot's
audio service on domain 0, not ALSA — so the node runs wherever the robot is
visible on DDS: the cortex container on the PC. Ported from the onboard
speaker_node (onboard is no longer used).

rclpy and unitree_sdk2py share ONE libddsc: the Docker image builds the SDK's
cyclonedds Python binding against ROS's own CycloneDDS. The SDK's
ChannelFactory.Init would create the domain a second time and fail, so it is
patched to only add a participant to the domain rclpy already runs.

Barge-in: "stop" can arrive while chunks of the old sentence are still queued
or on the wire. Every AudioPCM carries its publish stamp; anything stamped
before the stop is dropped, so the "멈춥니다." that follows it still plays.

Pipeline LOCKED: 16 kHz / 16-bit / mono (AudioPCM.msg). Other formats are dropped.
"""

import threading
import time
from collections import deque
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Header

from g1_onboard_msgs.msg import AudioPCM, SpeakerState

SAMPLE_RATE, CHANNELS, BIT_DEPTH = 16000, 1, 16
IDLE_CHUNK_ID = 0                 # SpeakerState.current_chunk_id when nothing plays
MAX_CHUNK_ID = 0xFFFFFFFF         # uint32


def make_audio_client(domain_id: int, timeout_s: float):
    """The Unitree AudioClient, joined to rclpy's DDS domain. Imported lazily so
    the node's logic is testable without the SDK (tests pass a fake client)."""
    import unitree_sdk2py.core.channel as sdk_ch
    from cyclonedds.domain import DomainParticipant
    from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

    def _join_existing_domain(self, id: int, networkInterface=None, qos=None) -> bool:
        cls = self.__class__
        if cls._ChannelFactory__initialized:
            return True
        with cls._ChannelFactory__init_lock:
            if not cls._ChannelFactory__initialized:
                cls._ChannelFactory__participant = DomainParticipant(id)
                cls._ChannelFactory__qos = qos
                cls._ChannelFactory__initialized = True
        return True

    sdk_ch.ChannelFactory.Init = _join_existing_domain
    sdk_ch.ChannelFactory().Init(domain_id)
    client = AudioClient()
    client.SetTimeout(timeout_s)
    client.Init()
    return client


class SpeakerNode(Node):
    def __init__(self, client=None) -> None:
        super().__init__('speaker_node')
        self.declare_parameter('audio_topic', '/cortex/tts/audio')        # = tts_node.audio_out_topic
        self.declare_parameter('state_topic', '/cortex/speaker/state')    # = stt_node.speaker_state_topic
        self.declare_parameter('stop_topic', '/cortex/tts/stop')          # barge-in, = tts_node.barge_in_topic
        self.declare_parameter('max_queue_depth', 50)                     # SpeakerState.queue_depth is uint8
        self.declare_parameter('app_name', 'cortex_tts')                  # AudioClient stream owner tag
        # PlayStream / PlayStop wait for the audio service's reply. If it is unreachable
        # each chunk stalls this long while SpeakerState says "playing" (the mic stays
        # muted), so keep it short — the robot answers well inside it.
        self.declare_parameter('client_timeout_s', 2.0)

        g = self.get_parameter
        self._max_q = int(g('max_queue_depth').value)
        if not 1 <= self._max_q < 256:
            raise ValueError(f'max_queue_depth must be in [1, 255]; got {self._max_q}')
        self._app = str(g('app_name').value)

        # --- state (writer thread and ROS callbacks share it under _lock) ----
        self._lock = threading.Lock()
        self._queue: deque = deque(maxlen=self._max_q)   # (chunk_id, stamp_ns, pcm)
        self._next_chunk_id = 1
        self._current_chunk_id = IDLE_CHUNK_ID
        self._stream_id: Optional[str] = None            # new id per playback session
        self._stop_ns = 0                                # stamp of the last barge-in
        self._stop_pending = False                       # PlayStop runs on the writer thread

        self._client = client or make_audio_client(self.context.get_domain_id(),
                                                   float(g('client_timeout_s').value))

        self._state_pub = self.create_publisher(SpeakerState, g('state_topic').value, 10)
        self.create_subscription(AudioPCM, g('audio_topic').value, self._on_pcm, 50)
        self.create_subscription(Bool, g('stop_topic').value, self._on_stop, 10)

        self._running = True
        self._wake = threading.Event()
        self._thread = threading.Thread(target=self._writer_loop, name='speaker_writer', daemon=True)
        self._thread.start()
        self._publish_state()
        self.get_logger().info(
            f"speaker_node up: {g('audio_topic').value} -> AudioClient.PlayStream "
            f"(domain {self.context.get_domain_id()}, app {self._app}), state -> {g('state_topic').value}")

    # --- inputs ------------------------------------------------------------
    def _on_pcm(self, msg: AudioPCM) -> None:
        if (msg.sample_rate, msg.channels, msg.bit_depth) != (SAMPLE_RATE, CHANNELS, BIT_DEPTH):
            self.get_logger().warning(
                f'drop AudioPCM {msg.sample_rate}Hz/{msg.channels}ch/{msg.bit_depth}bit '
                f'(locked {SAMPLE_RATE}/{CHANNELS}/{BIT_DEPTH})')
            return
        if not msg.data or len(msg.data) % 2:
            return
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        with self._lock:
            if stamp_ns and stamp_ns <= self._stop_ns:
                return                                   # said before "stop": never play it
            dropped = len(self._queue) == self._max_q    # deque(maxlen) drops the OLDEST
            self._queue.append((self._alloc_chunk_id(), stamp_ns, bytes(msg.data)))
        if dropped:
            self.get_logger().warning(f'queue full ({self._max_q}); dropped the oldest chunk')
        self._wake.set()
        self._publish_state()

    def _on_stop(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            self._stop_ns = self.get_clock().now().nanoseconds
            n = len(self._queue)
            self._queue = deque((c for c in self._queue if c[1] > self._stop_ns), maxlen=self._max_q)
            n -= len(self._queue)
            self._stop_pending = True
        self._wake.set()
        self.get_logger().info(f'barge-in: dropped {n} queued chunk(s), stopping playback')

    def _alloc_chunk_id(self) -> int:
        cid = self._next_chunk_id
        self._next_chunk_id = cid + 1 if cid < MAX_CHUNK_ID else 1   # 0 means idle
        return cid

    # --- writer thread -----------------------------------------------------
    def _writer_loop(self) -> None:
        while self._running:
            with self._lock:
                stop, self._stop_pending = self._stop_pending, False
                chunk = self._queue.popleft() if self._queue else None
            if stop:
                self._play_stop()
                self._stream_id = None                   # the next sentence is a new stream
            if chunk is None:
                if self._current_chunk_id != IDLE_CHUNK_ID:
                    self._current_chunk_id = IDLE_CHUNK_ID
                    self._stream_id = None
                    self._publish_state()                # idle transition
                self._wake.wait(timeout=0.1)
                self._wake.clear()
                continue
            chunk_id, _, pcm = chunk
            if self._stream_id is None:
                self._stream_id = str(time.time_ns() // 1_000_000)
            self._current_chunk_id = chunk_id
            self._publish_state()                        # now playing this chunk
            try:
                code, _ = self._client.PlayStream(self._app, self._stream_id, pcm)
                if code != 0:
                    self.get_logger().error(f'PlayStream failed: code={code} chunk_id={chunk_id}')
            except Exception as e:  # noqa: BLE001 - one bad chunk must not kill the writer
                self.get_logger().error(f'PlayStream raised: {e!r}')

    def _play_stop(self) -> None:
        try:
            self._client.PlayStop(self._app)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'PlayStop raised: {e!r}')

    def _publish_state(self) -> None:
        with self._lock:
            depth = len(self._queue)
        msg = SpeakerState()
        msg.header = Header(frame_id='speaker')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.playing = self._current_chunk_id != IDLE_CHUNK_ID
        msg.current_chunk_id = self._current_chunk_id
        msg.queue_depth = depth
        self._state_pub.publish(msg)

    # --- shutdown: finish what is queued, then PlayStop ----------------------
    def destroy_node(self) -> bool:
        self._running = False
        self._wake.set()
        self._thread.join(timeout=5.0)
        with self._lock:
            rest, self._queue = list(self._queue), deque(maxlen=self._max_q)
        for _, _, pcm in rest:
            try:
                self._client.PlayStream(self._app, self._stream_id or str(time.time_ns() // 1_000_000), pcm)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'drain PlayStream raised: {e!r}')
        self._play_stop()
        self._current_chunk_id = IDLE_CHUNK_ID
        try:
            self._publish_state()
        except Exception:  # noqa: BLE001 - context may already be down
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():                  # launch's SIGINT may have shut the context down already
            rclpy.shutdown()


if __name__ == '__main__':
    main()
