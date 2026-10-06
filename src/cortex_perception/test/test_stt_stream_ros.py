"""Google v1 request configuration and stream lifecycle; no cloud calls."""
import queue
import threading

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from g1_onboard_msgs.msg import SpeakerState
from kist_msgs.msg import AudioChunk
from cortex_perception.stt_node import SttNode, STTState, StreamingSpeechFilter


class LocalStt(SttNode):
    def _start_backend(self):
        self._audio_queue = queue.Queue(maxsize=200)
        self._state = STTState.STREAMING


def test_preserves_pcm_filter_mute_and_google_payload():
    rclpy.init()
    n = LocalStt()
    try:
        pcm = (np.sin(np.arange(1600) * 0.1) * 5000).astype('<i2').tobytes()
        msg = AudioChunk(seq=1, stamp_ns=n.get_clock().now().nanoseconds,
                         sample_rate=16000, channels=1, format='S16_LE', data=list(pcm))
        n._on_audio_msg(msg)
        packet = n._audio_queue.get_nowait()
        reference = StreamingSpeechFilter(16000, 120.0, 5500.0)
        assert packet == reference.process(np.frombuffer(pcm, dtype='<i2')).tobytes()
        n._audio_queue.put_nowait(packet)
        n._audio_queue.put_nowait(None)
        requests = list(n._google_request_gen())
        assert len(requests) == 1 and requests[0].audio_content == packet
        n._on_speaker_state(SpeakerState(playing=True))
        n._on_audio_msg(msg)
        assert n._audio_queue.get_nowait() == bytes(len(pcm))
        n._on_speaker_state(SpeakerState(playing=False))
        n._on_audio_msg(msg)  # existing echo tail must still silence this chunk
        assert n._audio_queue.get_nowait() == bytes(len(pcm))
    finally:
        n.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def local_stt():
    rclpy.init()
    node = LocalStt()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_close_wakes_empty_request_consumer_without_taking_next_audio(local_stt):
    n = local_stt
    requests = n._google_request_gen()
    waiting = threading.Event()
    finished = threading.Event()
    consumed = []

    def consume():
        waiting.set()
        consumed.extend(requests)
        finished.set()

    consumer = threading.Thread(target=consume, daemon=True)
    consumer.start()
    assert waiting.wait(1)
    requests.close()
    packet = bytes(3200)
    n._audio_queue.put_nowait(packet)
    assert finished.wait(1), 'half-close must wake a consumer with no audio'
    consumer.join(timeout=1)
    assert consumed == []
    assert n._audio_queue.get_nowait() is packet


@pytest.mark.parametrize('end_event', ['SPEECH_ACTIVITY_TIMEOUT', 'END_OF_SINGLE_UTTERANCE'])
def test_v1_half_closes_real_grpc_drains_final_and_reopens(monkeypatch, local_stt, end_event):
    """Real Google SDK + local gRPC: the server waits for EOF before its final.

    No Google endpoint or credentials are used. The second utterance is queued
    while the first stream drains, then must reach only the second RPC.
    """
    from concurrent.futures import ThreadPoolExecutor
    import grpc
    from google.auth.credentials import AnonymousCredentials
    from google.cloud import speech
    from google.cloud.speech_v1.services.speech.transports.grpc import SpeechGrpcTransport
    import cortex_perception.stt_node as stt

    n = local_stt
    n._config.speech_end_timeout_s = 1.0
    n._config.interim_results = False
    transcripts = []
    incoming = []
    emitted = n._emit_transcript

    def emit(event):
        emitted(event)
        transcripts.append(event.text)
        if event.text == '두 번째 문장':
            n._stop_event.set()

    monkeypatch.setattr(n, '_emit_transcript', emit)
    Request = speech.StreamingRecognizeRequest
    Response = speech.StreamingRecognizeResponse
    first, second = bytes([1, 0]) * 1600, bytes([2, 0]) * 1600
    n._audio_queue.put_nowait(first)

    def recognize(requests, context):
        config = next(requests)
        assert config.streaming_config.voice_activity_timeout.speech_end_timeout.total_seconds() == 1
        audio = next(requests).audio_content
        incoming.append(audio)
        if len(incoming) == 1:
            yield Response(speech_event_type=getattr(Response.SpeechEventType, end_event))
            assert list(requests) == [], 'client must half-close instead of waiting for more audio'
            assert n.state == STTState.DRAINING
            n._audio_queue.put_nowait(second)
            yield Response(results=[dict(is_final=False, alternatives=[dict(transcript='중간 결과')])])
            yield Response(results=[dict(is_final=True, alternatives=[dict(transcript='첫 번째 문장')])])
        else:
            assert transcripts == ['첫 번째 문장'], 'final must drain before the next RPC starts'
            yield Response(results=[dict(is_final=True, alternatives=[dict(transcript='두 번째 문장')])])
            list(requests)  # node shutdown closes this send side too

    server = grpc.server(ThreadPoolExecutor(max_workers=2))
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler('google.cloud.speech.v1.Speech', {
        'StreamingRecognize': grpc.stream_stream_rpc_method_handler(
            recognize, request_deserializer=Request.deserialize, response_serializer=Response.serialize),
    }),))
    port = server.add_insecure_port('127.0.0.1:0')
    server.start()
    transport = SpeechGrpcTransport(channel=grpc.insecure_channel(f'127.0.0.1:{port}'),
                                    credentials=AnonymousCredentials())
    client = speech.SpeechClient(transport=transport)
    monkeypatch.setattr(stt, '_load_google_credentials', lambda: None)
    monkeypatch.setattr(speech, 'SpeechClient', lambda: client)
    worker = threading.Thread(target=n._google_worker, daemon=True)
    try:
        worker.start()
        worker.join(timeout=5)
        assert not worker.is_alive(), 'SDK stream did not finish after half-close'
        assert incoming == [first, second]
        assert transcripts == ['첫 번째 문장', '두 번째 문장']
        assert n._audio_queue.empty()
    finally:
        n._stop_event.set()
        transport.close()
        server.stop(0).wait(timeout=2)
        worker.join(timeout=2)


def test_expected_timeout_cancellation_restarts_without_backoff_or_audio_loss(monkeypatch, local_stt):
    from google.api_core.exceptions import Cancelled
    from google.cloud import speech
    import cortex_perception.stt_node as stt

    n = local_stt
    calls = []
    pcm = bytes(3200)
    n._audio_queue.put_nowait(pcm)

    class Client:
        def streaming_recognize(self, *, config, requests):
            calls.append(1)
            assert next(requests).audio_content == pcm
            if len(calls) == stt._MAX_RECONNECT + 2:
                n._stop_event.set()
                return
            yield speech.StreamingRecognizeResponse(
                speech_event_type=speech.StreamingRecognizeResponse.SpeechEventType.SPEECH_ACTIVITY_TIMEOUT)
            assert n.state == STTState.DRAINING
            assert list(requests) == []
            n._audio_queue.put_nowait(pcm)
            raise Cancelled('server cancelled after endpoint')

    monkeypatch.setattr(stt, '_load_google_credentials', lambda: None)
    monkeypatch.setattr(speech, 'SpeechClient', Client)
    monkeypatch.setattr(n._stop_event, 'wait', lambda **kw: pytest.fail('unexpected retry backoff'))
    n._google_worker()
    assert len(calls) == stt._MAX_RECONNECT + 2
    assert n._audio_queue.empty()


@pytest.mark.parametrize('end_first,error_name', [(False, 'Cancelled'), (True, 'ServiceUnavailable')])
def test_unexpected_stream_errors_keep_retry_policy(monkeypatch, local_stt, end_first, error_name):
    from google.api_core import exceptions
    from google.cloud import speech
    import cortex_perception.stt_node as stt

    n = local_stt
    waits = []
    requests_seen = []

    class Client:
        def streaming_recognize(self, *, config, requests):
            requests_seen.append(requests)
            if end_first:
                yield speech.StreamingRecognizeResponse(
                    speech_event_type=speech.StreamingRecognizeResponse.SpeechEventType.SPEECH_ACTIVITY_TIMEOUT)
            n._audio_queue.put_nowait(bytes(3200))
            raise getattr(exceptions, error_name)('unexpected stream failure')

    def backoff(timeout):
        waits.append(timeout)
        if len(waits) == 2:
            n._stop_event.set()

    monkeypatch.setattr(stt, '_load_google_credentials', lambda: None)
    monkeypatch.setattr(speech, 'SpeechClient', Client)
    monkeypatch.setattr(n._stop_event, 'wait', backoff)
    n._google_worker()
    assert waits == [1.0, 2.0]
    assert n._audio_queue.empty()
    assert all(list(requests) == [] for requests in requests_seen)


@pytest.mark.parametrize('timeout_s', [0.0, 0.75, 1.0])
def test_v1_worker_passes_timeout_and_events_to_google(monkeypatch, timeout_s):
    """Inspect the actual SDK request config; zero retains the original stream."""
    from google.cloud import speech
    import cortex_perception.stt_node as stt

    captured = {}
    rclpy.init()
    n = LocalStt()
    try:
        n._config.speech_end_timeout_s = timeout_s
        n._config.model = 'latest_short'
        pcm = bytes(3200)
        n._audio_queue.put_nowait(pcm)
        n._audio_queue.put_nowait(None)

        class Client:
            def streaming_recognize(self, *, config, requests):
                assert n.state == STTState.CONNECTING
                captured['config'] = config
                captured['requests'] = list(requests)
                n._stop_event.set()
                return iter(())

        monkeypatch.setattr(stt, '_load_google_credentials', lambda: None)
        monkeypatch.setattr(speech, 'SpeechClient', Client)
        n._google_worker()
        config = captured['config']
        assert config.config.model == 'latest_short'
        assert config.interim_results is False
        assert config.single_utterance is False
        assert captured['requests'][0].audio_content == pcm
        assert len(captured['requests']) == 1
        assert config.enable_voice_activity_events == (timeout_s > 0)
        assert config._pb.HasField('voice_activity_timeout') == (timeout_s > 0)
        if timeout_s > 0:
            assert config.voice_activity_timeout.speech_end_timeout.total_seconds() == timeout_s
            assert not config.voice_activity_timeout._pb.HasField('speech_start_timeout')
    finally:
        n.destroy_node()
        rclpy.shutdown()
