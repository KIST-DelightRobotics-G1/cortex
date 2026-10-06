"""Google v1 request configuration; no cloud calls."""
import queue

import pytest

rclpy = pytest.importorskip('rclpy')
from cortex_perception.stt_node import SttNode, STTState


class LocalStt(SttNode):
    def _start_backend(self):
        self._audio_queue = queue.Queue(maxsize=200)
        self._state = STTState.STREAMING


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
