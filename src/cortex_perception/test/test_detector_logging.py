"""Console evidence must describe, never change, the presence decision."""
from types import SimpleNamespace

import pytest

from cortex_perception.detection import DetectionRecord, PresenceWindow, fill_response
from cortex_perception.detector_logging import DetectorLogStats

CU = DetectionRecord('cucumber', .8, .5, .5, .2, .1)


def make_window():
    return PresenceWindow({'cucumber': {'cucumber'}, 'user': {'person'}}, ['cucumber'],
                          min_confidence=.25, require_latest_hit=True)


def response(verdict):
    return vars(fill_response(SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)), verdict))


@pytest.mark.parametrize('hits,reason,samples,expected_hits,latest', [
    ([], 'no_frame', 0, None, None),
    ([True, True], 'insufficient_frames', 2, None, None),
    ([False, False, False], 'no_hits', 3, 0, False),
    ([True, False, False], 'below_majority', 3, 1, False),
    ([True, True, False], 'latest_miss', 3, 2, False),
    ([True, True, True], 'present', 3, 3, True),
])
def test_check_evidence_preserves_wire_response(hits, reason, samples, expected_hits, latest):
    window = make_window()
    now = 0.
    for i, hit in enumerate(hits):
        now = i*.125
        assert window.add([CU] if hit else [], now, i+1, now)
    plain = window.check('cucumber', now)
    diagnostic = {}
    observed = window.check('cucumber', now, diagnostics=diagnostic)
    assert response(plain) == response(observed)
    assert diagnostic['reason'] == reason
    assert diagnostic['samples'] == samples
    assert diagnostic['hits'] == expected_hits
    assert diagnostic['latest_hit'] is latest
    assert diagnostic['max_candidate_confidence'] == (.8 if any(hits) else None)
    # Ordinary absence must retain empty detail (the executor depends on this).
    if reason in ('no_hits', 'below_majority', 'latest_miss'):
        assert observed.detail == ''


def test_candidates_below_check_threshold_are_visible_in_log_only():
    window = make_window()
    weak = DetectionRecord('cucumber', .3, .5, .5, .2, .1)
    for i in range(3):
        window.add([weak], i*.125, i+1, i*.125)
    diagnostic = {}
    result = window.check('cucumber', .25, min_confidence=.4, diagnostics=diagnostic)
    assert not result.found and result.detail == ''
    assert diagnostic['reason'] == 'no_hits'
    assert diagnostic['max_candidate_confidence'] == .3
    assert diagnostic['min_confidence'] == .4
    assert window.check('cucumber', .25).found


@pytest.mark.parametrize('target,kwargs,reason', [
    ('missing', {}, 'unknown_target'),
    ('user', {}, 'unsupported_class'),
    ('cucumber', {'min_confidence': float('nan')}, 'invalid_request'),
    ('cucumber', {'max_age_s': -1.}, 'invalid_request'),
])
def test_unavailable_reasons_remain_unchanged(target, kwargs, reason):
    window = make_window()
    diagnostic = {}
    plain = window.check(target, 0., **kwargs)
    logged = window.check(target, 0., **kwargs, diagnostics=diagnostic)
    assert logged == plain and logged.detail == reason
    assert diagnostic['reason'] == reason and diagnostic['hits'] is None


def test_stale_and_inference_fault_do_not_report_old_candidates():
    window = make_window()
    for i in range(3):
        window.add([CU], i*.125, i+1, i*.125)
    diagnostic = {}
    assert window.check('cucumber', 1., diagnostics=diagnostic).detail == 'stale'
    assert diagnostic['max_candidate_confidence'] is None
    window.invalidate('inference_error')
    assert window.check('cucumber', 1., diagnostics=diagnostic).detail == 'inference_error'
    assert diagnostic['reason'] == 'inference_error'


def test_pipeline_rates_cover_the_last_two_seconds_and_count_drops():
    now = [10.]
    stats = DetectorLogStats(True, clock=lambda: now[0])
    for t in (8.5, 9.0, 9.5, 10.0):
        now[0] = t
        stats.record('received')
        stats.record('frame_accepted', source_observed_t=t)
    stats.record('inference_completed', infer_ms=24.6, best_confidence={'cucumber': .8})
    stats.record('stale_input', rejected_source_age_ms=900.)
    stats.record('h264_no_output')
    now[0] = 10.6                                                  # 8.5 has left the 2 s window
    first = stats.pipeline()
    assert first['camera_fps'] == 1.5 and first['infer_hz'] == .5    # 3 frames / 1 inference in 2 s
    assert first['infer_ms'] == 24.6 and first['best_confidence'] == {'cucumber': .8}
    assert first['dropped'] == {'h264_no_output': 1, 'stale_input': 1}
    now[0] = 14.
    later = stats.pipeline()
    assert later['camera_fps'] == 0. and later['infer_hz'] == 0.      # input stopped
    assert later['dropped'] == first['dropped']                     # lifetime counts


def test_disabled_statistics_retain_no_pipeline_data():
    stats = DetectorLogStats(False)
    stats.record('frame_accepted')
    stats.record('inference_completed', infer_ms=12.)
    assert stats.pipeline() == {'camera_fps': 0., 'infer_hz': 0., 'infer_ms': None,
                                'best_confidence': {}, 'dropped': {}}
