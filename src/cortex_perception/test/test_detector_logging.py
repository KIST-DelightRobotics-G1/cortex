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


def test_statistics_distinguish_received_accepted_and_inferred_rates():
    now = [10.]
    stats = DetectorLogStats(True, clock=lambda: now[0])
    for _ in range(6):
        stats.record('received')
    stats.record('frame_accepted', source_observed_t=9.9, source_stamp_ns=123)
    stats.record('inference_completed', infer_ms=24.6, best_confidence={'cucumber': .8})
    stats.record('stale_input')
    now[0] = 12.
    first = stats.snapshot()
    assert first['rates_hz'] == {'received': 3., 'frame_accepted': .5, 'inference_completed': .5}
    assert first['latest_frame_age_ms'] == 2100.
    assert first['counters']['stale_input'] == 1
    assert first['infer_ms'] == 24.6 and first['best_confidence'] == {'cucumber': .8}
    now[0] = 14.
    second = stats.snapshot()
    assert all(v == 0. for v in second['rates_hz'].values())
    assert second['received_age_ms'] == 4000.  # distinguishes stopped input from class absence
    assert first['counters'] == second['counters']


def test_disabled_statistics_retain_no_pipeline_data():
    stats = DetectorLogStats(False)
    stats.record('received')
    stats.record('inference_completed', infer_ms=12.)
    result = stats.snapshot()
    assert result['counters'] == {}
    assert result['latest_frame_age_ms'] is None and 'infer_ms' not in result


def test_detailed_frame_evidence_matches_actual_threshold_and_wire_result():
    window = make_window()
    for i, confidence in enumerate([.8, .2, .9]):
        window.add([DetectionRecord('cucumber', confidence, .5, .5, .2, .1)],
                   i*.125, 100+i, i*.125)
    plain = window.check('cucumber', .25)
    diagnostic, frames = {}, []
    observed = window.check('cucumber', .25, diagnostics=diagnostic, frame_evidence=frames)
    assert observed == plain and observed.found
    assert [r['source_stamp_ns'] for r in frames] == [100, 101, 102]
    assert [r['hit'] for r in frames] == [True, False, True]
    assert [r['age_ms'] for r in frames] == [250., 125., 0.]
    assert sum(r['hit'] for r in frames) == diagnostic['hits']
    stale_frames = []
    assert window.check('cucumber', 1., frame_evidence=stale_frames).detail == 'stale'
    assert stale_frames == []
