"""Diagnostics explain existing decisions without changing the service contract."""
import json

from cortex_perception.detection import DetectionRecord, PresenceWindow, fill_response
from cortex_perception.detector_diagnostics import DetectorDiagnostics
from types import SimpleNamespace


HIT = DetectionRecord('refrigerator', .8, .5, .5, .5, .5)


def fixture_window(pattern):
    window = PresenceWindow({'fridge_door': frozenset(['refrigerator'])}, ['refrigerator'],
                            min_confidence=.25, require_latest_hit=True)
    for index, present in enumerate(pattern):
        window.add([HIT] if present else [], index*.125, index+1, index*.125)
    return window


def test_diagnostics_disambiguate_latest_miss_without_leaking_into_response_detail():
    window = fixture_window([True, True, True, True, False])
    evidence = {}
    before = window.check('fridge_door', .5)
    with_diagnostics = window.check('fridge_door', .5, diagnostics=evidence)
    assert before == with_diagnostics
    assert not before.found and before.detail == ''
    assert evidence['reason'] == 'latest_miss'
    assert (evidence['samples'], evidence['hits'], evidence['latest_hit']) == (5, 4, False)
    assert evidence['last_hit_age_ms'] == 125
    response = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
    fill_response(response, with_diagnostics)
    assert response.detail == '' and response.confidence == 0
    assert evidence['max_candidate_confidence'] == .8


def test_insufficient_frames_can_contain_real_hits_and_stale_clears_snapshot():
    window = fixture_window([True, True])
    evidence = {}
    verdict = window.check('fridge_door', .125, diagnostics=evidence)
    assert verdict.detail == 'insufficient_frames' and verdict.hits == 0
    assert evidence['hits'] == 2  # Preserve old Verdict, but explain available raw evidence.
    window.check('fridge_door', 2, diagnostics=evidence)
    assert evidence['reason'] == 'stale' and 'max_candidate_confidence' not in evidence


def test_majority_failure_and_true_no_hits_have_separate_diagnostics():
    for pattern, reason in [([True, False, False], 'below_majority'), ([False]*3, 'no_hits')]:
        evidence = {}
        assert not fixture_window(pattern).check('fridge_door', .25, diagnostics=evidence).found
        assert evidence['reason'] == reason and evidence['detail'] == ''


def test_diagnostic_failures_never_raise_and_storage_is_bounded_by_event_names():
    published = []
    diag = DetectorDiagnostics(published.append, lambda: 123)
    for i in range(500):
        diag.record('accepted_frame', source_stamp_ns=i)
    diag.heartbeat(config={'loaded': True})
    record = json.loads(published[0])
    assert record['counters']['accepted_frame'] == 500
    assert len(record['last']) == 1 and record['last']['accepted_frame']['source_stamp_ns'] == 499
    assert record['ros_time_ns'] == 123
    def fail(_):
        raise OSError('subscriber/output unavailable')
    diag.publish = fail
    diag.emit('check', found=False)
    assert diag._counts['diagnostics_publish_error'] == 1
    disabled = DetectorDiagnostics()
    disabled.record('camera_message')
    disabled.heartbeat()
    assert disabled._counts == {}
