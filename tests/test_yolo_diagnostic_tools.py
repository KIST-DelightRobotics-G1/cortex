"""ROS-free diagnostic replay tests; no model downloads or hardware required."""
import importlib.util
from pathlib import Path

import pytest
from cortex_perception.detection import DetectionRecord

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('diagnose_yolo', ROOT/'scripts/diagnose_yolo.py')
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


PARAMS = dict(target_keys=['fridge_door', 'cucumber'], target_classes=['refrigerator', 'cucumber'],
              infer_conf=.25, default_min_confidence=.25, window_s=.6, min_frames=3,
              require_latest_hit=True)
HIT = DetectionRecord('refrigerator', .2, .5, .5, .5, .5)


def test_sweep_exposes_low_candidates_without_mutating_deployed_baseline():
    cases, windows = diag.make_windows(PARAMS, ['refrigerator', 'cucumber'], [.15, .25])
    for t in (0., .125, .25):
        rows = diag.evaluate(windows, cases, [], [HIT], t, t+.02)
    fridge = {r['case']: r for r in rows if r['target'] == 'fridge_door'}
    assert fridge['baseline']['reason'] == 'no_hits'
    assert fridge['confidence_0.15']['found']
    assert not fridge['confidence_0.25']['found']
    assert PARAMS['infer_conf'] == .25


def test_delayed_positive_frames_are_not_treated_as_recent_arrivals():
    cases, windows = diag.make_windows(PARAMS, ['refrigerator', 'cucumber'], [.25])
    hit = DetectionRecord('refrigerator', .9, .5, .5, .5, .5)
    for i in range(10):
        rows = diag.evaluate(windows, cases, [hit], [hit], i*.125, i*.125+.4)
    fridge = next(r for r in rows if r['target'] == 'fridge_door' and r['case'] == 'baseline')
    assert fridge['reason'] == 'insufficient_frames'
    assert fridge['samples'] == 2 and fridge['hits'] == 2


def test_timestamp_fallback_is_explicit_and_never_reuses_a_source_frame():
    assert diag.source_time(0, 0, 30, None) == (0, 'pts')
    assert diag.source_time(1, 0, 30, 0) == (1/30, 'fps_fallback')
    with pytest.raises(ValueError, match='nonmonotonic'):
        diag.source_time(1, 0, 30, 5)


def test_runtime_param_dump_supported_and_incomplete_profiles_rejected(tmp_path):
    import yaml
    profile = tmp_path/'runtime.yaml'
    profile.write_text(yaml.safe_dump({'/detector_node': {'ros__parameters': dict(PARAMS, imgsz=640, rate_hz=8)}}))
    assert diag.profile_values(profile)['rate_hz'] == 8
    profile.write_text('detector_node: {ros__parameters: {diagnostics_enabled: true}}')
    with pytest.raises(ValueError, match='missing detector'):
        diag.profile_values(profile)
