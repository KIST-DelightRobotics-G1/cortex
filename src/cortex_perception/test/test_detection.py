import math
from types import SimpleNamespace

import pytest

from cortex_perception.detection import (DetectionRecord, PresenceWindow, decode_boxes,
                                         fill_response, frame_time, target_mapping)

TARGETS = target_mapping(['fridge_door', 'cucumber', 'user'],
                         ['refrigerator', 'cucumber', 'person'])
CU = DetectionRecord('cucumber', .8, .5, .4, .2, .1)


def window():
    return PresenceWindow(TARGETS, ['refrigerator', 'cucumber'])


def add(w, i, ds, now=None):
    t = i*.125
    return w.add(ds, t, int((100+t)*1e9), t if now is None else now)


def test_requires_distinct_fresh_frames_and_majority_not_box_count():
    w = window()
    add(w, 0, [CU]*10)
    assert w.check('cucumber', 0).detail == 'insufficient_frames'
    assert not add(w, 0, [CU])
    add(w, 1, [])
    add(w, 2, [])
    assert not w.check('cucumber', .25).found
    add(w, 3, [CU])
    assert not w.check('cucumber', .375).found  # 2/4 is a tie.
    add(w, 4, [CU])
    assert w.check('cucumber', .5).found       # 3/5.


def test_unseen_and_unsupported_and_stale_are_not_true_absence():
    w = window()
    assert w.check('cucumber', 0).detail == 'no_frame'
    assert w.check('user', 0).detail == 'unsupported_class'
    assert w.check('banana', 0).detail == 'unknown_target'
    for i in range(3):
        add(w, i, [])
    v = w.check('cucumber', .25)
    assert not v.found and v.detail == ''
    assert w.check('cucumber', 1).detail == 'stale'


def test_default_service_threshold_differs_from_inference_threshold():
    w = window()
    weak = DetectionRecord('cucumber', .3, .5, .5, .1, .1)
    for i in range(3):
        add(w, i, [weak])
    assert not w.check('cucumber', .25).found
    assert w.check('cucumber', .25, min_confidence=.25).found
    assert w.check('cucumber', .25, max_age_s=.1).detail == 'insufficient_frames'


def test_slow_inference_and_out_of_order_frames_do_not_refresh_evidence():
    w = window()
    assert not add(w, 0, [CU], now=.7)
    assert add(w, 2, [CU])
    assert not add(w, 1, [CU], now=.3)
    assert w.check('cucumber', .9).detail == 'stale'


def test_fault_discards_old_positives_and_recovery_warms_up_again():
    w = window()
    for i in range(3):
        add(w, i, [CU])
    w.invalidate('inference_error')
    assert w.check('cucumber', .25).detail == 'inference_error'
    add(w, 3, [CU])
    assert w.check('cucumber', .375).detail == 'insufficient_frames'


def test_response_returns_source_stamp_and_normalized_coordinates():
    w = window()
    for i in range(3):
        add(w, i, [CU])
    response = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0))
    fill_response(response, w.check('cucumber', .25))
    assert response.found and response.stamp.sec == 100
    assert response.stamp.nanosec == 0  # highest-confidence tie chooses first source frame.
    assert (response.cx, response.cy, response.w, response.h) == (.5, .4, .2, .1)
    fill_response(response, w.check('cucumber', 2))
    assert response.label == '' and response.confidence == 0 and response.stamp.sec == 0


def test_decode_uses_weight_names_not_coco_class_ids():
    d = decode_boxes([[64, 48, 320, 240, .8, 1]], {0: 'refrigerator', 1: 'cucumber'}, 640, 480)[0]
    assert d.label == 'cucumber'
    assert (d.cx, d.cy, d.w, d.h) == (.3, .3, .4, .4)
    assert decode_boxes([[1, 1, 1, 8, .8, 1]], {1: 'cucumber'}, 640, 480) == ()
    with pytest.raises(ValueError):
        decode_boxes([[0, 0, math.nan, 1, .8, 1]], {1: 'cucumber'}, 640, 480)


def test_source_clock_age_includes_transport_and_disallows_unknown_stamp():
    assert frame_time(1_000_000_000, 1_200_000_000, 5., .6) == (4.8, 1_000_000_000)
    assert frame_time(1_000_000_000, 1_700_000_000, 5., .6) is None
    assert frame_time(2_000_000_000, 1_000_000_000, 5., .6) is None
    assert frame_time(0, 1_000_000_000, 5., .6) is None
    assert frame_time(0, 1_000_000_000, 5., .6, True) == (5., 1_000_000_000)


@pytest.mark.parametrize('confidence,age', [(math.nan, 0), (-.1, 0), (1.1, 0), (0, math.inf), (0, -1)])
def test_invalid_service_parameters(confidence, age):
    assert window().check('cucumber', 0, confidence, age).detail == 'invalid_request'


def test_target_config_errors_fail_at_startup():
    with pytest.raises(ValueError):
        target_mapping(['a', 'b'], ['cucumber'])
    with pytest.raises(ValueError):
        target_mapping(['a', 'a'], ['cucumber', 'refrigerator'])


def test_wrong_deployment_weight_is_rejected_before_loading(tmp_path):
    from cortex_perception.detection import YoloDetector
    p = tmp_path/'wrong.pt'
    p.write_bytes(b'not the selected weight')
    with pytest.raises(ValueError, match='SHA-256'):
        YoloDetector(p, expected_sha256='0'*64)


def test_latest_frame_guard_drops_old_majority_and_recovers_on_fresh_hit():
    w = PresenceWindow(TARGETS, ['refrigerator', 'cucumber'], min_confidence=.25,
                       require_latest_hit=True)
    for i in range(3):add(w, i, [CU])
    assert w.check('cucumber', .25).found
    add(w, 3, [])
    v = w.check('cucumber', .375)
    assert not v.found and v.hits == 3 and v.samples == 4
    add(w, 4, [CU])
    assert w.check('cucumber', .5).found
    assert not w.check('cucumber', 1.2).found
