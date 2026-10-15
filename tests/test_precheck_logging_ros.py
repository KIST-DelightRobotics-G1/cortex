"""Exercise the actual console/service adapter with generated Humble messages."""
import json
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 Humble and colcon-built interfaces')
from builtin_interfaces.msg import Time
from cortex_cognition import executor as ex
from cortex_cognition.orchestrator_node import OrchestratorNode
from cortex_common.events import EventLog


def context(enabled=True):
    logs = []
    node = SimpleNamespace(
        _det_timeout=0., _last_precheck={},
        _exec=SimpleNamespace(plan_id='p-test', cur=2, steps={2: SimpleNamespace(action='pick')}),
        get_logger=lambda: SimpleNamespace(info=logs.append, warning=logs.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1))))
    node.events = EventLog(node, known=('plan', 'precheck'),
                           topics='plan,precheck' if enabled else '')
    return node, logs


@pytest.mark.parametrize('enabled', [True, False])
@pytest.mark.parametrize('found,detail', [(True, ''), (False, ''), (False, 'insufficient_frames')])
def test_console_result_includes_plan_without_changing_service_result(enabled, found, detail):
    node, logs = context(enabled)
    result = SimpleNamespace(found=found, detail=detail, label='cucumber', confidence=.8)
    node.det_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: SimpleNamespace(done=lambda: True, result=lambda: result))
    answer = OrchestratorNode._check_target(node, 'cucumber')
    assert answer == (found, 'cucumber 0.80' if found else detail)
    if enabled:
        assert logs[0].startswith('PRECHECK_RESULT ')
        event = json.loads(logs[0].split(' ', 1)[1])
        assert event['plan_id'] == 'p-test' and event['index'] == 2
        assert event['action'] == 'pick' and event['target'] == 'cucumber'
        assert event['service_ms'] >= 0. and event['found'] == found
    else:
        assert logs == []


@pytest.mark.parametrize('failure', ['no_detector', 'detector_timeout', 'detector_error'])
def test_console_logs_service_failures_without_masking_them(failure):
    node, logs = context()
    cancelled = []
    def result():
        raise RuntimeError('service failed')
    node.det_client = SimpleNamespace(
        service_is_ready=lambda: failure != 'no_detector',
        call_async=lambda request: SimpleNamespace(done=lambda: failure != 'detector_timeout',
                                                   cancel=lambda: cancelled.append(True), result=result))
    assert OrchestratorNode._check_target(node, 'cucumber') == (False, failure)
    assert json.loads(logs[0].split(' ', 1)[1])['detail'] == failure
    assert bool(cancelled) == (failure == 'detector_timeout')


def test_existing_trace_events_are_mirrored_without_changing_messages():
    node, logs = context()
    published = []
    node.trace_pub = SimpleNamespace(publish=published.append)
    OrchestratorNode._trace(node, ex.T_GROUND, 'p-test', 2, '오이 확인 시간 초과', 'not_visible')
    assert published[0].title == '오이 확인 시간 초과'
    assert published[0].body == 'not_visible'
    event = json.loads(logs[0].split(' ', 1)[1])
    assert logs[0].startswith('PRECHECK_STATE ') and event['detail'] == 'not_visible'
    OrchestratorNode._trace(node, ex.T_CANCEL, 'p-test', -1, '계획 중단', 'precheck_timeout: cucumber: not_visible')
    assert logs[1].startswith('PLAN_CANCEL ')


def test_repeated_precheck_answer_is_printed_once_per_step():
    node, logs = context()
    result = SimpleNamespace(found=False, detail='', label='', confidence=0.)
    node.det_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: SimpleNamespace(done=lambda: True, result=lambda: result))
    for _ in range(5):                                    # retries every 0.1 s, same answer
        OrchestratorNode._check_target(node, 'cucumber')
    result.found, result.label, result.confidence = True, 'cucumber', .9
    OrchestratorNode._check_target(node, 'cucumber')      # the answer changed
    node._exec.cur = 3                                    # next step asks again
    OrchestratorNode._check_target(node, 'cucumber')
    events = [json.loads(line.split(' ', 1)[1]) for line in logs if line.startswith('PRECHECK_RESULT ')]
    assert [(e['index'], e['found']) for e in events] == [(2, False), (2, True), (3, True)]
