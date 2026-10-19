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


@pytest.mark.parametrize('age,label,detail,expected', [
    (.2, 'cucumber', '', True), (.4, 'cucumber', '', False),  # predates PRECHECK
    (1., 'cucumber', '', False), (-.1, 'cucumber', '', False),
    (.2, 'refrigerator', '', False), (.2, 'cucumber', 'stub', False)])
def test_wrist_query_routes_to_separate_service_and_rejects_old_or_stub_evidence(age, label, detail, expected):
    from cortex_msgs.srv import CheckTarget
    node, logs = context()
    node._holding_max_age = .6
    node._now = lambda: 10.3
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=100_000_000_000, to_msg=lambda: Time(sec=100)))
    response = CheckTarget.Response(found=True, label=label, confidence=.9, detail=detail)
    response.stamp.sec, response.stamp.nanosec = divmod(round((100-age)*1e9), 1_000_000_000)
    requests = []
    def call(request):
        requests.append(request)
        return SimpleNamespace(done=lambda: True, result=lambda: response)
    node.wrist_client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    answer = OrchestratorNode._check_target(node, 'held_cucumber', wrist_since=10.)
    assert answer[0] is expected
    assert requests[0].target == 'held_cucumber' and requests[0].max_age_s == pytest.approx(.3)
    event = json.loads(logs[0].split(' ', 1)[1])
    assert event['source'] == 'right_wrist' and event['found'] is expected


def test_wrist_entry_does_not_send_zero_age_which_would_enable_old_history():
    node, _ = context()
    node._holding_max_age, node._now = .6, lambda: 10.
    requests = []
    node.wrist_client = SimpleNamespace(service_is_ready=lambda: True,
        call_async=lambda request: (requests.append(request) or
            SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(found=False, detail='no_frame'))))
    OrchestratorNode._check_target(node, 'held_cucumber', wrist_since=10.)
    assert 0 < requests[0].max_age_s <= 1e-5


def test_expected_holding_survives_carrying_steps_and_is_cleared_only_on_completed_release():
    from cortex_msgs.msg import SubtaskState
    node, _ = context()
    node.at, node.holding = 'fridge', 'none'
    node._PICKS, node._PUTS = OrchestratorNode._PICKS, OrchestratorNode._PUTS
    node._sent = {'vla': ('p1', 0, 'pick', ['cucumber'])}
    OrchestratorNode._update_robot_state(node, 'vla', SubtaskState(plan_id='p1', index=0, status=ex.DONE))
    assert node.holding == 'cucumber'
    node._sent['vla'] = ('p1', 1, 'step_back', ['fridge'])
    OrchestratorNode._update_robot_state(node, 'vla', SubtaskState(plan_id='p1', index=1, status=ex.DONE))
    assert node.holding == 'cucumber'
    node._sent['vla'] = ('p1', 2, 'close', ['fridge_door'])
    OrchestratorNode._update_robot_state(node, 'vla', SubtaskState(plan_id='p1', index=2, status=ex.DONE))
    assert node.holding == 'cucumber'
    node._sent['nav'] = ('p1', 3, 'move_to', ['table'])
    OrchestratorNode._update_robot_state(node, 'nav', SubtaskState(plan_id='p1', index=3, status=ex.DONE))
    assert node.holding == 'cucumber' and node.at == 'table'
    node._sent['vla'] = ('p1', 4, 'place', ['cucumber', 'table'])
    OrchestratorNode._update_robot_state(node, 'vla', SubtaskState(plan_id='p1', index=4, status=ex.FAILED))
    assert node.holding == 'cucumber'
    OrchestratorNode._update_robot_state(node, 'vla', SubtaskState(plan_id='p1', index=4, status=ex.DONE))
    assert node.holding == 'none'


@pytest.mark.parametrize('mode', ['hit', 'miss', 'no_detector', 'timeout', 'error', 'call_error'])
def test_ego_auxiliary_failure_or_delay_never_changes_wrist_verdict(monkeypatch, mode):
    from rclpy.task import Future
    import cortex_cognition.orchestrator_node as module
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    node, logs = context()
    node._det_timeout, node._ego_since, node._ego_observation = .3, None, None
    node._exec.steps[2].action = 'close'
    future, calls = Future(), []
    def call(request):
        calls.append(request)
        if mode == 'call_error':
            raise RuntimeError('transport failed')
        return future
    node.det_client = SimpleNamespace(service_is_ready=lambda: mode != 'no_detector', call_async=call)
    node._observe_ego = lambda since: OrchestratorNode._observe_ego(node, since)
    node._check_target = lambda *a, **kw: (True, 'cucumber 0.90')
    # Ego has not replied, but wrist succeeds immediately; no synchronous wait.
    for _ in range(3):
        assert OrchestratorNode._check_holding(node, 10.) == (True, 'cucumber 0.90')
    assert len(calls) == (0 if mode == 'no_detector' else 1)
    assert clock[0] == 10.
    assert all(request.target == 'cucumber' for request in calls)
    if mode in ('hit', 'miss'):
        future.set_result(SimpleNamespace(found=mode == 'hit', label='cucumber', confidence=.9, detail=''))
    elif mode == 'error':
        future.set_exception(RuntimeError('service failed'))
    elif mode == 'timeout':
        OrchestratorNode._poll_ego_observation(node)
        assert node._ego_observation is not None and not logs
    # A late reply must identify the original close, not whatever is running now.
    node._exec.plan_id, node._exec.cur = 'next-plan', 3
    clock[0] = 10.4
    OrchestratorNode._poll_ego_observation(node)
    assert node._ego_observation is None
    event = json.loads(logs[-1].split(' ', 1)[1])
    assert logs[-1].startswith('PRECHECK_AUXILIARY ')
    assert (event['plan_id'], event['index'], event['action']) == ('p-test', 2, 'close')
    assert event['source'] == 'ego' and event['required'] is False
    assert event['target'] == 'cucumber'
    assert event['found'] is (mode == 'hit')
    if mode == 'timeout':
        assert future.cancelled() and event['detail'] == 'detector_timeout'
    elif mode in ('error', 'call_error'):
        assert event['detail'] == 'detector_error'
    elif mode == 'no_detector':
        assert event['detail'] == 'no_detector'
    OrchestratorNode._poll_ego_observation(node)
    assert len(logs) == 1


def test_ego_auxiliary_logging_is_optional_and_pending_request_is_bounded():
    from rclpy.task import Future
    node, logs = context(enabled=False)
    node._ego_since, node._ego_observation = None, None
    # Logging off means no auxiliary service access at all (no client is provided).
    OrchestratorNode._observe_ego(node, 10.)
    assert node._ego_observation is None
    node.events = EventLog(node, known=('precheck',), topics='precheck')
    futures = []
    def call(request):
        futures.append(Future())
        return futures[-1]
    node.det_client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    OrchestratorNode._observe_ego(node, 10.)
    OrchestratorNode._observe_ego(node, 10.)
    assert len(futures) == 1
    OrchestratorNode._observe_ego(node, 11.)
    assert len(futures) == 2 and futures[0].cancelled()
    node._det_timeout = 0.
    OrchestratorNode._poll_ego_observation(node)
    assert futures[1].cancelled() and node._ego_observation is None
