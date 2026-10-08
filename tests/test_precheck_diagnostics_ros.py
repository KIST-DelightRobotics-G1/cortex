"""Exercise actual orchestrator service wrapper with generated Humble types."""
from types import SimpleNamespace
import json

import pytest
pytest.importorskip('rclpy')
from cortex_cognition.orchestrator_node import OrchestratorNode


@pytest.mark.parametrize('found,detail', [(True, ''), (False, ''), (False, 'insufficient_frames')])
def test_service_diagnostics_preserve_result_and_include_plan_context(found, detail):
    messages = []
    result = SimpleNamespace(found=found, detail=detail, label='refrigerator', confidence=.8)
    client = SimpleNamespace(service_is_ready=lambda: True,
                             call_async=lambda request: SimpleNamespace(done=lambda: True, result=lambda: result))
    node = SimpleNamespace(det_client=client, _det_timeout=.3,
                           _det_diag_pub=SimpleNamespace(publish=messages.append),
                           _exec=SimpleNamespace(plan_id='test-plan', cur=2),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123)))
    answer = OrchestratorNode._check_target(node, 'fridge_door')
    assert answer == (found, 'refrigerator 0.80' if found else detail)
    evidence = json.loads(messages[0].data)
    assert evidence['plan_id'] == 'test-plan' and evidence['index'] == 2
    assert evidence['found'] == found and evidence['elapsed_ms'] >= 0
    node._det_diag_pub = None
    assert OrchestratorNode._check_target(node, 'fridge_door') == answer
