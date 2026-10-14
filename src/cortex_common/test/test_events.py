"""EventLog without ROS: a stand-in node with one parameter."""
import json
from types import SimpleNamespace

from cortex_common.events import EventLog, parse_topics


def node(value):
    logs = {'info': [], 'warning': []}
    params = {}
    return SimpleNamespace(
        has_parameter=lambda name: name in params,
        declare_parameter=lambda name, default: params.setdefault(name, value),
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        get_logger=lambda: SimpleNamespace(info=logs['info'].append,
                                           warning=logs['warning'].append)), logs


def test_parse_accepts_string_or_list():
    assert parse_topics('plan, precheck  subtask') == {'plan', 'precheck', 'subtask'}
    assert parse_topics(['plan', ' ']) == {'plan'}
    assert parse_topics('') == frozenset()


def test_only_named_topics_print_one_json_line():
    n, logs = node('plan,precheck')
    ev = EventLog(n, known=('plan', 'precheck', 'subtask'))
    ev('plan', 'PLAN_STEP', index=0, title='냉장고로 이동')
    ev('subtask', 'SUBTASK_SENT', index=0)
    assert len(logs['info']) == 1 and not logs['warning']
    name, body = logs['info'][0].split(' ', 1)
    assert name == 'PLAN_STEP' and json.loads(body) == {'index': 0, 'title': '냉장고로 이동'}


def test_all_and_unknown_topics():
    n, logs = node('all,plann')
    ev = EventLog(n, known=('plan',))
    assert ev.on('plan') and ev.on('anything')
    assert 'plann' in logs['warning'][0]


def test_off_by_default():
    n, logs = node('')
    ev = EventLog(n, known=('plan',))
    ev('plan', 'PLAN_STEP')
    assert not logs['info'] and not ev.on('plan')
