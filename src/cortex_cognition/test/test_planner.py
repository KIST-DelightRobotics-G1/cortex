# -*- coding: utf-8 -*-
"""planner: prompt generation + line validation against config/actions.yaml."""
import json
import os

import pytest

from cortex_cognition import planner

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = planner.load_config(os.path.join(HERE, '..', 'config', 'actions.yaml'))


def test_prompt_mentions_only_enabled_actions():
    p = planner.build_system_prompt(CFG)
    for a, s in CFG['actions'].items():
        line = f'  {a}('
        assert (line in p) == bool(s.get('enabled')), a


def test_examples_validate_under_their_own_rules():
    for ex in CFG['examples']:
        if 'lines' not in ex:
            continue
        for ln in ex['lines']:
            raw = json.dumps({'i': ln['i'], 'a': ln['a'], 'args': ln['args'], 'say': ln['say']},
                             ensure_ascii=False)
            v = planner.validate_line(raw, CFG, ln['i'])
            assert v.kind == 'sub', (raw, v.error)
            assert v.say_warn == [], (raw, v.say_warn)


@pytest.mark.parametrize('raw, expected_i, err', [
    ('{"i":0,"a":"fly","args":["fridge"],"say":"x"}', 0, 'unknown action'),
    ('{"i":0,"a":"take_out","args":["cucumber","fridge"],"say":"x"}', 0, 'not enabled'),
    ('{"i":0,"a":"move_to","args":["sink"],"say":"x"}', 0, 'not in vocab'),
    ('{"i":0,"a":"place","args":["cucumber"],"say":"x"}', 0, 'expects 2 args'),
    ('{"i":1,"a":"move_to","args":["fridge"],"say":"x"}', 0, 'index 1, expected 0'),
    ('{"i":8,"a":"move_to","args":["fridge"],"say":"x"}', 8, 'exceeds limit'),
    ('not json', 0, 'not json'),
    ('{"end":true}', 0, 'end without int n'),
    ('{"reply":"maybe","say":"x"}', 0, 'bad reply type'),
])
def test_rejections(raw, expected_i, err):
    v = planner.validate_line(raw, CFG, expected_i)
    assert v.kind == 'error' and err in v.error, v


def test_reply_and_end():
    assert planner.validate_line('{"reply":"confirm","say":"오이 말씀이신가요?"}', CFG, 0).reply_type == 'confirm'
    assert planner.validate_line('{"end":true,"n":6}', CFG, 6).n == 6
    assert planner.validate_line('```json', CFG, 0).kind == 'skip'


def test_say_checks():
    assert 'promise' in planner.check_say('오이를 가져올게요.', 'pick', ['cucumber'], CFG)
    assert 'object-missing' in planner.check_say('문을 엽니다.', 'open', ['fridge_door'], CFG) or \
        'action-mismatch' not in planner.check_say('냉장고 문을 엽니다.', 'open', ['fridge_door'], CFG)
    assert planner.check_say('냉장고 문을 엽니다.', 'open', ['fridge_door'], CFG) == []


def test_derived_step_data():
    assert planner.instruction_for(CFG, 'handover', ['cucumber', 'user']) == 'Hand the cucumber to the person.'
    assert planner.instruction_for(CFG, 'move_to', ['fridge']) == ''
    assert planner.precheck_target(CFG, 'open', ['fridge_door']) == 'fridge_door'
    assert planner.precheck_target(CFG, 'handover', ['cucumber', 'user']) == 'user'
    assert planner.precheck_target(CFG, 'move_to', ['fridge']) is None
    assert planner.step_title(CFG, 'move_to', ['user']) == '사용자에게 이동'
    assert planner.step_title(CFG, 'open', ['fridge_door']) == '냉장고 문 열기'
    assert planner.step_title(CFG, 'place', ['cucumber', 'table']) == '오이 테이블에 놓기'
    assert planner.step_title(CFG, 'handover', ['cucumber', 'user']) == '오이 건네기'


def test_particles():
    assert planner.phrase(CFG, 'not_visible', 'fridge_door') == '냉장고 문이 보이지 않습니다.'
    assert planner.phrase(CFG, 'not_visible', 'cucumber') == '오이가 보이지 않습니다.'
    assert planner.phrase(CFG, 'step_failed', '냉장고 문 열기') == '냉장고 문 열기를 하지 못했습니다.'


def test_stream_lines_splits_across_chunks():
    got = []
    pieces = ['{"i":0,"a":"move_to","args":["fri', 'dge"],"say":"냉장고로 갑니다."}\n{"end":tr', 'ue,"n":1}']
    planner.stream_lines(pieces, CFG, got.append)
    assert [ln.kind for ln in got] == ['sub', 'end']
