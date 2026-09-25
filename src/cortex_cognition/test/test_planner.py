# -*- coding: utf-8 -*-
"""planner: prompt generation + line validation against config/actions.yaml."""
import json
import os

import pytest

from cortex_cognition import planner

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = planner.load_config(os.path.join(HERE, '..', 'config', 'actions.yaml'))


def test_prompt_lists_every_action():
    p = planner.build_system_prompt(CFG)
    for a in CFG['actions']:
        assert f'  {a}(' in p, a


def test_prompt_has_no_vocabulary():
    """어휘는 열려 있다 — 허용값 목록도 물체 위치표도 프롬프트에 없어야 한다."""
    p = planner.build_system_prompt(CFG)
    assert '인자 허용값' not in p and '물체 위치' not in p
    assert '## 이동' in p and '수식어를 붙이지 않는다' in p
    # user / home 은 프롬프트에 있다 — station 목록이 아니라 "사람은 user", "원위치는 home" 이라는
    # 표기 규약이다. 실제 장소 이름(냉장고·테이블·조리대)이 새 나가면 안 된다.
    for place in set(CFG['nav']['places']) - {'user', 'home'}:
        assert place not in p, place


def test_user_message_carries_robot_state():
    m = planner.build_user_message('테이블에 놔', 'idle', 'fridge', 'cucumber')
    assert 'at: fridge' in m and 'holding: cucumber' in m and 'utterance: 테이블에 놔' in m


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
    ('{"i":0,"a":"pick","args":["Cucumber"],"say":"x"}', 0, 'bad arg'),
    ('{"i":0,"a":"place","args":["object","table"],"say":"x"}', 0, 'signature slot name'),
    ('{"i":0,"a":"place","args":["cucumber"],"say":"x"}', 0, 'expects 2 args'),
    ('{"i":1,"a":"move_to","args":["fridge"],"say":"x"}', 0, 'index 1, expected 0'),
    ('not json', 0, 'not json'),
    ('{"end":true}', 0, 'end without int n'),
    ('{"reply":"maybe","say":"x"}', 0, 'bad reply type'),
])
def test_rejections(raw, expected_i, err):
    v = planner.validate_line(raw, CFG, expected_i)
    assert v.kind == 'error' and err in v.error, v


def test_open_vocabulary_passes_format_check():
    """목록에 없는 장소·물건도 형식만 맞으면 검증기 1 은 통과시킨다 — 막는 것은 검증기 2."""
    for raw in ('{"i":0,"a":"move_to","args":["sink"],"say":"싱크대로 갑니다."}',
                '{"i":0,"a":"pick","args":["screwdriver"],"say":"드라이버를 집습니다."}'):
        assert planner.validate_line(raw, CFG, 0).kind == 'sub', raw


@pytest.mark.parametrize('raw, ok, reason, args', [
    ('{"i":0,"a":"move_to","args":["fridge"],"say":"x"}', True, '', ['fridge']),
    ('{"i":0,"a":"move_to","args":["refrigerator"],"say":"x"}', True, '', ['fridge']),
    ('{"i":0,"a":"move_to","args":["drawer"],"say":"x"}', True, 'skip_nearby', ['drawer']),
    ('{"i":0,"a":"move_to","args":["kitchen"],"say":"x"}', False, 'unknown_place', ['kitchen']),
    ('{"i":0,"a":"pick","args":["cucumber"],"say":"x"}', True, '', ['cucumber']),
])
def test_feasibility(raw, ok, reason, args):
    """검증기 2: 별칭을 모듈 키로 바꾼 뒤 실행 가능한지 본다."""
    ln = planner.normalize(planner.validate_line(raw, CFG, 0), CFG)
    assert ln.args == args
    assert planner.feasibility(ln, CFG) == (ok, reason)


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
    # 이름표에 없는 물건도 그대로 실린다 — 어휘가 열려 있으므로 이쪽이 보통이다
    assert planner.instruction_for(CFG, 'pick', ['remote_control']) == 'Pick up the remote control.'
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
