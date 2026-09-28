# -*- coding: utf-8 -*-
"""plan_rewrites.yaml: demo-only steps inserted between validated plan lines."""
import os

import pytest

from cortex_cognition import executor as ex
from cortex_cognition import planner
from cortex_cognition import rewrite
from test_executor import CFG, CUCUMBER, END, SUB, Harness, run_step

HERE = os.path.dirname(os.path.abspath(__file__))
RW_PATH = os.path.join(HERE, '..', 'config', 'plan_rewrites.yaml')
RW = rewrite.Rewriter.load(RW_PATH, CFG)

# the demo plan as the LLM streams it (no approach / step_back)
FRIDGE = [('move_to', ['fridge']), ('open', ['fridge_door']), ('pick', ['cucumber']),
          ('close', ['fridge_door']), ('move_to', ['user']), ('handover', ['cucumber', 'user'])]


def actions(h):
    return [(s.index, s.action) for _, s in sorted(h.x.steps.items())]


# --- matching -----------------------------------------------------------------------

def test_shipped_file_has_the_two_fridge_rules():
    assert RW.names == ['fridge_reach', 'fridge_back_off']


def test_pair_match_and_built_step():
    [ins] = RW.between('open', ['fridge_door'], 'pick', ['cucumber'])
    assert (ins.action, ins.args, ins.exec) == ('approach', ['fridge'], 'vla')
    assert ins.instruction == 'Approach the refrigerator.'
    assert ins.title == '냉장고 앞으로 다가가기'
    [ins] = RW.between('pick', ['cucumber'], 'close', ['fridge_door'])
    assert ins.instruction == 'Step back from the refrigerator.' and ins.title == '냉장고에서 물러나기'


def test_leading_args_match_take_out_too():
    assert RW.between('open', ['fridge_door'], 'take_out', ['cucumber', 'fridge'])


def test_other_pairs_do_not_match():
    assert RW.between('open', ['cupboard_door'], 'pick', ['cucumber']) == []
    assert RW.between('open', ['fridge_door'], 'pick', ['apple']) == []
    assert RW.between('pick', ['cucumber'], 'move_to', ['user']) == []


def test_no_file_means_no_rules():
    assert rewrite.Rewriter.load('', CFG).between('open', ['fridge_door'], 'pick', ['cucumber']) == []


def test_disabled_rule_is_skipped():
    rw = rewrite.Rewriter({'verbs': RW.verbs, 'rules': [
        {'name': 'x', 'enabled': False, 'between': [{'a': 'open'}, {'a': 'pick'}],
         'insert': [{'a': 'approach', 'args': ['fridge']}]}]}, CFG)
    assert rw.names == [] and rw.between('open', ['fridge_door'], 'pick', ['cucumber']) == []


@pytest.mark.parametrize('spec, why', [
    ({'verbs': {'v': {'exec': 'arm'}}}, 'exec must be'),
    ({'verbs': {'v': {'exec': 'vla'}}}, 'needs an instruction'),
    ({'rules': [{'name': 'r', 'between': [{'a': 'open'}], 'insert': [{'a': 'pick'}]}]}, 'two patterns'),
    ({'rules': [{'name': 'r', 'between': [{'a': 'open'}, {'a': 'pick'}], 'insert': [{'a': 'fly'}]}]},
     'unknown verb'),
])
def test_malformed_file_fails_at_load(spec, why):
    with pytest.raises(rewrite.RewriteConfigError, match=why):
        rewrite.Rewriter(spec, CFG)


# --- executor -------------------------------------------------------------------------

def test_streamed_fridge_plan_gets_both_inserts_and_renumbers():
    h = Harness(rewriter=RW)
    h.x.heard('p1', '냉장고에서 오이 가져와줘')
    h.plan('p1', FRIDGE)
    assert actions(h) == [(0, 'move_to'), (1, 'open'), (2, 'approach'), (3, 'pick'),
                          (4, 'step_back'), (5, 'close'), (6, 'move_to'), (7, 'handover')]
    assert h.x.count == 8
    assert (ex.T_PLAN_END, 8, '계획 8단계') in h.traces
    lines = [(i, t) for k, i, t in h.traces if k == ex.T_PLAN_LINE]
    assert (2, '냉장고 앞으로 다가가기') in lines and (4, '냉장고에서 물러나기') in lines


def test_inserted_steps_run_in_order_silently():
    h = Harness(rewriter=RW)
    h.x.heard('p1', 'x')
    h.plan('p1', FRIDGE)
    for i, (_, a) in enumerate(actions(h)):
        which = 'nav' if a == 'move_to' else 'vla'
        assert h.cmds[-1] == (which, i, a, 'p1')
        says_before = len(h.says)
        run_step(h, which, 'p1', i)
        if i + 1 < 8 and actions(h)[i + 1][1] in ('approach', 'step_back'):
            assert len(h.says) == says_before     # nothing spoken for the next (inserted) step
    assert h.x.phase == 'IDLE' and ex.T_PLAN_DONE in h.kinds()
    assert (ex.T_STEP_START, 2, '냉장고 앞으로 다가가기') in h.traces   # NOW shows the title


def test_insert_is_sent_with_its_instruction():
    sent = []
    h = Harness(rewriter=RW)
    h.x.p.send_cmd = lambda which, step, pid: sent.append((step.action, step.instruction))
    h.x.heard('p1', 'x')
    h.plan('p1', FRIDGE)
    run_step(h, 'nav', 'p1', 0)
    run_step(h, 'vla', 'p1', 1)
    assert sent[-1] == ('approach', 'Approach the refrigerator.')


def test_line_arriving_while_waiting_gets_its_insert_first():
    h = Harness(rewriter=RW)
    h.x.heard('p1', 'x')
    h.x.on_step('p1', SUB, 0, 'move_to', ['fridge'], '냉장고로 갑니다.', '', '')
    h.x.on_step('p1', SUB, 1, 'open', ['fridge_door'], '문을 엽니다.', '', '')
    run_step(h, 'nav', 'p1', 0)
    run_step(h, 'vla', 'p1', 1)
    assert h.x._st == 'WAIT_STEP'                 # pick not streamed yet
    h.x.on_step('p1', SUB, 2, 'pick', ['cucumber'], '오이를 집습니다.', '', '')
    assert h.cmds[-1] == ('vla', 2, 'approach', 'p1')   # the insert goes first


def test_without_rewriter_the_plan_is_untouched():
    h = Harness()
    h.x.heard('p1', 'x')
    h.plan('p1', FRIDGE)
    assert [a for _, a in actions(h)] == [a for a, _ in FRIDGE] and h.x.count == 6


def test_other_scene_is_untouched():
    h = Harness(rewriter=RW)
    h.x.heard('p1', '이 컵 찬장에 넣어줘')
    h.plan('p1', [('pick', ['cup']), ('open', ['cupboard_door']), ('put_in', ['cup', 'cupboard']),
                  ('close', ['cupboard_door'])])
    assert h.x.count == 4 and 'approach' not in [a for _, a in actions(h)]


def test_preempting_plan_is_rewritten_too():
    h = Harness(rewriter=RW)
    h.x.heard('p1', '테이블로 가')
    h.plan('p1', [('move_to', ['table'])])
    h.state('nav', ex.RUNNING, 'p1', 0)
    h.x.heard('p2', '냉장고에서 오이 가져와줘')
    for i, (a, args) in enumerate(FRIDGE):
        h.x.on_step('p2', SUB, i, a, args, f'{a} say', '', '')
    h.x.on_step('p2', END, len(FRIDGE), '', [], '', '', '')
    h.state('nav', ex.IDLE, '', 0, 'cancelled')   # p1 stopped → p2 starts
    assert h.x.plan_id == 'p2' and h.x.count == 8
    assert [a for _, a in actions(h)][2] == 'approach'
    assert planner.exec_of(CFG, 'pick') == 'vla'  # sanity: fixtures still line up
