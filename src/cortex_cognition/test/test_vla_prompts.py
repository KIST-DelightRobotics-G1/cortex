# -*- coding: utf-8 -*-
"""vla_prompts.yaml — VLA training sentences take precedence over the templates."""
import os

import pytest

from cortex_cognition import planner, rewrite

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config')
CFG_PATH = os.path.join(CONFIG, 'actions.yaml')
SENTENCE = 'Open the right door of the refrigerator. Hook the yellow tip under the handle.'


def cfg_with(prompts: dict) -> dict:
    cfg = planner.load_config(CFG_PATH)
    cfg['vla_prompts'] = prompts
    return cfg


def write(tmp_path, body: str) -> str:
    p = tmp_path / 'vla_prompts.yaml'
    p.write_text(body, encoding='utf-8')
    return str(p)


def known():
    cfg = planner.load_config(CFG_PATH)
    rw = rewrite.Rewriter.load(os.path.join(CONFIG, 'plan_rewrites.yaml'), cfg)
    return set(cfg['actions']) | set(rw.verbs)


# --- lookup ---------------------------------------------------------------------------

def test_prompt_key_format():
    assert planner.prompt_key('open', ['fridge_door']) == 'open(fridge_door)'
    assert planner.prompt_key('take_out', ['cucumber', 'fridge']) == 'take_out(cucumber,fridge)'


def test_table_sentence_wins_over_template():
    cfg = cfg_with({'open(fridge_door)': SENTENCE})
    assert planner.instruction_for(cfg, 'open', ['fridge_door']) == SENTENCE


def test_missing_entry_falls_back_to_template():
    cfg = cfg_with({'open(fridge_door)': SENTENCE})
    assert planner.instruction_for(cfg, 'pick', ['cucumber']) == 'Pick up the cucumber.'
    # different args → different key → template
    assert planner.instruction_for(cfg, 'open', ['drawer']) == 'Open the drawer.'


def test_no_table_is_the_old_behaviour():
    cfg = planner.load_config(CFG_PATH)
    assert planner.instruction_for(cfg, 'open', ['fridge_door']) == 'Open the refrigerator door.'


def test_rewrite_verb_uses_the_table_too():
    cfg = cfg_with({'approach(fridge)': 'Walk up to the fridge.'})
    rw = rewrite.Rewriter.load(os.path.join(CONFIG, 'plan_rewrites.yaml'), cfg)
    ins = rw.between('open', ['fridge_door'], 'pick', ['cucumber'])
    assert [(i.action, i.instruction) for i in ins] == [('approach', 'Walk up to the fridge.')]


def test_rewrite_verb_without_entry_keeps_its_template():
    cfg = cfg_with({})
    rw = rewrite.Rewriter.load(os.path.join(CONFIG, 'plan_rewrites.yaml'), cfg)
    ins = rw.between('open', ['fridge_door'], 'pick', ['cucumber'])
    assert ins[0].instruction == 'Approach the refrigerator.'


# --- load / validate ------------------------------------------------------------------

def test_load_splits_filled_and_empty(tmp_path):
    path = write(tmp_path, f'prompts:\n  "open(fridge_door)": "{SENTENCE}"\n'
                           '  "pick( cucumber )": ""\n  "close(fridge_door)":\n')
    filled, empty = planner.load_vla_prompts(path, known())
    assert filled == {'open(fridge_door)': SENTENCE}
    assert empty == ['pick(cucumber)', 'close(fridge_door)']   # spaces ignored, None = empty


def test_sentence_is_kept_verbatim(tmp_path):
    path = write(tmp_path, 'prompts:\n  "open(fridge_door)": "Open it.  Now"\n')
    filled, _ = planner.load_vla_prompts(path, known())
    assert filled['open(fridge_door)'] == 'Open it.  Now'


def test_no_path_or_missing_file_is_off(tmp_path):
    assert planner.load_vla_prompts('', known()) == ({}, [])
    assert planner.load_vla_prompts(str(tmp_path / 'nope.yaml'), known()) == ({}, [])


@pytest.mark.parametrize('key', ['open fridge_door', 'open[fridge_door]', 'Open(fridge_door)',
                                 'open(fridge_door,)', 'open(,x)', 'open(a,,b)'])
def test_bad_key_fails_fast(tmp_path, key):
    path = write(tmp_path, f'prompts:\n  "{key}": "x"\n')
    with pytest.raises(planner.PromptConfigError):
        planner.load_vla_prompts(path, known())


def test_unknown_action_fails_fast(tmp_path):
    path = write(tmp_path, 'prompts:\n  "teleport(fridge)": "x"\n')
    with pytest.raises(planner.PromptConfigError, match='unknown action'):
        planner.load_vla_prompts(path, known())


def test_duplicate_key_after_normalising_fails(tmp_path):
    path = write(tmp_path, 'prompts:\n  "pick(cucumber)": "a"\n  "pick( cucumber)": "b"\n')
    with pytest.raises(planner.PromptConfigError, match='twice'):
        planner.load_vla_prompts(path, known())


def test_non_string_sentence_fails(tmp_path):
    path = write(tmp_path, 'prompts:\n  "pick(cucumber)": [a, b]\n')
    with pytest.raises(planner.PromptConfigError, match='string'):
        planner.load_vla_prompts(path, known())


def test_shipped_file_loads_and_covers_the_demo():
    filled, empty = planner.load_vla_prompts(os.path.join(CONFIG, 'vla_prompts.yaml'), known())
    keys = set(filled) | set(empty)
    for k in ('open(fridge_door)', 'approach(fridge)', 'pick(cucumber)',
              'step_back(fridge)', 'close(fridge_door)', 'place(cucumber,table)'):
        assert k in keys
