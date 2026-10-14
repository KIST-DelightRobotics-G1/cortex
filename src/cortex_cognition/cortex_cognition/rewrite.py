"""rewrite — demo-only plan patches applied between validated plan lines. Pure logic.

The LLM decomposes the task; this module never changes how. It only inserts
steps between two adjacent validated lines when that pair matches a rule in
config/plan_rewrites.yaml — for gaps that come from the demo setup or what the
VLA was trained on, not from the task (e.g. the arm cannot reach the cucumber
from where the fridge door was opened). Verbs used here live in that file, not
in actions.yaml, so the LLM never sees them. Delete a rule, or the whole file,
and plans go back to exactly what the LLM produced.

    Rewriter.load(path, cfg)      '' or a missing file → no rules
    rw.between(prev, nxt)         Insert specs for the pair (prev, nxt), in order;
                                  prev is None for the plan's first line

Matching: a pattern names an action (or a list of them) and optionally leading
args; `args: [cucumber]` matches pick(cucumber) and take_out(cucumber, fridge).
Because lines stream in order, the pair is checked when the second line
arrives — always before it is dispatched.

auto_open (a plan fix, not a demo patch): taking something out of a place with
a door — take_out(x, <place>), or pick right after move_to(<place>) — gets an
open(<door>) in front of it unless the line before already opens that door. The
LLM drops that open about one time in ten. The rules are then matched against
(open, take) as if the LLM had written the open, so demo inserts still apply.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import yaml

from . import planner


class RewriteConfigError(ValueError):
    """plan_rewrites.yaml is malformed — raised at load so the node fails fast."""


@dataclass
class Insert:
    rule: str
    action: str
    args: list
    exec: str
    instruction: str
    title: str
    say: str = ''            # spoken when the inserted step starts; '' = silent


class Rewriter:
    def __init__(self, spec: dict | None, cfg: dict) -> None:
        spec = spec or {}
        self.cfg = cfg
        self.verbs = spec.get('verbs') or {}
        self.rules = [r for r in (spec.get('rules') or []) if r.get('enabled', True)]
        auto = spec.get('auto_open') or {}
        self.doors = dict(auto.get('doors') or {})           # place -> its door
        self.open_before = set(auto.get('before') or [])
        self.open_say = str(auto.get('say') or '')
        self._check()

    @classmethod
    def load(cls, path: str, cfg: dict) -> 'Rewriter':
        if not path or not os.path.exists(path):
            return cls(None, cfg)
        with open(path, encoding='utf-8') as f:
            return cls(yaml.safe_load(f), cfg)

    @property
    def names(self) -> list:
        return (['auto_open'] if self.doors else []) + [r['name'] for r in self.rules]

    # ------------------------------------------------------------------ match
    def between(self, prev_action, prev_args, nxt_action: str, nxt_args: list) -> list:
        """Steps to insert in front of nxt. prev_action is None for the plan's first line."""
        out = []
        door = self._missing_door(prev_action, prev_args or [], nxt_action, nxt_args)
        if door:
            out.append(self._insert('auto_open', {'a': 'open', 'args': [door],
                                                  'say': self.open_say}))
            prev_action, prev_args = 'open', [door]     # the rules see the open we just added
        if prev_action is None:
            return out
        for r in self.rules:
            a, b = r['between']
            if _matches(a, prev_action, prev_args) and _matches(b, nxt_action, nxt_args):
                out += [self._insert(r['name'], ins) for ins in r['insert']]
        return out

    def _missing_door(self, prev_action, prev_args: list, nxt_action: str, nxt_args: list):
        """The door nxt needs opened first, or None (no door, or prev already opens it)."""
        if not self.doors or nxt_action not in self.open_before:
            return None
        if nxt_action == 'take_out' and len(nxt_args) > 1:
            place = nxt_args[1]
        elif prev_action == 'move_to' and prev_args:
            place = prev_args[0]
        else:
            return None
        door = self.doors.get(place)
        if door is None or (prev_action == 'open' and prev_args[:1] == [door]):
            return None
        return door

    def _insert(self, rule: str, ins: dict) -> Insert:
        verb, args = ins['a'], list(ins.get('args', []))
        say = planner.fill(self.cfg, ins.get('say', ''), args[0] if args else '')
        v = self.verbs.get(verb)
        if v is not None:                        # demo-only verb defined in this file
            # VLA training sentence (vla_prompts.yaml) first, this file's template otherwise
            en = self.cfg.get('english', {})
            instruction = planner.vla_prompt(self.cfg, verb, args) or v.get('instruction', '').format(
                *[en.get(x, x.replace('_', ' ')) for x in args])
            title = v.get('title', '{0} ' + v.get('ko', verb)).format(
                *[planner.ko_name(self.cfg, x) for x in args])
            return Insert(rule, verb, args, v['exec'], instruction, title, say)
        # a regular action from actions.yaml
        return Insert(rule, verb, args, planner.exec_of(self.cfg, verb),
                      planner.instruction_for(self.cfg, verb, args),
                      planner.step_title(self.cfg, verb, args), say)

    # --------------------------------------------------------------- validate
    def _check(self) -> None:
        actions = self.cfg.get('actions', {})
        if self.doors:
            if 'open' not in actions:
                raise RewriteConfigError('auto_open needs the open action in actions.yaml')
            unknown = self.open_before - set(actions)
            if not self.open_before or unknown:
                raise RewriteConfigError(f'auto_open.before must list known actions (unknown: {sorted(unknown)})')
            if not all(isinstance(p, str) and isinstance(d, str) and p and d for p, d in self.doors.items()):
                raise RewriteConfigError('auto_open.doors must map place -> door names')
        for name, v in self.verbs.items():
            if v.get('exec') not in ('nav', 'vla'):
                raise RewriteConfigError(f'verb {name!r}: exec must be nav or vla')
            if v['exec'] == 'vla' and not v.get('instruction'):
                raise RewriteConfigError(f'verb {name!r}: a vla verb needs an instruction')
        seen = set()
        for r in self.rules:
            name = r.get('name')
            if not name or name in seen:
                raise RewriteConfigError(f'rule name missing or repeated: {name!r}')
            seen.add(name)
            pair = r.get('between')
            if not isinstance(pair, list) or len(pair) != 2 or not all('a' in p for p in pair):
                raise RewriteConfigError(f'rule {name!r}: between must be two patterns with "a"')
            if not r.get('insert'):
                raise RewriteConfigError(f'rule {name!r}: insert is empty')
            for ins in r['insert']:
                verb = ins.get('a')
                if verb not in self.verbs and verb not in self.cfg.get('actions', {}):
                    raise RewriteConfigError(f'rule {name!r}: unknown verb {verb!r}')


def _matches(pat: dict, action: str, args: list) -> bool:
    acts = pat['a'] if isinstance(pat['a'], list) else [pat['a']]
    if action not in acts:
        return False
    want = pat.get('args', [])
    return list(args[:len(want)]) == list(want)
