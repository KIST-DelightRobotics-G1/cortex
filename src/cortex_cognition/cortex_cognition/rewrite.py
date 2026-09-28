"""rewrite — demo-only plan patches applied between validated plan lines. Pure logic.

The LLM decomposes the task; this module never changes how. It only inserts
steps between two adjacent validated lines when that pair matches a rule in
config/plan_rewrites.yaml — for gaps that come from the demo setup or what the
VLA was trained on, not from the task (e.g. the arm cannot reach the cucumber
from where the fridge door was opened). Verbs used here live in that file, not
in actions.yaml, so the LLM never sees them. Delete a rule, or the whole file,
and plans go back to exactly what the LLM produced.

    Rewriter.load(path, cfg)      '' or a missing file → no rules
    rw.between(prev, nxt)         Insert specs for the pair (prev, nxt), in order

Matching: a pattern names an action (or a list of them) and optionally leading
args; `args: [cucumber]` matches pick(cucumber) and take_out(cucumber, fridge).
Because lines stream in order, the pair is checked when the second line
arrives — always before it is dispatched.
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


class Rewriter:
    def __init__(self, spec: dict | None, cfg: dict) -> None:
        spec = spec or {}
        self.cfg = cfg
        self.verbs = spec.get('verbs') or {}
        self.rules = [r for r in (spec.get('rules') or []) if r.get('enabled', True)]
        self._check()

    @classmethod
    def load(cls, path: str, cfg: dict) -> 'Rewriter':
        if not path or not os.path.exists(path):
            return cls(None, cfg)
        with open(path, encoding='utf-8') as f:
            return cls(yaml.safe_load(f), cfg)

    @property
    def names(self) -> list:
        return [r['name'] for r in self.rules]

    # ------------------------------------------------------------------ match
    def between(self, prev_action: str, prev_args: list,
                nxt_action: str, nxt_args: list) -> list:
        out = []
        for r in self.rules:
            a, b = r['between']
            if _matches(a, prev_action, prev_args) and _matches(b, nxt_action, nxt_args):
                out += [self._insert(r['name'], ins) for ins in r['insert']]
        return out

    def _insert(self, rule: str, ins: dict) -> Insert:
        verb, args = ins['a'], list(ins.get('args', []))
        v = self.verbs.get(verb)
        if v is not None:                        # demo-only verb defined in this file
            en = self.cfg.get('english', {})
            instruction = v.get('instruction', '').format(
                *[en.get(x, x.replace('_', ' ')) for x in args])
            title = v.get('title', '{0} ' + v.get('ko', verb)).format(
                *[planner.ko_name(self.cfg, x) for x in args])
            return Insert(rule, verb, args, v['exec'], instruction, title)
        # a regular action from actions.yaml
        return Insert(rule, verb, args, planner.exec_of(self.cfg, verb),
                      planner.instruction_for(self.cfg, verb, args),
                      planner.step_title(self.cfg, verb, args))

    # --------------------------------------------------------------- validate
    def _check(self) -> None:
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
