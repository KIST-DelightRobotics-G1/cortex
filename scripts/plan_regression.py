#!/usr/bin/env python3
"""plan_regression.py — run the production planner prompt over an utterance set and score it.

Uses cortex's own planner (system prompt, validators 1/2) and plan rewriter, so a
prompt or rewrite change is measured exactly as llm_node + executor would run it.
No ROS needed. Needs GOOGLE_API_KEY (env, or the repo's .env).

    python3 scripts/plan_regression.py --repeat 5                    # this checkout
    python3 scripts/plan_regression.py --repeat 5 --ref origin/main  # the prompt/rewriter at a git ref
    python3 scripts/plan_regression.py --ids R1 R2 A1 --repeat 10
    python3 scripts/plan_regression.py --dry-run          # prompt + yaml only, no API calls

--ref <git ref> runs cortex_cognition (planner, rewrite, config/) as it is at that
ref, so a before/after comparison needs no second checkout.

Each utterance's expect list is scored against the LLM plan (after validator 2),
as in the v2 harness:
    strict seq   verbs match exactly, args within 'a|b' alternatives ('*' = any)
    loose seq    required lines appear in order; other lines only move_to/open/close/pick/take_out
    reply        reply kind matches
    infeasible   a plan was produced, loose seq holds, and validator 2 blocked a line

Two extra columns show what the executor would actually run:
    exec         the plan after config/plan_rewrites.yaml (auto-open, demo inserts)
    open_ok      for plans that take something out of a door-closed place (fridge …):
                 'yes' if an open(<door>) comes before the first pick/take_out, else 'NO'
"""

from __future__ import annotations

import argparse
import importlib
import io
import json
import os
import subprocess
import sys
import tarfile
import tempfile
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join('src', 'cortex_cognition')
planner = rewrite = None          # bound by use_package()
EXTRA_OK = {'move_to', 'open', 'close', 'pick', 'take_out'}


def use_package(ref: str) -> str:
    """Import cortex_cognition from this checkout, or as it is at a git ref. Returns its root."""
    global planner, rewrite
    base = os.path.join(ROOT, PKG)
    if ref:
        tmp = tempfile.mkdtemp(prefix='plan-regression-')
        blob = subprocess.run(['git', '-C', ROOT, 'archive', ref, PKG],
                              check=True, capture_output=True).stdout
        with tarfile.open(fileobj=io.BytesIO(blob)) as tar:
            tar.extractall(tmp)
        base = os.path.join(tmp, PKG)
    sys.path.insert(0, base)
    planner = importlib.import_module('cortex_cognition.planner')
    rewrite = importlib.import_module('cortex_cognition.rewrite')
    return base
TAKES = {'pick', 'take_out'}


# ---------------------------------------------------------------------------
# scoring (ported from llm-router-test plan/v2/run_v2.py)
# ---------------------------------------------------------------------------
def arg_ok(pat: str, val: str) -> bool:
    return pat == '*' or val in str(pat).split('|')


def line_ok(pat: list, ln) -> bool:
    verb, args = pat[0], pat[1:]
    if ln.action not in str(verb).split('|') or len(args) != len(ln.args):
        return False
    return all(arg_ok(p, v) for p, v in zip(args, ln.args))


def match_seq(exp: dict, subs: list, at: str) -> bool:
    seq = exp['seq']
    if not exp.get('loose'):
        if len(subs) == len(seq) and all(line_ok(p, ln) for p, ln in zip(seq, subs)):
            return True
        if at != 'unknown':
            return False
        k = 0
        for ln in subs:                       # unknown position: extra move_to is fair
            if k < len(seq) and line_ok(seq[k], ln):
                k += 1
            elif ln.action != 'move_to':
                return False
        return k == len(seq)
    k = 0
    for ln in subs:
        if k < len(seq) and line_ok(seq[k], ln):
            k += 1
        elif ln.action not in EXTRA_OK:
            return False
    return k == len(seq)


def score(u: dict, lines: list, blocked: list, at: str) -> tuple:
    subs = [ln for ln in lines if ln.kind == 'sub']
    replies = [ln for ln in lines if ln.kind == 'reply']
    errs = [ln for ln in lines if ln.kind == 'error']
    if errs:
        return 'invalid', errs[0].error
    if not any(ln.kind == 'end' for ln in lines):
        return 'invalid', 'no end line'
    if u.get('manual'):
        return 'manual', ''
    for exp in u['expect']:
        if 'reply' in exp:
            if replies and replies[0].reply_type == exp['reply'] and not subs:
                return 'pass', f'reply {exp["reply"]}'
        elif exp.get('infeasible'):
            if subs and not replies and match_seq(exp, subs, at) and blocked:
                return 'pass', f'blocked at {blocked[0]}'
        elif 'seq' in exp and subs and not replies and match_seq(exp, subs, at):
            return 'pass', 'seq'
    if replies:
        return 'fail', f'got reply {replies[0].reply_type}: {replies[0].say}'
    return 'fail', 'seq mismatch'


# ---------------------------------------------------------------------------
# executed plan (what the executor would run after rewrites)
# ---------------------------------------------------------------------------
def executed(subs: list, rw) -> list:
    """(action, args, say) after the rewriter, mirroring Executor._place.

    Older refs' rewriter has no first-line call and no Insert.say — handled here."""
    out = []
    for ln in subs:
        prev = out[-1] if out else None
        if prev is not None or hasattr(rw, 'doors'):
            for ins in rw.between(prev[0] if prev else None, prev[1] if prev else None,
                                  ln.action, list(ln.args)):
                out.append((ins.action, list(ins.args), getattr(ins, 'say', '')))
        out.append((ln.action, list(ln.args), ln.say))
    return out


def open_check(plan: list, doors: dict) -> str:
    """'' if the plan takes nothing from a door-closed place; else yes/NO for open first."""
    place, opened = None, set()
    for action, args, _ in plan:
        if action == 'move_to':
            place = args[0] if args else None
        elif action == 'open' and args:
            opened.add(args[0])
        elif action in TAKES:
            src = args[1] if action == 'take_out' and len(args) > 1 else place
            door = doors.get(src)
            if door:
                return 'yes' if door in opened else 'NO'
    return ''


def fmt(plan) -> str:
    return ' → '.join(f'{a}({", ".join(x)})' for a, x, *_ in plan)


# ---------------------------------------------------------------------------
def load_key() -> str:
    key = os.environ.get('GOOGLE_API_KEY', '')
    env = os.path.join(ROOT, '.env')
    if not key and os.path.exists(env):
        for line in io.open(env, encoding='utf-8'):
            if line.startswith('GOOGLE_API_KEY='):
                key = line.split('=', 1)[1].strip().strip('"').strip("'")
    return key


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--utterances', default=os.path.join(HERE, 'plan_regression.yaml'))
    ap.add_argument('--model', default='gemini-3.6-flash')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--ids', nargs='*')
    ap.add_argument('--ref', default='', help='git ref to take cortex_cognition from (default: this checkout)')
    ap.add_argument('--label', default='')
    ap.add_argument('--out', default=None)
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    a.label = a.label or (a.ref.replace('/', '-') if a.ref else 'worktree')

    config = os.path.join(use_package(a.ref), 'config')
    cfg = planner.load_config(os.path.join(config, 'actions.yaml'))
    rw = rewrite.Rewriter.load(os.path.join(config, 'plan_rewrites.yaml'), cfg)
    # the regression's door table is fixed, so old and new refs are judged alike
    doors = dict(getattr(rw, 'doors', None) or {'fridge': 'fridge_door', 'cupboard': 'cupboard_door'})
    spec = yaml.safe_load(io.open(a.utterances, encoding='utf-8'))
    utts = spec['utterances']
    if a.ids:
        utts = [u for u in utts if u['id'] in a.ids]
    out = a.out or os.path.join(ROOT, 'out', f'plan-regression-{a.label}-{time.strftime("%Y%m%d-%H%M%S")}')
    os.makedirs(out, exist_ok=True)
    io.open(os.path.join(out, 'prompt.txt'), 'w', encoding='utf-8').write(planner.build_system_prompt(cfg))
    print(f'{len(utts)} utterances x {a.repeat}, model {a.model}, cortex_cognition @ '
          f'{a.ref or "this checkout"}, rewrites {rw.names} -> {out}')
    if a.dry_run:
        return 0
    key = load_key()
    if not key:
        print('GOOGLE_API_KEY is not set (env or .env)', file=sys.stderr)
        return 2
    client = planner.make_client(a.model, key)

    rows = []
    jl = io.open(os.path.join(out, 'raw.jsonl'), 'w', encoding='utf-8')
    for u in utts:
        at, holding = u.get('at', 'home'), u.get('holding', 'none')
        for r in range(a.repeat):
            lines, err = [], ''
            try:
                dt = planner.stream_plan(client, a.model, cfg, u['text'], u.get('state', 'idle'),
                                         lines.append, at=at, holding=holding,
                                         timeout_s=20.0, first_line_timeout_s=8.0)
            except Exception as e:  # noqa: BLE001
                dt, err = float('nan'), f'{type(e).__name__}: {e}'
            eff, blocked = [], []
            for ln in lines:
                if ln.kind == 'sub':
                    planner.normalize(ln, cfg)
                    ok, why = planner.feasibility(ln, cfg)
                    if why == 'skip_nearby':
                        continue
                    if not ok:
                        blocked.append(f'{ln.i}:{why}')
                eff.append(ln)
            verdict, note = ('invalid', err) if err else score(u, eff, blocked, at)
            subs = [ln for ln in eff if ln.kind == 'sub']
            llm_plan = [(ln.action, list(ln.args), ln.say) for ln in subs]
            run_plan = executed(subs, rw)
            row = dict(label=a.label, id=u['id'], cat=u.get('cat', ''), rep=r, text=u['text'],
                       verdict=verdict, note=note, llm=fmt(llm_plan), exec=fmt(run_plan),
                       open_llm=open_check(llm_plan, doors), open_exec=open_check(run_plan, doors),
                       inserted=[f'{x}({", ".join(y)}) say={s!r}' for x, y, s in run_plan
                                 if (x, y, s) not in llm_plan],
                       done_s=round(dt, 2),
                       reply=next((f'{ln.reply_type}: {ln.say}' for ln in eff if ln.kind == 'reply'), ''))
            rows.append(row)
            jl.write(json.dumps(row, ensure_ascii=False) + '\n')
            jl.flush()
            mark = {'pass': 'O', 'fail': 'X', 'invalid': '!', 'manual': '?'}[verdict]
            print(f'{mark} {u["id"]:4s} r{r} open llm={row["open_llm"] or "-":3s} exec={row["open_exec"] or "-":3s} '
                  f'{row["llm"] or row["reply"]}  {note}')

    def count(rs, k, v):
        return sum(1 for x in rs if x[k] == v)
    md = [f'# plan regression — {a.label} ({a.model}, repeat {a.repeat})', '',
          '| cat | runs | pass | fail | invalid | open missing (LLM) | open missing (exec) |',
          '|---|---|---|---|---|---|---|']
    cats = sorted({x['cat'] for x in rows}) + ['ALL']
    for c in cats:
        rs = [x for x in rows if c == 'ALL' or x['cat'] == c]
        md.append(f'| {c} | {len(rs)} | {count(rs, "verdict", "pass")} | {count(rs, "verdict", "fail")} | '
                  f'{count(rs, "verdict", "invalid")} | {count(rs, "open_llm", "NO")} | {count(rs, "open_exec", "NO")} |')
    fails = [x for x in rows if x['verdict'] != 'pass' or x['open_exec'] == 'NO']
    if fails:
        md += ['', '## not passing', '', '| id | rep | verdict | LLM plan | executed |', '|---|---|---|---|---|']
        md += [f'| {x["id"]} | {x["rep"]} | {x["verdict"]} {x["note"]} | {x["llm"] or x["reply"]} | {x["exec"]} |'
               for x in fails]
    io.open(os.path.join(out, 'summary.md'), 'w', encoding='utf-8').write('\n'.join(md) + '\n')
    print('\n'.join(md) + f'\n-> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
