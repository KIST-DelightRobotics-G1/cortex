"""Group /cortex/events into one file per voice command. No ROS here (session_recorder_node wraps it).

A session opens on the orchestrator's SESSION_START and closes post_roll_s after its
SESSION_END (trailing TTS / detector events still land in it). Events that carry the
session's plan_id go to that session only; events without one (stt, tts, detector …)
go to every open session. pre_roll_s of earlier events are copied in at the start
(STT stream state, the transcript that triggered it).

    <dir>/2026-10-15/sessions.csv                     one row per finished session of that day
    <dir>/2026-10-15/142233_p-1792012895-0001.jsonl   one event per line, as published

A session belongs to the day it started. Date folders older than keep_days are deleted
whole, index included (only folders named YYYY-MM-DD).
"""
import collections
import csv
import datetime
import json
import os
import re
import shutil
from pathlib import Path

_DAY = re.compile(r'^\d{4}-\d{2}-\d{2}$')
_SAFE = re.compile(r'[^A-Za-z0-9_.-]')
CSV_FIELDS = ['start', 'plan_id', 'utterance', 'outcome', 'detail', 'last_exec', 'last_action',
              'last_state', 'step', 'duration_s', 'file']


def _step(end: dict) -> str:
    """'2/6' — the step the session ended in (1-based) over the plan's step count."""
    try:
        steps, last = int(end.get('steps', 0)), int(end.get('last_step', -1))
    except (TypeError, ValueError):
        return ''
    return f'{last + 1}/{steps}' if steps > 0 and last >= 0 else ''


def _local(stamp_ns: int) -> datetime.datetime:
    return datetime.datetime.fromtimestamp(stamp_ns / 1e9)


class _Session:
    def __init__(self, path: Path, start_ns: int, utterance: str):
        self.path, self.start_ns, self.utterance = path, start_ns, utterance
        self.file = open(path, 'a', encoding='utf-8')
        self.count = 0
        self.end = None                 # SESSION_END fields once seen
        self.end_ns = 0

    def write(self, line: str) -> None:
        self.file.write(line + '\n')
        self.file.flush()               # a crash keeps everything up to here
        self.count += 1


class SessionLog:
    def __init__(self, directory, pre_roll_s=5.0, post_roll_s=2.0, end_timeout_s=120.0,
                 keep_days=14):
        self.dir = Path(os.path.expanduser(str(directory)))
        self.pre_roll_ns = int(pre_roll_s * 1e9)
        self.post_roll_ns = int(post_roll_s * 1e9)
        self.end_timeout_ns = int(end_timeout_s * 1e9)
        self.keep_days = int(keep_days)
        self.dir.mkdir(parents=True, exist_ok=True)
        self._recent = collections.deque()          # (stamp_ns, line) for the pre-roll
        self._open: dict[str, _Session] = {}

    # ----------------------------------------------------------------- input
    def add(self, line: str) -> None:
        """One /cortex/events message (a JSON object as text)."""
        try:
            ev = json.loads(line)
            stamp = int(ev['stamp_ns'])
        except (ValueError, KeyError, TypeError):
            return
        fields = ev.get('fields') or {}
        name, plan_id = ev.get('event', ''), fields.get('plan_id', '')
        if name == 'SESSION_START' and plan_id and plan_id not in self._open:
            self._start(plan_id, stamp, fields.get('utterance', ''))
        if plan_id in self._open:
            targets = [self._open[plan_id]]
        else:
            targets = [s for s in self._open.values() if s.end is None or stamp <= s.end_ns + self.post_roll_ns]
        for s in targets:
            s.write(line)
        if name == 'SESSION_END' and plan_id in self._open:
            s = self._open[plan_id]
            s.end, s.end_ns = fields, stamp
        self._recent.append((stamp, line))
        while self._recent and self._recent[0][0] < stamp - self.pre_roll_ns:
            self._recent.popleft()

    def tick(self, now_ns: int) -> None:
        """Close sessions whose post-roll is over, or that never got an END."""
        for plan_id, s in list(self._open.items()):
            if s.end is not None and now_ns >= s.end_ns + self.post_roll_ns:
                self._close(plan_id)
            elif s.end is None and now_ns - s.start_ns > self.end_timeout_ns:
                s.end = {'outcome': 'recorder_timeout', 'detail': 'no SESSION_END',
                         'duration_s': round((now_ns - s.start_ns) / 1e9, 2)}
                self._close(plan_id)

    def close_all(self) -> None:
        for plan_id, s in list(self._open.items()):
            if s.end is None:
                s.end = {'outcome': 'recorder_stopped', 'detail': 'shutdown before SESSION_END'}
            self._close(plan_id)

    # ------------------------------------------------------------- internals
    def _start(self, plan_id: str, stamp: int, utterance: str) -> None:
        when = _local(stamp)
        day = self.dir / when.strftime('%Y-%m-%d')
        day.mkdir(exist_ok=True)
        path = day / f'{when.strftime("%H%M%S")}_{_SAFE.sub("_", plan_id)}.jsonl'
        s = _Session(path, stamp, utterance)
        for t, line in self._recent:
            if t >= stamp - self.pre_roll_ns:
                s.write(line)
        self._open[plan_id] = s

    def _close(self, plan_id: str) -> None:
        s = self._open.pop(plan_id)
        s.file.close()
        end = s.end or {}
        index = s.path.parent / 'sessions.csv'          # the session's day folder
        new = not index.exists()
        with open(index, 'a', encoding='utf-8', newline='') as f:
            w = csv.DictWriter(f, CSV_FIELDS)
            if new:
                w.writeheader()
            w.writerow({'start': _local(s.start_ns).isoformat(timespec='seconds'),
                        'plan_id': plan_id, 'utterance': s.utterance,
                        'outcome': end.get('outcome', ''), 'detail': end.get('detail', ''),
                        'last_exec': end.get('last_exec', ''),
                        'last_action': end.get('last_action', ''),
                        'last_state': end.get('last_state', ''), 'step': _step(end),
                        'duration_s': end.get('duration_s', ''), 'file': s.path.name})

    def prune(self, today: datetime.date) -> list:
        """Delete YYYY-MM-DD folders older than keep_days. Returns the deleted names."""
        cutoff = today - datetime.timedelta(days=self.keep_days)
        gone = []
        for p in self.dir.iterdir():
            if p.is_dir() and _DAY.match(p.name):
                try:
                    old = datetime.date.fromisoformat(p.name) < cutoff
                except ValueError:
                    continue
                if old:
                    shutil.rmtree(p)
                    gone.append(p.name)
        return sorted(gone)
