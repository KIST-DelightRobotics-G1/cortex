"""planner — utterance → NDJSON subtask stream. Pure logic, no rclpy (testable).

    load_config(path)                         actions.yaml
    build_system_prompt(cfg)                  prompt generated from the yaml
    validate_line(raw, cfg, expected_i)       one line → Line (sub | end | reply | error | skip)
    stream_plan(client, model, cfg, text, state, on_line, should_stop)
                                              streaming call; on_line(Line) per closed line
    instruction_for(cfg, action, args)        VLA English instruction from the template
    precheck_target(cfg, action, args)        vocabulary key the detector must see, or None

Line schema (design 2.1):
    {"i":0,"a":"move_to","args":["fridge"],"say":"냉장고로 갑니다."}
    {"end":true,"n":6}
    {"reply":"none","say":"..."}

Ported from llm-router-test/plan/planner.py (3rd-round prompt, v2 rules).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field

import yaml


def load_config(path: str) -> dict:
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def provider_of(model: str) -> str:
    """google | openai — decides which SDK streams this model."""
    return 'google' if model.startswith(('gemini', 'gemma')) else 'openai'


# ---------------------------------------------------------------------------
# Prompt
# ---------------------------------------------------------------------------
def build_system_prompt(cfg: dict) -> str:
    acts = {k: v for k, v in cfg['actions'].items() if v.get('enabled')}
    ko = cfg['korean']
    sig_lines = [f'  {name}({", ".join(spec["sig"])})' for name, spec in acts.items()]
    vocab_lines = []
    for typ, vals in cfg['vocab'].items():
        vocab_lines.append(f'  {typ}: ' + ', '.join(f'"{v}"({ko.get(v, v)})' for v in vals))
    ex_blocks = []
    for ex in cfg.get('examples', []):
        out = []
        if 'lines' in ex:
            for ln in ex['lines']:
                out.append(json.dumps({'i': ln['i'], 'a': ln['a'], 'args': ln['args'], 'say': ln['say']},
                                      ensure_ascii=False))
            out.append(json.dumps({'end': True, 'n': len(ex['lines'])}))
        else:
            out.append(json.dumps(ex['reply'], ensure_ascii=False))
            out.append(json.dumps({'end': True, 'n': 0}))
        ex_blocks.append(f'발화: "{ex["utterance"]}"\n출력:\n' + '\n'.join(out))
    examples = '\n\n'.join(ex_blocks)
    max_n = cfg.get('limits', {}).get('max_subtasks', 8)
    loc_lines = []
    for obj, spec in cfg.get('object_locations', {}).items():
        line = f'  {obj}({ko.get(obj, obj)}) 는 {spec["at"]}({ko.get(spec["at"], spec["at"])}) 에 있다'
        if spec.get('inside'):
            line += f'. 꺼내거나 넣으려면 먼저 open({spec["inside"]}), 끝나면 close({spec["inside"]})'
        loc_lines.append(line)
    locations = '\n'.join(loc_lines)

    return f"""당신은 Unitree G1 가정용 보조 로봇의 태스크 분해기다. 사용자의 한국어 발화 하나와 로봇 상태를
받아, 로봇이 순서대로 수행할 subtask 를 **한 줄에 하나씩 JSON 으로** 출력한다.

## 출력 형식 (NDJSON)
- 한 줄 = 완결된 JSON 객체 하나. 배열·마크다운 펜스·설명·빈 줄 금지. 줄 순서가 실행 순서다.
- subtask 줄: {{"i":<0부터 순번>,"a":"<action>","args":[<위치 인자>],"say":"<한국어 안내 한 문장>"}}
- 마지막 줄: {{"end":true,"n":<subtask 수>}}  (반드시 출력)
- 동작이 필요 없는 발화면 subtask 줄 대신 응답 줄 하나: {{"reply":"chat"|"none"|"confirm","say":"<한국어>"}}
  그 뒤에 {{"end":true,"n":0}}.
- subtask 는 확정되는 대로 즉시 출력한다. 전체를 다 정한 뒤 한꺼번에 내지 않는다.

## 사용할 수 있는 action (이 목록 밖의 action 은 절대 쓰지 않는다)
{chr(10).join(sig_lines)}

## 인자 허용값 (이 값 밖의 물건·장소는 절대 쓰지 않는다. 비슷한 것으로 바꾸지도 않는다)
{chr(10).join(vocab_lines)}

## 물체 위치 (조작 전에 이 장소로 move_to 한다. 로봇이 이미 그 자리라는 정보는 없다)
{locations}

## say 규칙
- 그 subtask 를 시작할 때 로봇이 말할 한 문장. 존댓말 "~합니다" 형. 그 동작만 말한다.
- 미래형·약속 표현 금지 ("~드릴게요", "~할게요", "~하겠습니다" 금지).

## 순서 규칙
- 조작(open/close/pick/place/handover) 전에 그 대상이 있는 장소로 move_to 가 먼저 와야 한다 (위 물체 위치 표 기준. 항상 넣는다).
- 물건을 어디에 놓거나 건네려면 먼저 그 물건을 pick 해야 한다 (손에 들고 있다는 정보는 없다).
- open 한 것은 작업 뒤 close 한다.
- "가져와/가져다줘/줘" 처럼 목적지가 없는 요청은 사용자에게 가져다주는 것이다: 마지막에 move_to(user), handover(물건, user). 목적지를 되묻지 않는다.
- 인사·잡담이 명령과 함께 있으면 명령을 계획으로 낸다 (reply 로 끝내지 않는다). 첫 subtask 의 say 앞에 인사를 붙여도 된다.
- awaiting_confirm 에 긍정 응답이면 확인했던 요청의 **완전한** 계획을 낸다 (끝까지: 전달 요청이면 handover 까지).
- subtask 수는 {max_n} 이하.

## 허용값 밖의 요청
- 인자 허용값에 없는 물건·장소가 하나라도 필요하면 subtask 를 한 줄도 내지 말고 reply none 으로 안내한다.
  비슷한 값으로 바꾸지 않는다 (예: "트레이에 놓아" → tray 는 놓는 장소(target)가 아니므로 none. "싱크대" → none).
- 잘못 들은 것 같은 낱말(예: "오리")은 none 이 아니라 confirm 으로 되묻는다: "오이 말씀이신가요?"

## reply 규칙
- chat: 인사·감사·잡담·로봇에 대한 질문. 두 문장 이내. 로봇 이름은 G1. 모르는 사실은 모른다고 답한다.
- none: 허용값 밖의 물건·장소이거나 위 action 으로 할 수 없는 요청. "…는 아직 못 합니다." 로 안내.
- confirm: 물건·장소가 발화에 없어 짐작해야 하거나 STT 오인식이 의심될 때. 짧은 확인 질문.
- 말투는 판단 근거가 아니다. "~해 줘", "~부탁할게", "~주시겠어요?" 같은 완곡·존댓말 부탁도 물건과 동작이 정해지면 subtask 로 분해한다.

## 상태(state)
- idle: 위 규칙 그대로.
- running:<action>: 그 subtask 실행 중. 새 명령이면 새 계획을 출력(선점). 격려·잡담은 reply chat.
- awaiting_confirm:<요약>: 직전 확인 질문에 대한 답. 긍정이면 계획 출력, 부정이면 reply none "알겠습니다."

## 예시
{examples}
"""


def build_user_message(text: str, state: str = 'idle') -> str:
    return f'state: {state}\nutterance: {text}'


# ---------------------------------------------------------------------------
# Derived per-step data (executor side)
# ---------------------------------------------------------------------------
def exec_of(cfg: dict, action: str) -> str:
    return cfg['actions'][action]['exec']


def instruction_for(cfg: dict, action: str, args: list) -> str:
    tpl = cfg.get('instruction', {}).get(action)
    if not tpl:
        return ''
    en = cfg.get('english', {})
    return tpl.format(*[en.get(a, a) for a in args])


def precheck_target(cfg: dict, action: str, args: list) -> str | None:
    slot = cfg.get('precheck', {}).get(action)
    if slot is None:
        return None
    sig = cfg['actions'][action]['sig']
    try:
        return args[sig.index(slot)]
    except (ValueError, IndexError):
        return None


def ko_name(cfg: dict, key: str) -> str:
    return str(cfg.get('korean', {}).get(key, key)).split('/')[0]


def step_title(cfg: dict, action: str, args: list) -> str:
    """Korean one-liner for the display: '냉장고 문 열기', '냉장고로 이동'."""
    ako = cfg.get('action_ko', {}).get(action, action)
    if not args:
        return ako
    obj = ko_name(cfg, args[0])
    if action == 'move_to':
        return f'{obj}로 이동' if obj != '사용자' else '사용자에게 이동'
    if action == 'place' and len(args) > 1:
        return f'{obj} {ko_name(cfg, args[1])}에 {ako}'
    return f'{obj} {ako}'


def _has_batchim(word: str) -> bool:
    ch = word.rstrip()[-1:] if word else ''
    return bool(ch) and 0xAC00 <= ord(ch) <= 0xD7A3 and (ord(ch) - 0xAC00) % 28 != 0


def phrase(cfg: dict, key: str, target_key: str = '') -> str:
    """Fixed phrase with Korean particles resolved: {ko} name, {i} 이/가, {eul} 을/를, {eun} 은/는."""
    tpl = cfg.get('phrases', {}).get(key, '')
    if not tpl:
        return ''
    ko = ko_name(cfg, target_key)
    b = _has_batchim(ko)
    return tpl.format(ko=ko, i='이' if b else '가', eul='을' if b else '를', eun='은' if b else '는')


# ---------------------------------------------------------------------------
# Line validation
# ---------------------------------------------------------------------------
def check_say(say: str, action: str, args: list, cfg: dict) -> list[str]:
    """say quality check. Returns violations (empty = pass)."""
    w = []
    s = say.strip()
    if not s:
        return ['empty']
    if any(p in s for p in cfg.get('promise_patterns', [])):
        w.append('promise')
    if sum(s.count(c) for c in '.!?') > 1 or len(s) > 40:
        w.append('not-one-sentence')
    if not s.endswith(('다.', '다', '요.', '요')):
        w.append('not-polite-ending')
    stems = cfg.get('say_stems', {}).get(action, [])
    if stems and not any(st in s for st in stems):
        w.append('action-mismatch')
    ko = cfg['korean']
    if args:
        aliases = str(ko.get(args[0], args[0])).split('/')
        if not any(al in s for al in aliases):
            w.append('object-missing')
    return w


@dataclass
class Line:
    kind: str                   # sub | end | reply | error | skip
    raw: str
    i: int = -1
    action: str = ''
    args: list = field(default_factory=list)
    say: str = ''
    n: int = -1
    reply_type: str = ''
    error: str = ''
    say_warn: list = field(default_factory=list)


def validate_line(raw: str, cfg: dict, expected_i: int) -> Line:
    s = raw.strip()
    if s.startswith('```'):                       # stray fence — ignore
        return Line('skip', raw)
    try:
        obj = json.loads(s)
    except json.JSONDecodeError as e:
        return Line('error', raw, error=f'not json: {e.msg}')
    if not isinstance(obj, dict):
        return Line('error', raw, error='not an object')

    if obj.get('end') is True:
        n = obj.get('n')
        if not isinstance(n, int):
            return Line('error', raw, error='end without int n')
        return Line('end', raw, n=n)

    if 'reply' in obj:
        rt = obj.get('reply')
        if rt not in ('chat', 'none', 'confirm'):
            return Line('error', raw, error=f'bad reply type {rt!r}')
        say = str(obj.get('say') or '')
        return Line('reply', raw, reply_type=rt, say=say,
                    say_warn=[p for p in cfg.get('promise_patterns', []) if p in say])

    a = obj.get('a')
    acts = cfg['actions']
    if a not in acts:
        return Line('error', raw, error=f'unknown action {a!r}')
    if not acts[a].get('enabled'):
        return Line('error', raw, error=f'action {a!r} not enabled')
    sig = acts[a]['sig']
    args = obj.get('args')
    if not isinstance(args, list) or len(args) != len(sig):
        return Line('error', raw, error=f'{a} expects {len(sig)} args {sig}, got {args!r}')
    for typ, val in zip(sig, args):
        if val not in cfg['vocab'].get(typ, []):
            return Line('error', raw, error=f'arg {val!r} not in vocab[{typ}]')
    i = obj.get('i')
    if i != expected_i:
        return Line('error', raw, error=f'index {i!r}, expected {expected_i}')
    max_n = cfg.get('limits', {}).get('max_subtasks', 8)
    if i >= max_n:                                # "N 이하" enforced in code, not only in the prompt
        return Line('error', raw, error=f'subtask count exceeds limit {max_n}')
    say = str(obj.get('say') or '')
    return Line('sub', raw, i=i, action=a, args=list(args), say=say,
                say_warn=check_say(say, a, list(args), cfg))


# ---------------------------------------------------------------------------
# Streaming backends
# ---------------------------------------------------------------------------
def make_client(model: str, api_key: str | None = None):
    if provider_of(model) == 'google':
        from google import genai
        return genai.Client(api_key=api_key) if api_key else genai.Client()
    from openai import OpenAI
    return OpenAI(api_key=api_key) if api_key else OpenAI()


def _openai_stream(client, model, system, user):
    base = dict(model=model, stream=True,
                messages=[{'role': 'system', 'content': system},
                          {'role': 'user', 'content': user}])
    if model.startswith(('gpt-5', 'gpt-6', 'o1', 'o3', 'o4')):
        effort = 'none' if model.startswith(('gpt-5.6', 'gpt-6')) else 'minimal'
        attempts = [dict(base, reasoning_effort=effort), dict(base, reasoning_effort='low'), base]
    else:
        attempts = [dict(base, temperature=0.0), base]
    last = None
    for kw in attempts:
        try:
            stream = client.chat.completions.create(**kw)
            for chunk in stream:
                if chunk.choices and chunk.choices[0].delta and chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content
            return
        except Exception as e:
            last = e
            m = str(e).lower()
            if any(k in m for k in ('unsupported', 'not supported', 'invalid', 'unknown parameter')):
                continue
            raise
    raise last


def _google_stream(client, model, system, user):
    import logging
    from google.genai import types
    logging.getLogger('google_genai').setLevel(logging.ERROR)   # keep the AFC notice out of the stream
    common = dict(system_instruction=system, temperature=0.0,
                  automatic_function_calling=types.AutomaticFunctionCallingConfig(disable=True))
    attempts = [dict(common, thinking_config=types.ThinkingConfig(thinking_budget=0)), common]
    last = None
    for cfg in attempts:
        try:
            for chunk in client.models.generate_content_stream(
                    model=model, contents=user, config=types.GenerateContentConfig(**cfg)):
                if chunk.text:
                    yield chunk.text
            return
        except Exception as e:
            last = e
            if 'thinking' in str(e).lower() or 'invalid' in str(e).lower():
                continue
            raise
    raise last


def stream_lines(pieces, cfg: dict, on_line, should_stop=None) -> None:
    """Split a text stream into lines, validate each, call on_line(Line).
    should_stop() → True aborts (latest-wins preemption)."""
    buf = ''
    expected_i = 0
    for piece in pieces:
        if should_stop and should_stop():
            return
        buf += piece
        while '\n' in buf:
            raw, buf = buf.split('\n', 1)
            if not raw.strip():
                continue
            ln = validate_line(raw, cfg, expected_i)
            if ln.kind == 'sub':
                expected_i += 1
            if ln.kind != 'skip':
                on_line(ln)
    if buf.strip():                                  # last line without a newline
        ln = validate_line(buf, cfg, expected_i)
        if ln.kind != 'skip':
            on_line(ln)


def stream_plan(client, model: str, cfg: dict, text: str, state: str = 'idle',
                on_line=None, should_stop=None) -> float:
    """One utterance, streamed. Returns wall time in seconds. Raises on API failure."""
    system = build_system_prompt(cfg)
    user = build_user_message(text, state)
    gen = _google_stream if provider_of(model) == 'google' else _openai_stream
    t0 = time.perf_counter()
    stream_lines(gen(client, model, system, user), cfg, on_line or (lambda ln: None), should_stop)
    return time.perf_counter() - t0
