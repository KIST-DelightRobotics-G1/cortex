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
import os
import re
import time
from dataclasses import dataclass, field

import yaml


_ARG_RE = re.compile(r'^[a-z0-9_]+$')


def load_config(path: str, capabilities_path: str = '') -> dict:
    """actions.yaml (프롬프트가 보는 것) + capabilities.yaml (모듈·표시용) 을 한 dict 으로.

    capabilities_path 를 비우면 actions.yaml 옆의 capabilities.yaml 을 찾는다.
    두 파일을 나눈 이유: actions.yaml 만 프롬프트에 들어간다. 장소·물건 목록과
    한국어 이름표는 검증기·GUI·TTS 쪽 자료여서 LLM 에 보이면 안 된다.
    """
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    caps_path = capabilities_path or os.path.join(os.path.dirname(path), 'capabilities.yaml')
    if os.path.exists(caps_path):
        with open(caps_path, encoding='utf-8') as f:
            cfg.update(yaml.safe_load(f) or {})
    cfg.setdefault('nav', {}).setdefault('places', [])
    cfg['nav'].setdefault('not_places', [])
    cfg.setdefault('vla', {})
    cfg.setdefault('aliases', {})
    cfg.setdefault('korean', {})
    cfg.setdefault('english', {})
    return cfg


def provider_of(model: str) -> str:
    """google | openai — decides which SDK streams this model."""
    return 'google' if model.startswith(('gemini', 'gemma')) else 'openai'


# ---------------------------------------------------------------------------
# Prompt
# ---------------------------------------------------------------------------
_PROMPT = """당신은 Unitree G1 가정용 보조 로봇의 태스크 분해기다. 사용자의 한국어 발화 하나와 로봇 상태를
받아, 로봇이 순서대로 수행할 subtask 를 **한 줄에 하나씩 JSON 으로** 출력한다.

## 출력 형식 (NDJSON)
- 한 줄 = 완결된 JSON 객체 하나. 배열·마크다운 펜스·설명·빈 줄 금지. 줄 순서가 실행 순서다.
- subtask 줄: {{"i":<0부터 순번>,"a":"<동사>","args":[<인자>],"say":"<한국어 안내 한 문장>"}}
- 마지막 줄: {{"end":true,"n":<subtask 수>}}  (반드시 출력)
- 동작이 필요 없는 발화면 subtask 줄 대신 응답 줄 하나: {{"reply":"chat"|"none"|"confirm","say":"<한국어>"}}
  그 뒤에 {{"end":true,"n":0}}.
- subtask 는 확정되는 대로 즉시 출력한다. 전체를 다 정한 뒤 한꺼번에 내지 않는다.

## 동사 (이 20개만 쓴다. 인자 개수는 서명과 정확히 같아야 한다)
{verbs}

## 인자 형식
- 영어 소문자 명사, 여러 단어는 밑줄로 (예: water_bottle, remote_control, living_room).
  한국어·대문자·공백·관사 금지.
- 사람은 "user". 사용자 위치·"나한테"·"이리" 도 "user". 로봇의 기본 위치("원위치·제자리·홈")는 "home".
- 사용자가 말하지 않은 수식어를 붙이지 않는다. ("문 닫아" → door)
- 동사 서명의 인자 이름(object, target, device 등)을 값으로 쓰지 않는다.

## 이동
- 발화에 장소나 가구가 나오고 그곳이 at 과 다르면, 그 동작 앞에 move_to 를 넣는다.
- 장소 언급이 없으면 이동하지 않고 지금 자리에서 한다.

## 로봇 상태
요청에는 로봇의 현재 상태가 함께 온다.
- at: 지금 있는 장소. unknown 이면 위치를 모른다.
- holding: 지금 들고 있는 물건. none 이면 손이 비어 있고, unknown 이면 직전 조작이 확실하지 않다.
- mode: idle(대기) · running:<동사>(실행 중) · awaiting_confirm:<요약>(되물은 뒤 답 대기).

## say 규칙
- 그 subtask 를 시작할 때 로봇이 말할 한 문장. 존댓말 "~합니다" 형. 그 동작만 말한다.
- 미래형·약속 표현 금지 ("~드릴게요", "~할게요", "~하겠습니다" 금지).

## reply 규칙
- chat: 인사·감사·잡담·로봇에 대한 질문. 두 문장 이내. 로봇 이름은 G1. 이동만 요청했는데 그곳이 at 과 같거나
  이미 그 상태면 chat 으로 알린다. at 이 unknown 이면 이렇게 판단하지 말고 이동을 넣는다.
- none: 위 20개 동사로 표현할 수 없는 요청(노래, 춤, 사진, 접기 같은 동작)에만.
- confirm: 무엇을 원하는지 특정할 수 없을 때(대상이 "그거"뿐이거나, 잘못 들은 것 같은 낱말). 짧은 확인 질문.
- 인사·잡담이 명령과 함께 있으면 명령을 계획으로 낸다.
- 한 발화에 여러 과제가 있으면 말한 순서대로 모두 넣는다.
- mode 가 running:<동사> 면 새 명령은 새 계획으로 출력(선점), 격려·잡담은 chat.
  awaiting_confirm:<요약> 이면 긍정일 때 확인했던 요청의 완전한 계획을, 부정이면 none "알겠습니다."

{examples}"""


def build_system_prompt(cfg: dict) -> str:
    """측정에 쓴 프롬프트 그대로. 동사 서명과 예시만 actions.yaml 에서 채운다.

    허용값 목록과 물체 위치표는 없다 — 어휘가 열려 있고, 할 수 없는 것은
    프롬프트가 아니라 feasibility() 가 막는다.
    """
    verbs = chr(10).join(f'  {name}({", ".join(spec["sig"])})' for name, spec in cfg['actions'].items())
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
        ex_blocks.append(f'발화: "{ex["utterance"]}"' + chr(10) + '출력:' + chr(10) + chr(10).join(out))
    examples = '## 예시' + chr(10) + chr(10).join(ex_blocks) + chr(10)
    return _PROMPT.format(verbs=verbs, examples=examples)


def build_user_message(text: str, state: str = 'idle', at: str = 'home', holding: str = 'none') -> str:
    """호출마다 붙는 사용자 메시지. at / holding 이 연속 발화를 가능하게 한다.

    at      마지막으로 도착에 성공한 장소. 시작값 home(S1), 이동 실패·중단 시 unknown
    holding 지금 들고 있는 물건. 시작값 none
    """
    return f'mode: {state}' + chr(10) + f'at: {at}' + chr(10) + f'holding: {holding}' + chr(10) + f'utterance: {text}'


# ---------------------------------------------------------------------------
# Derived per-step data (executor side)
# ---------------------------------------------------------------------------
def exec_of(cfg: dict, action: str) -> str:
    return cfg['actions'][action]['exec']


def instruction_for(cfg: dict, action: str, args: list) -> str:
    """VLA 에 넘기는 영어 지시문. 이름표에 없는 인자는 밑줄만 공백으로 바꿔 그대로 쓴다."""
    tpl = cfg.get('instruction', {}).get(action)
    if not tpl:
        return ''
    en = cfg.get('english', {})
    return tpl.format(*[en.get(a, a.replace('_', ' ')) for a in args])


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
    """표시·안내용 한국어 이름. 이름표에 없으면 영어 키를 그대로 쓴다 (어휘가 열려 있어 늘 일어난다)."""
    return str(cfg.get('korean', {}).get(key, key.replace('_', ' '))).split('/')[0]


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
    ko = cfg.get('korean', {})
    if args and args[0] in ko:      # 이름표가 있는 물건만 본다 — 열린 어휘에서는 대부분 없다
        aliases = str(ko[args[0]]).split('/')
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
    if acts[a].get('enabled') is False:            # 키가 없으면 사용 가능 (v2 는 20개 전부 사용)
        return Line('error', raw, error=f'action {a!r} not enabled')
    sig = acts[a]['sig']
    args = obj.get('args')
    if not isinstance(args, list) or len(args) != len(sig):
        return Line('error', raw, error=f'{a} expects {len(sig)} args {sig}, got {args!r}')
    # 어휘는 열려 있다 — 값이 무엇인지는 여기서 따지지 않고 feasibility() 가 판단한다.
    # 여기서 보는 것은 형식뿐: 소문자 영어·숫자·밑줄, 그리고 서명의 인자 이름을 값으로 쓰지 않을 것.
    slots = {s for spec in cfg['actions'].values() for s in spec['sig']}
    for val in args:
        if not isinstance(val, str) or not val or not _ARG_RE.match(val):
            return Line('error', raw, error=f'bad arg {val!r} (lowercase a-z0-9_ only)')
        if val in slots:
            return Line('error', raw, error=f'arg {val!r} is a signature slot name, not a value')
    i = obj.get('i')
    if i != expected_i:
        return Line('error', raw, error=f'index {i!r}, expected {expected_i}')
    say = str(obj.get('say') or '')
    return Line('sub', raw, i=i, action=a, args=list(args), say=say,
                say_warn=check_say(say, a, list(args), cfg))


# ---------------------------------------------------------------------------
# 검증기 2 — 실행 가능성
#
# 검증기 1 이 "읽을 수 있는 줄인가" 를 봤다면, 여기서는 "이 로봇이 할 수 있는 일인가" 를 본다.
# LLM 은 어휘가 열려 있어 무엇이든 계획할 수 있고, 할 수 없는 것을 막는 책임은 전적으로 이쪽에 있다.
#
# 장소 목록(nav.places)은 통합 담당자가 capabilities.yaml 에 손으로 적는 설정이다. nav 이
# 발행하는 목적지 카탈로그를 구독하지 않는 이유는 계약을 cmd/state 두 토픽으로 유지하기
# 위해서다. 따라서 이 검사는 "빠른 사전 차단"일 뿐 권한이 아니다 — 목록이 낡아 통과한 줄은
# 모듈이 FAILED 로 돌려주고, 그 경로도 반드시 사용자에게 안내되어야 한다.
# station 이 바뀌면 nav 의 destinations.yaml 과 이 파일을 같이 고쳐야 한다.
# ---------------------------------------------------------------------------
def normalize(ln: Line, cfg: dict) -> Line:
    """같은 것을 다르게 부른 이름을 모듈이 아는 키로 바꾼다 (refrigerator → fridge). ln 을 고쳐서 돌려준다."""
    al = cfg.get('aliases') or {}
    if ln.kind == 'sub':
        ln.args = [al.get(a, a) for a in ln.args]
    return ln


def feasibility(ln: Line, cfg: dict) -> tuple[bool, str]:
    """(ok, reason). normalize() 를 먼저 부를 것.

    reason
        ''                  그대로 실행
        'skip_nearby'       ok=True 지만 이 move_to 줄은 버린다 (장소가 아닌 조작 대상)
        'unknown_place'     갈 수 없는 곳 — 차단하고 안내
        'unsupported_action'  VLA 가 못 하는 동작
        'unknown_object'    다룰 수 없는 물건
    """
    if ln.kind != 'sub':
        return True, ''
    exec_ = cfg['actions'][ln.action]['exec']
    nav = cfg.get('nav') or {}
    vla = cfg.get('vla') or {}
    objs = vla.get('objects') or []
    if exec_ == 'nav':
        if ln.args[0] in (nav.get('places') or []):
            return True, ''
        if ln.args[0] in (nav.get('not_places') or []):
            # 문·서랍·창문처럼 장소가 아닌 조작 대상에 붙은 move_to 는 이 줄만 건너뛰고 계획은 잇는다.
            # 소파·책상처럼 "갈 수 없는 진짜 장소"는 여기 없으므로 아래에서 차단된다.
            return True, 'skip_nearby'
        return False, 'unknown_place'
    if vla.get('actions') and ln.action not in vla['actions']:
        return False, 'unsupported_action'
    if ln.action == 'receive':                     # 받는 물건은 사용자가 정한다 — 이름을 검사하지 않는다
        return True, ''
    if objs and ln.args and ln.args[0] not in objs:
        return False, 'unknown_object'
    if objs and ln.action in ('place', 'handover') and len(ln.args) > 1 \
            and ln.args[1] not in objs and ln.args[1] not in (nav.get('places') or []):
        return False, 'unknown_object'
    return True, ''


# ---------------------------------------------------------------------------
# Streaming backends
# ---------------------------------------------------------------------------
def make_client(model: str, api_key: str | None = None):
    if provider_of(model) == 'google':
        from google import genai
        return genai.Client(api_key=api_key) if api_key else genai.Client()
    from openai import OpenAI
    return OpenAI(api_key=api_key) if api_key else OpenAI()


def _openai_stream(client, model, system, user, timeout_s: float = 0.0):
    base = dict(model=model, stream=True,
                messages=[{'role': 'system', 'content': system},
                          {'role': 'user', 'content': user}])
    if timeout_s:
        base['timeout'] = timeout_s
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


def _google_stream(client, model, system, user, timeout_s: float = 0.0):
    import logging
    from google.genai import types
    logging.getLogger('google_genai').setLevel(logging.ERROR)   # keep the AFC notice out of the stream
    common = dict(system_instruction=system, temperature=0.0,
                  automatic_function_calling=types.AutomaticFunctionCallingConfig(disable=True))
    if timeout_s:                        # 응답이 아예 안 오는 경우를 끊는 것은 여기뿐이다
        common['http_options'] = types.HttpOptions(timeout=int(timeout_s * 1000))
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


def stream_lines(pieces, cfg: dict, on_line, should_stop=None,
                 deadline: float = 0.0, first_line_deadline: float = 0.0) -> None:
    """Split a text stream into lines, validate each, call on_line(Line).
    should_stop() → True aborts (latest-wins preemption).

    deadline / first_line_deadline 은 perf_counter 기준 절대 시각. 넘기면 TimeoutError.
    청크가 아예 안 오는 경우는 여기서 못 잡으므로 SDK 쪽 HTTP 타임아웃이 함께 있어야 한다.
    """
    buf = ''
    expected_i = 0
    got_line = False
    for piece in pieces:
        if should_stop and should_stop():
            return
        now = time.perf_counter()
        if deadline and now > deadline:
            raise TimeoutError('stream deadline passed')
        if first_line_deadline and not got_line and now > first_line_deadline:
            raise TimeoutError('no first line within the limit')
        buf += piece
        while '\n' in buf:
            raw, buf = buf.split('\n', 1)
            if not raw.strip():
                continue
            ln = validate_line(raw, cfg, expected_i)
            if ln.kind == 'sub':
                expected_i += 1
            if ln.kind != 'skip':
                got_line = True
                on_line(ln)
    if buf.strip():                                  # last line without a newline
        ln = validate_line(buf, cfg, expected_i)
        if ln.kind != 'skip':
            on_line(ln)


def stream_plan(client, model: str, cfg: dict, text: str, state: str = 'idle',
                on_line=None, should_stop=None, at: str = 'home', holding: str = 'none',
                timeout_s: float = 0.0, first_line_timeout_s: float = 0.0) -> float:
    """One utterance, streamed. Returns wall time in seconds. Raises on API failure.

    timeout_s            전체 응답 제한. SDK 의 HTTP 타임아웃으로 내려보낸다.
    first_line_timeout_s 첫 줄이 이 시간 안에 안 오면 TimeoutError. 0 이면 끄기.
    """
    system = build_system_prompt(cfg)
    user = build_user_message(text, state, at, holding)
    gen = _google_stream if provider_of(model) == 'google' else _openai_stream
    t0 = time.perf_counter()
    stream_lines(gen(client, model, system, user, timeout_s), cfg,
                 on_line or (lambda ln: None), should_stop,
                 deadline=(t0 + timeout_s) if timeout_s else 0.0,
                 first_line_deadline=(t0 + first_line_timeout_s) if first_line_timeout_s else 0.0)
    return time.perf_counter() - t0
