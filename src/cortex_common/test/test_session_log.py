"""SessionLog without ROS: feed /cortex/events lines with explicit stamps."""
import csv
import datetime
import json

from cortex_common.session_log import SessionLog

S = 1_000_000_000
T0 = int(datetime.datetime(2026, 10, 15, 14, 22, 33).timestamp()) * S


def ev(t, node, event, **fields):
    return json.dumps({'stamp_ns': T0 + int(t * S), 'node': node, 'topic': 't',
                       'event': event, 'fields': fields}, ensure_ascii=False)


def lines(path):
    return [json.loads(l)['event'] for l in path.read_text(encoding='utf-8').splitlines()]


def rows(log):
    with open(log.dir / 'sessions.csv', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def test_one_file_per_session_with_pre_and_post_roll(tmp_path):
    log = SessionLog(tmp_path, pre_roll_s=5, post_roll_s=2)
    log.add(ev(-10, 'stt_node', 'STT_SESSION'))                      # too old for the pre-roll
    log.add(ev(-1, 'stt_node', 'STT_TRANSCRIPT', text='오이 가져와'))
    log.add(ev(0, 'orchestrator_node', 'SESSION_START', plan_id='p-1', utterance='오이 가져와'))
    log.add(ev(1, 'detector_node', 'YOLO_CHECK', target='cucumber'))
    log.add(ev(3, 'orchestrator_node', 'SESSION_END', plan_id='p-1', outcome='succeeded',
               detail='', duration_s=3.0, steps=6))
    log.add(ev(4, 'tts_node', 'TTS_SAY', text='모두 마쳤습니다.'))     # within the post-roll
    log.tick(T0 + 4 * S)
    assert not (log.dir / 'sessions.csv').exists()                   # still in post-roll
    log.add(ev(6, 'tts_node', 'TTS_SAY', text='later'))               # after the post-roll
    log.tick(T0 + 6 * S)
    files = list((tmp_path / '2026-10-15').iterdir())
    assert [f.name for f in files] == ['142233_p-1.jsonl']
    assert lines(files[0]) == ['STT_TRANSCRIPT', 'SESSION_START', 'YOLO_CHECK', 'SESSION_END', 'TTS_SAY']
    (row,) = rows(log)
    assert row['plan_id'] == 'p-1' and row['utterance'] == '오이 가져와'
    assert row['outcome'] == 'succeeded' and row['steps'] == '6' and row['events'] == '5'
    assert row['file'].replace('\\', '/') == '2026-10-15/142233_p-1.jsonl'


def test_overlapping_sessions_keep_their_own_plan_events(tmp_path):
    log = SessionLog(tmp_path, pre_roll_s=0, post_roll_s=0)
    log.add(ev(0, 'orchestrator_node', 'SESSION_START', plan_id='p-1', utterance='a'))
    log.add(ev(1, 'orchestrator_node', 'SESSION_START', plan_id='p-2', utterance='b'))
    log.add(ev(2, 'orchestrator_node', 'PLAN_STEP', plan_id='p-1'))
    log.add(ev(2, 'tts_node', 'TTS_SAY', text='shared'))
    log.add(ev(3, 'orchestrator_node', 'SESSION_END', plan_id='p-2', outcome='reply'))
    log.tick(T0 + 4 * S)
    log.add(ev(9, 'orchestrator_node', 'SESSION_END', plan_id='p-1', outcome='preempted'))
    log.tick(T0 + 10 * S)
    day = tmp_path / '2026-10-15'
    assert lines(day / '142233_p-1.jsonl') == ['SESSION_START', 'PLAN_STEP', 'TTS_SAY', 'SESSION_END']
    assert lines(day / '142234_p-2.jsonl') == ['SESSION_START', 'TTS_SAY', 'SESSION_END']
    assert [(r['plan_id'], r['outcome']) for r in rows(log)] == [('p-2', 'reply'), ('p-1', 'preempted')]


def test_missing_end_times_out_and_shutdown_closes(tmp_path):
    log = SessionLog(tmp_path, end_timeout_s=60)
    log.add(ev(0, 'orchestrator_node', 'SESSION_START', plan_id='p-1', utterance='a'))
    log.add(ev(1, 'orchestrator_node', 'SESSION_START', plan_id='p-2', utterance='b'))
    log.tick(T0 + int(60.5 * S))
    log.close_all()
    assert [(r['plan_id'], r['outcome']) for r in rows(log)] == \
        [('p-1', 'recorder_timeout'), ('p-2', 'recorder_stopped')]


def test_garbage_lines_are_ignored(tmp_path):
    log = SessionLog(tmp_path)
    log.add('not json')
    log.add('{"event": "SESSION_START"}')
    assert not log._open and not (tmp_path / 'sessions.csv').exists()


def test_prune_removes_only_old_date_folders(tmp_path):
    log = SessionLog(tmp_path, keep_days=14)
    for name in ('2026-09-30', '2026-10-01', '2026-10-02', 'notes', '2026-13-99'):
        (tmp_path / name).mkdir()
    (tmp_path / 'sessions.csv').write_text('x')
    assert log.prune(datetime.date(2026, 10, 15)) == ['2026-09-30']
    assert sorted(p.name for p in tmp_path.iterdir()) == \
        ['2026-10-01', '2026-10-02', '2026-13-99', 'notes', 'sessions.csv']
