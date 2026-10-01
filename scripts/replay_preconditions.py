#!/usr/bin/env python3
"""Replay recorded YOLO detections through the real presence policy and executor.

No ROS, network, fresh inference or robot motion. VLA completion is explicitly
mocked. Reused KIST test footage: exploratory policy comparison, not threshold
selection or evidence of task success. Run verify_detector_offline.py first.
"""
import argparse
from bisect import bisect_right
from dataclasses import asdict
import hashlib
import heapq
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/cortex_perception'), str(ROOT/'src/cortex_cognition')]
from cortex_perception.detection import PresenceWindow, DetectionRecord, target_mapping
from cortex_cognition import planner, executor as ex
from cortex_cognition.rewrite import Rewriter


def make_window(p, names, confidence):
    return PresenceWindow(target_mapping(p['target_keys'], p['target_classes']), names,
                          p['window_s'], confidence, p['min_frames'], p.get('require_latest_hit', False))


def add_record(w, r):
    w.add(tuple(DetectionRecord(**b) for b in r['detections']), r['source_s'],
          1_700_000_000_000_000_000 + round(r['source_s']*1e9), r['query_s'])


def compare(video, p, names, conf, timing, labels):
    w = make_window(p, names, conf)
    rows = []
    for r in video['records']:
        add_record(w, r)
        v = w.check('cucumber', r['query_s'])
        rows.append({'t':r['query_s'], 'source_frame':r['source_frame'], **asdict(v)})
    times = [r['pts_ms']/1000 for r in timing['frames']]
    def truth(t):
        i = max(0, bisect_right(times, t)-1)
        if i in labels['ambiguous_frames']: return None
        if any(a <= i <= b for a,b in labels['absence_intervals']): return False
        if any(e['frame'] <= i <= e['last_frame'] for e in labels['appearance_events']): return True
        return None
    counts = dict(tp=0, fn=0, fp=0, tn=0, unavailable=0, ambiguous=0)
    for r in rows:
        gt = truth(r['t'])
        if gt is None: counts['ambiguous'] += 1; continue
        if r['detail']: counts['unavailable'] += 1
        # An unavailable verdict cannot authorize; included as a negative output.
        counts['tp' if gt and r['found'] else 'fn' if gt else 'fp' if r['found'] else 'tn'] += 1
    onsets=[];offsets=[]
    for e in labels['appearance_events']:
        t=times[e['frame']]; end=times[e['last_frame']]
        r=next((r for r in rows if t <= r['t'] <= end and r['found']),None)
        onsets.append({'frame':e['frame'],'visible_at_s':t,'first_allow_s':None if r is None else r['t'],
                       'delay_s':None if r is None else r['t']-t})
    for e in labels['disappearance_events']:
        t=times[e['frame']]; end=times[e['last_frame']]
        r=next((r for r in rows if t <= r['t'] <= end and not r['found']),None)
        offsets.append({'frame':e['frame'],'absent_at_s':t,'first_block_s':None if r is None else r['t'],
                        'delay_s':None if r is None else r['t']-t})
    return {'confidence':conf,'require_latest_hit':p.get('require_latest_hit',False),'scope':'Query-time frames; sampled ~8 Hz, source PTS ground truth. Unavailable counts as no authorization; ambiguous excluded.',
            'counts':counts,'recall':counts['tp']/(counts['tp']+counts['fn']),
            'precision':counts['tp']/(counts['tp']+counts['fp']),
            'appearance':onsets,'disappearance':offsets,'records':rows}


def scenario(video, p, names, config, params, rewriter, name, done_times, stop_at=None, idle_delay=.3):
    now=0.;q=[];seq=0;events=[];commands=[];checks=[];statuses=[]
    w=make_window(p,names,p['default_min_confidence'])
    def schedule(t,kind,data=None):
        nonlocal seq
        seq+=1;heapq.heappush(q,(t,seq,kind,data))
    def send(module,st,pid):
        assert not any(c['index']==st.index for c in commands),'unexpected duplicate dispatch'
        commands.append({'t':now,'module':module,'index':st.index,'action':st.action,'args':st.args})
    def check(target):
        v=w.check(target,now);checks.append({'t':now,'target':target,**asdict(v)});return v.found,v.detail
    ports=ex.Ports(now=lambda:now,send_cmd=send,send_cancel=lambda *a:events.append({'t':now,'cancel':a}),
                   check_target=check,say=lambda s:None,stop_speech=lambda:None,
                   trace=lambda k,pid,i,title,detail:events.append({'t':now,'kind':k,'index':i,'title':title,'detail':detail}),
                   status=lambda s,task,title,i,n,detail:statuses.append({'t':now,'state':s,'index':i,'detail':detail}))
    x=ex.Executor(config,ports,ex.Params(detector_fail_open=False,
                   precheck_timeout_s=params['precheck_timeout_s'],precheck_retry_s=params['precheck_retry_s']),
                   rewriter=rewriter)
    for r in video['records']:schedule(r['query_s'],'detection',r)
    schedule(0.,'plan')
    for i,t in enumerate(done_times):schedule(t,'mock_done',i)
    if stop_at is not None:schedule(stop_at,'stop')
    end=video['records'][-1]['query_s']+1.
    for k in range(int(end*10)+1):schedule(round(k*.1,8),'tick')
    while q:
        now,_,kind,data=heapq.heappop(q)
        if kind=='detection':add_record(w,data)
        elif kind=='plan':
            x.heard(name,'냉장고 문을 열고 오이를 집은 뒤 문을 닫아줘')
            for i,(a,arg) in enumerate([('open','fridge_door'),('pick','cucumber'),('close','fridge_door')]):
                ln=planner.normalize(planner.validate_line(json.dumps({'i':i,'a':a,'args':[arg],'say':a}),config,i),config)
                assert planner.feasibility(ln,config)==(True,'')
                x.on_step(name,0,i,ln.action,ln.args,ln.say,'',ln.raw)
            x.on_step(name,1,3,'',[],'','','')
        elif kind=='mock_done':
            if x.phase=='RUNNING' and x.cur==data and x._st in ('ACCEPT','RUN'):
                events.append({'t':now,'mock_done':data})
                x.on_state('vla',ex.DONE,name,data,'MOCK completion',1.)
                if idle_delay is not None:schedule(round(now+idle_delay,8),'mock_idle',data)
        elif kind=='mock_idle':
            if x.phase=='RUNNING' and x.cur==data and x._st=='WAIT_IDLE':
                events.append({'t':now,'mock_idle':data})
                x.on_state('vla',ex.IDLE,'',0,'MOCK cleanup complete',0.)
        elif kind=='stop':x.stop()
        elif kind=='tick':
            if x.phase=='RUNNING' and x._st in ('ACCEPT','RUN'):
                x.on_state('vla',ex.RUNNING,name,x.cur,'MOCK heartbeat',.1)
            x.tick()
    return {'name':name,'commands':commands,'checks':checks,'events':events,'statuses':statuses,
            'success':any(r['state']==ex.S_SUCCEEDED for r in statuses),'final_phase':x.phase}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--evidence',required=True)
    ap.add_argument('--near-timing',required=True,help='Dense near-video predictions containing source PTS')
    ap.add_argument('--visibility',required=True,help='Previously reviewed temporal visibility annotations')
    ap.add_argument('--output',required=True)
    ap.add_argument('--profile',required=True)
    args=ap.parse_args()
    evidence=json.loads(Path(args.evidence).read_text());timing=json.loads(Path(args.near_timing).read_text())
    ann=json.loads(Path(args.visibility).read_text())
    import yaml
    profile=yaml.safe_load(Path(args.profile).read_text())
    p=profile['detector_node']['ros__parameters'];ep=profile['orchestrator_node']['ros__parameters']
    original=evidence['profile']['detector_node']['ros__parameters']
    assert all(p[k]==original[k] for k in ('rate_hz','imgsz','infer_conf','model_sha256'))
    assert evidence['weight_sha256']==p['model_sha256']==timing['weight_sha256']
    assert ep['detector_fail_open'] is False
    names=evidence['classes'].values();video=next(v for v in evidence['videos'] if v['source']==timing['source'])
    cfg=planner.load_config(str(ROOT/'src/cortex_cognition/config/actions.yaml'))
    comparisons=[compare(video,{**p,'require_latest_hit':latest},names,c,timing,ann['videos']['v3']['cucumber'])
                 for latest in (False,True) for c in (.25,.4)]
    rewrites_path=ROOT/'src/cortex_cognition/config/plan_rewrites.yaml'
    rw=Rewriter.load(str(rewrites_path),cfg)
    scenarios=[scenario(video,p,names,cfg,ep,rw,'object_visible_after_short_wait',[1.,10.3,23.,23.5,25.2]),
               scenario(video,p,names,cfg,ep,rw,'cucumber_absent_timeout',[1.,5.,23.,23.5,25.2]),
               scenario(video,p,names,cfg,ep,rw,'without_completion_signal',[]),
               scenario(video,p,names,cfg,ep,rw,'stop_during_precheck',[],stop_at=.1),
               scenario(video,p,names,cfg,ep,rw,'done_without_idle',[1.],idle_delay=None)]
    assert [r['action'] for r in scenarios[0]['commands']]==['open','approach','pick','step_back','close'] and scenarios[0]['success']
    assert [r['action'] for r in scenarios[1]['commands']]==['open','approach'] and not scenarios[1]['success']
    assert [r['action'] for r in scenarios[2]['commands']]==['open'] and not scenarios[2]['success']
    assert not scenarios[3]['commands'] and scenarios[3]['final_phase']=='IDLE'
    assert [r['action'] for r in scenarios[4]['commands']]==['open'] and not scenarios[4]['success']
    result={'scope':'Exploratory reused test; saved real YOLO inference + real Cortex logic. No ROS/robot. VLA completion, cleanup IDLE and heartbeat mocked. Shipped demo rewrites enabled. Manual scenario timings.',
            'weight_sha256':evidence['weight_sha256'],'profile':profile,
            'rewrite_rules':rw.names,'mock_idle_delay_s':.3,
            'source_hashes':{str(Path(p)):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in [args.evidence,args.near_timing,args.visibility,rewrites_path,ROOT/'src/cortex_cognition/cortex_cognition/executor.py']},
            'near_query_comparison':comparisons,'scenarios':scenarios,'assertions_passed':True}
    out=Path(args.output)
    if out.exists():raise FileExistsError(out)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({'comparison':[{k:v for k,v in c.items() if k!='records'} for c in comparisons],
                      'scenarios':[{k:v for k,v in s.items() if k in ('name','commands','success')} for s in scenarios]},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
