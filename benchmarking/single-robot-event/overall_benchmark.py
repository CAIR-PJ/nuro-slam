#!/usr/bin/env python3
"""Guarded, dependency-aware six-pipeline benchmark for the existing nuro-bench.

No existing project file is modified. Run only with `run --execute`.
"""
import argparse
import csv
import hashlib
import importlib.machinery
import importlib.util
import json
import math
import os
import re
import shlex
import shutil
import statistics
import sys
from datetime import datetime, timezone
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PIPELINES = ('cmax','eroam','cmax_emba','eroam_emba','cmax_epba','eroam_epba')
CG = re.compile(r'\[EPBA_CG\]\s+iterations=(\d+)\s+estimated_error=([^\s]+)\s+relative_residual=([^\s]+)\s+converged=(\w+)')
PREVIEW = re.compile(r'EPBA preview:\s+initial=([^\s]+)\s+final=([^\s]+)\s+accepted=(\d+)\s+termination=(\S+)')
INLIERS = re.compile(r'Number of total events:\s*(\d+)')


def cli():
    loader=importlib.machinery.SourceFileLoader('nuro_bench_original',str(ROOT/'nuro-bench'))
    spec=importlib.util.spec_from_loader(loader.name,loader)
    module=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=module
    loader.exec_module(module)
    return module


def sha256(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def write_json(path, data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,indent=2,allow_nan=False,default=str)+'\n')
    temp.replace(path)


def trajectory(path):
    p=Path(path)
    if not p.is_file():return {'valid':False,'reason':'missing'}
    poses=[]
    try:
        for line in p.read_text().splitlines():
            if not line.strip() or line.lstrip().startswith('#'):continue
            v=[float(x) for x in line.split()]
            if len(v)!=8 or not all(math.isfinite(x) for x in v):raise ValueError('invalid TUM row')
            if poses and v[0]<=poses[-1][0]:raise ValueError('nonmonotonic timestamps')
            if abs(math.sqrt(sum(q*q for q in v[4:]))-1)>0.01:raise ValueError('quaternion not normalized')
            poses.append(v)
        if len(poses)<2:raise ValueError('fewer than two poses')
        jumps=[]
        for a,b in zip(poses,poses[1:]):
            dot=min(1.,max(0.,abs(sum(x*y for x,y in zip(a[4:],b[4:])))))
            jumps.append(2*math.degrees(math.acos(dot)))
        return {'valid':True,'poses':len(poses),'start_s':poses[0][0],
                'end_s':poses[-1][0],'duration_s':poses[-1][0]-poses[0][0],
                'max_rotation_step_deg':max(jumps),'steps_over_30_deg':sum(x>30 for x in jumps),
                'steps_over_90_deg':sum(x>90 for x in jumps),'net_translation':math.dist(poses[0][1:4],poses[-1][1:4])}
    except Exception as e:return {'valid':False,'reason':str(e)}


def gt_metrics(est_path,gt_path,tolerance=0.025):
    """ATE translation, rigid SE(3) alignment, nearest-time associations; RPE translation at adjacent associated poses."""
    try:import numpy as np
    except ImportError:return {'available':False,'reason':'numpy missing'}
    def read(p):
        rows=[]
        for line in Path(p).read_text().splitlines():
            if line.strip() and not line.lstrip().startswith('#'):
                a=[float(v) for v in line.split()]
                if len(a)==8 and all(math.isfinite(x) for x in a):rows.append(a)
        return np.asarray(rows,dtype=float)
    try:
        e,g=read(est_path),read(gt_path)
        if len(e)<3 or len(g)<3:return {'available':False,'reason':'insufficient poses'}
        i=np.searchsorted(g[:,0],e[:,0]);pairs=[]
        for k,t in enumerate(e[:,0]):
            opts=[j for j in (i[k]-1,i[k]) if 0<=j<len(g)]
            if opts:
                j=min(opts,key=lambda j:abs(g[j,0]-t))
                if abs(g[j,0]-t)<=tolerance:pairs.append((k,j))
        # A GT pose may only be associated once.
        pairs=sorted({j:(k,j) for k,j in pairs}.values())
        if len(pairs)<3:return {'available':False,'reason':'fewer than 3 timestamp matches','matched':len(pairs)}
        a=e[[k for k,j in pairs],1:4];b=g[[j for k,j in pairs],1:4]
        am=a.mean(0);bm=b.mean(0)
        u,s,vt=np.linalg.svd((a-am).T@(b-bm))
        d=np.eye(3);d[-1,-1]=np.linalg.det(u@vt)
        r=u@d@vt
        aligned=(a-am)@r+bm
        err=np.linalg.norm(aligned-b,axis=1)
        rel=np.linalg.norm(np.diff(aligned,axis=0)-np.diff(b,axis=0),axis=1)
        return {'available':True,'matched':len(pairs),'alignment':'SE3 rigid, no scale',
                'association_tolerance_s':tolerance,'ate_rmse_m':float(np.sqrt(np.mean(err**2))),
                'ate_median_m':float(np.median(err)),
                'rpe_adjacent_translation_rmse_m':float(np.sqrt(np.mean(rel**2))),
                'warning':'Adjacent-pose translation RPE; not full SE3 rotational RPE'}
    except Exception as ex:return {'available':False,'reason':str(ex)}


def inspect_backend(path,solver=None):
    path=Path(path);log=path.parent/'execution.log';csvfile=path/'iterations.csv'
    out={'trajectory':trajectory(path/'trajectory.tum'),'log_present':log.is_file()}
    if log.is_file():
        text=log.read_text(errors='replace')
        cg=[{'iterations':int(m[0]),'estimated_error':float(m[1]),'relative_residual':float(m[2]),
             'converged':m[3].lower() in ('1','true')} for m in CG.findall(text)]
        out['cg']={'calls':len(cg),'passed':sum(c['converged'] for c in cg),'failed':sum(not c['converged'] for c in cg),
                   'details':cg}
        out['cg_solver_calls']=text.count('[OEGM::solveNormalEqCG]')
        out['cholesky_solver_calls']=text.count('[OEGM::solveNormalEq]')
        m=PREVIEW.search(text)
        if m:
            out['objective']={'initial':float(m[1]),'final':float(m[2]),'accepted':int(m[3]),'termination':m[4]}
        counts=[int(x) for x in INLIERS.findall(text)]
        out['reported_total_events']=counts[-1] if counts else None
        if solver=='cg' and out['cg_solver_calls'] and len(cg)!=out['cg_solver_calls']:
            out['cg_diagnostics_complete']=False
        elif solver=='cg':out['cg_diagnostics_complete']=bool(cg)
    if csvfile.is_file():
        with csvfile.open(newline='') as f:rows=list(csv.DictReader(f))
        out['lm']={'iterations':len(rows),'solve_ok':sum(x.get('solve_ok')=='1' for x in rows),
                   'accepted':sum(x.get('accepted')=='1' for x in rows)}
    return out


def config(a):
    if a.duration<=0 or a.dt<=0 or a.max_events<1 or a.iterations<1 or a.repeats<1:raise ValueError('Invalid settings')
    if not re.fullmatch('[A-Za-z0-9_-]+',a.run_id):raise ValueError('Invalid run ID')
    selected=list(PIPELINES) if a.pipelines=='all' else list(dict.fromkeys(x.strip() for x in a.pipelines.split(',') if x.strip()))
    if not selected or any(x not in PIPELINES for x in selected):raise ValueError('Unknown pipeline')
    return {'selected':selected,'solver':a.solver,'emba_model_mode':a.emba_model_mode,'start':a.start,'duration':a.duration,'dt':a.dt,
            'max_events':a.max_events,'iterations':a.iterations,'max_knots':a.max_knots,
            'repeats':a.repeats,'frontend_timeout':a.frontend_timeout,'backend_timeout':a.backend_timeout,
            'epba_root':str(a.epba_root.expanduser()),'ground_truth':str(a.ground_truth) if a.ground_truth else None,
            'gt_tolerance':a.gt_tolerance}


def window_ns(cfg):
    # Decimal conversion avoids binary float rounding at nanosecond boundaries.
    start = Decimal(str(cfg['start']))
    duration = Decimal(str(cfg['duration']))
    if not start.is_finite() or not duration.is_finite() or start < 0 or duration <= 0:
        raise ValueError('Invalid event window')
    scale = Decimal('1000000000')
    first = int((start * scale).to_integral_value(rounding=ROUND_HALF_UP))
    last = int(((start + duration) * scale).to_integral_value(rounding=ROUND_HALF_UP))
    if last <= first:
        raise ValueError('Event window is less than one nanosecond')
    return first, last


def plan_one(mod,selected,run_dir,cfg):
    front,back=mod.create_plan(selected,run_dir)
    first_ns,last_ns=window_ns(cfg)
    epba_exe=Path(cfg['epba_root'])/'install/epba_ros2/lib/epba_ros2/epba_native_cli'
    for name,cmd in list(back.items()):
        if name.endswith('_emba'):
            # EMBA accepts absolute event-file timestamps in nanoseconds.
            # Replace any inherited boundaries rather than duplicating options.
            cleaned=[];i=0
            while i<len(cmd):
                if cmd[i] in ('--start-ns','--stop-ns','--model-mode'):
                    if i+1>=len(cmd):raise ValueError(f'{name}: missing value for {cmd[i]}')
                    i+=2;continue
                cleaned.append(cmd[i]);i+=1
            cleaned+=['--start-ns',str(first_ns),'--stop-ns',str(last_ns),'--model-mode',cfg['emba_model_mode']]
            back[name]=cleaned
            continue
        if not name.endswith('_epba'):continue
        cleaned=[];i=0
        while i<len(cmd):
            if cmd[i] in ('--start','--duration','--dt','--max-events','--iterations','--max-knots','--solver','--init-map'):
                i+=2;continue
            cleaned.append(cmd[i]);i+=1
        cleaned[0]=str(epba_exe)
        cleaned+=['--start',str(cfg['start']),'--duration',str(cfg['duration']),
                  '--dt',str(cfg['dt']),'--max-events',str(cfg['max_events']),
                  '--iterations',str(cfg['iterations']),'--max-knots',str(cfg['max_knots']),
                  '--init-map','events','--solver',cfg['solver']]
        back[name]=cleaned
    return front,back


def preflight(mod,front,back,cfg,execute):
    issues=[];warnings=[]
    for name,cmd in back.items():
        if cmd[0].startswith('/') and not Path(cmd[0]).is_file():issues.append(f'{name}: executable missing {cmd[0]}')
    if execute:
        for overlay in (mod.WS/'ports/cmax_ros2_port/install/setup.zsh',mod.WS/'ports/eroam_ros2_port/install/setup.zsh'):
            if not overlay.is_file():issues.append(f'Missing ROS overlay {overlay}')
    if cfg['ground_truth'] and not Path(cfg['ground_truth']).is_file():issues.append('Ground truth file missing')
    warnings.extend(['EPBA --max-events is a cap; actual event count must be checked from logs',
                     'CMax/EROAM frontend event window and calibration equivalence are not automatically established',
                     'EMBA and EPBA have different objectives; do not compare raw objective values',
                     'RSS is sampled process-tree RSS, not OS high-water-mark'])
    return issues,warnings


def stage_result(mod,cmd,log,timeout,cmax=False):
    r=mod.supervise(cmd,log,timeout=timeout,cmax=cmax)
    r['status']='PASS' if (r['cleanup_complete'] and not r['forced_termination'] and
        ((cmax and r['cmax_eof'] is not None) or (not cmax and r['return_code']==0))) else 'FAIL'
    return r


def validate_frontend_reuse(source_trial, cfg, frontend_names):
    """Fail closed before starting any new benchmark process."""
    source_trial=Path(source_trial).expanduser().resolve()
    state_file=source_trial/'results_full.json'
    if not state_file.is_file():
        raise RuntimeError(f'Reuse source missing results_full.json: {state_file}')
    old=json.loads(state_file.read_text())
    previous=old.get('config',{})
    for key in ('start','dt','max_events','iterations','max_knots','solver'):
        if previous.get(key)!=cfg.get(key):
            raise RuntimeError(f'Cannot reuse: configuration {key} differs ({previous.get(key)} vs {cfg.get(key)})')
    if cfg['duration'] < previous.get('duration', float('inf')):
        raise RuntimeError('Cannot reuse: new window is shorter than original window')
    if cfg['repeats']!=1:
        raise RuntimeError('Frontend reuse currently supports exactly one trial (--repeats 1)')
    if old.get('manifest_sha256')!=sha256(ROOT/'manifest.json'):
        raise RuntimeError('Cannot reuse: manifest.json hash differs')
    for filename in ('controller.py','nuro-bench'):
        if old.get('source_hashes',{}).get(filename)!=sha256(ROOT/filename):
            raise RuntimeError(f'Cannot reuse: {filename} source hash differs')
    first_ns,last_ns=window_ns(cfg)
    sources={}
    for name in frontend_names:
        record=old.get('stages',{}).get(name)
        path=source_trial/name/'frontend/trajectory.tum'
        if not record or record.get('status')!='PASS' or not record.get('trajectory',{}).get('valid'):
            raise RuntimeError(f'Cannot reuse: {name} did not PASS in source')
        actual=trajectory(path)
        if not actual.get('valid'):
            raise RuntimeError(f'Cannot reuse: {name} trajectory invalid: {actual}')
        if actual['poses']!=record['trajectory'].get('poses'):
            raise RuntimeError(f'Cannot reuse: {name} trajectory pose count changed')
        # The frontends are whole-dataset runs. Their trajectory must cover the new backend window.
        if actual['start_s']>first_ns/1e9+1e-6 or actual['end_s']<last_ns/1e9-1e-6:
            raise RuntimeError(f'Cannot reuse: {name} trajectory does not cover requested window')
        folder=source_trial/name
        if not (folder/'execution.log').is_file():
            raise RuntimeError(f'Cannot reuse: {name} execution log missing')
        sources[name]={'folder':folder,'record':record,'trajectory_sha256':sha256(path)}
    return source_trial,sources


def execute_trial(mod,run_dir,front,back,cfg,manifest,reuse=None):
    run_dir.mkdir(parents=True,exist_ok=False)
    state={'schema':'nuro-full-benchmark-v2','created_utc':datetime.now(timezone.utc).isoformat(),
           'config':cfg,'manifest_sha256':sha256(ROOT/'manifest.json'),
           'source_hashes':{name:sha256(ROOT/name) for name in ('controller.py','nuro-bench','overall_benchmark.py')},
           'stages':{},'pipelines':{},'status':'RUNNING'}
    write_json(run_dir/'results_full.json',state)
    try:
        for name,cmd in front.items():
            if reuse is not None:
                src=reuse['sources'][name]
                print('REUSE FRONTEND',name,flush=True)
                shutil.copytree(src['folder'],run_dir/name,symlinks=False)
                copied=run_dir/name/'frontend/trajectory.tum'
                if sha256(copied)!=src['trajectory_sha256']:
                    raise RuntimeError(f'Copied frontend {name} hash mismatch')
                r=dict(src['record'])
                r['reused']=True
                r['reuse_source']=str(src['folder'])
                r['trajectory_sha256']=src['trajectory_sha256']
            else:
                print('FRONTEND',name,flush=True)
                r=stage_result(mod,cmd,run_dir/name/'execution.log',cfg['frontend_timeout'],cmax=name=='cmax')
            r['trajectory']=trajectory(run_dir/name/'frontend/trajectory.tum')
            state['stages'][name]=r;write_json(run_dir/'results_full.json',state)
            if r['status']!='PASS' or not r['trajectory']['valid']:raise RuntimeError(f'Frontend {name} failed validation')
        for name in cfg['selected']:
            if name in ('cmax','eroam'):
                state['pipelines'][name]={'status':'PASS','trajectory':state['stages'][name]['trajectory'],
                                          'elapsed_seconds':state['stages'][name]['elapsed_seconds'],
                                          'peak_sampled_rss_mib':state['stages'][name]['peak_sampled_rss_mib']}
                continue
            print('BACKEND',name,flush=True)
            r=stage_result(mod,back[name],run_dir/name/'execution.log',cfg['backend_timeout'])
            r['analysis']=inspect_backend(run_dir/name/'backend',cfg['solver'] if name.endswith('_epba') else None)
            if name.endswith('_epba'):
                analysis=r['analysis'];obj=analysis.get('objective')
                if cfg['solver']=='cg' and not analysis.get('cg_diagnostics_complete'):
                    r['status']='FAIL';r['reason']='Missing/incomplete CG diagnostics'
                if obj is None or not math.isfinite(obj['final']) or obj['final']>obj['initial']:
                    r['status']='FAIL';r['reason']='EPBA objective invalid or increased'
                if analysis.get('reported_total_events') != cfg['max_events']:
                    r['workload_verified']=False
                    r['workload_warning']='Reported event count differs from configured cap or is unavailable'
                else:r['workload_verified']=True
            if not r['analysis']['trajectory']['valid']:
                r['status']='FAIL';r['reason']='Backend trajectory missing or invalid'
            state['pipelines'][name]=r;write_json(run_dir/'results_full.json',state)
            if r['status']!='PASS':raise RuntimeError(f'Backend {name} failed validation: {r.get("reason", "process failure")}')
        if cfg['ground_truth']:
            for name,p in state['pipelines'].items():
                trajectory_path=(run_dir/name/'backend/trajectory.tum' if '_' in name else run_dir/name/'frontend/trajectory.tum')
                p['ground_truth']=gt_metrics(trajectory_path,cfg['ground_truth'],cfg['gt_tolerance'])
        state['status']='COMPLETE'
    except Exception as ex:
        state['status']='FAILED';state['error']=f'{type(ex).__name__}: {ex}'
        print('ERROR:',state['error'],file=sys.stderr)
    finally:
        state['finished_utc']=datetime.now(timezone.utc).isoformat()
        write_json(run_dir/'results_full.json',state)
    return state


def summarize(root,trial_states,cfg):
    out={'schema':'nuro-full-summary-v2','config':cfg,'trials':[],'pipelines':{},'limitations':[
        'ATE/RPE available only with timestamp-compatible ground truth',
        'Calibration and temporal equivalence require dataset-specific external verification',
        'No raw-objective comparison between EMBA and EPBA']}
    for i,state in enumerate(trial_states,1):
        out['trials'].append({'trial':i,'status':state['status'],'directory':str(root/f'trial_{i:02d}')})
    for p in cfg['selected']:
        entries=[]
        for st in trial_states:
            r=st.get('pipelines',{}).get(p)
            if r and r.get('status')=='PASS':entries.append(r)
        times=[r['elapsed_seconds'] for r in entries if 'elapsed_seconds' in r]
        rss=[r['peak_sampled_rss_mib'] for r in entries if 'peak_sampled_rss_mib' in r]
        out['pipelines'][p]={'successful_trials':len(entries),'requested_trials':cfg['repeats'],
            'runtime_seconds':{'mean':statistics.mean(times),'stdev':statistics.stdev(times) if len(times)>1 else None} if times else None,
            'peak_sampled_rss_mib':{'mean':statistics.mean(rss),'stdev':statistics.stdev(rss) if len(rss)>1 else None} if rss else None}
    write_json(root/'summary.json',out)
    with (root/'summary.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['pipeline','successful_trials','requested_trials','runtime_mean_s','runtime_stdev_s','peak_rss_mean_mib'])
        for p,v in out['pipelines'].items():
            t=v['runtime_seconds'];m=v['peak_sampled_rss_mib']
            w.writerow([p,v['successful_trials'],v['requested_trials'],t['mean'] if t else '',t['stdev'] if t else '',m['mean'] if m else ''])
    lines=['# Neuromorphic SLAM benchmark summary','',f"Run: {root.name}",'',
           '| Pipeline | Successful trials | Mean runtime (s) | Mean sampled peak RSS (MiB) |',
           '|---|---:|---:|---:|']
    for p,v in out['pipelines'].items():
        t=v['runtime_seconds'];m=v['peak_sampled_rss_mib']
        lines.append(f"| {p} | {v['successful_trials']}/{v['requested_trials']} | {t['mean']:.3f} "
                     f"| {m['mean']:.1f} |" if t and m else f"| {p} | {v['successful_trials']}/{v['requested_trials']} | N/A | N/A |")
    lines+=['','## Limitations']+['- '+x for x in out['limitations']]
    (root/'summary.md').write_text('\n'.join(lines)+'\n')
    return out


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action',choices=('plan','run','audit'))
    ap.add_argument('--run-id',default='journal_full_v1')
    ap.add_argument('--pipelines',default='all')
    ap.add_argument('--solver',choices=('cg','cholesky'),default='cg')
    ap.add_argument('--emba-model-mode',choices=('reference','periodic'),default='periodic')
    ap.add_argument('--start',type=float,default=0.056249173)
    ap.add_argument('--duration',type=float,default=2.20)
    ap.add_argument('--dt',type=float,default=0.05)
    ap.add_argument('--max-events',type=int,default=3087255)
    ap.add_argument('--iterations',type=int,default=5)
    ap.add_argument('--max-knots',type=int,default=96)
    ap.add_argument('--repeats',type=int,default=1)
    ap.add_argument('--frontend-timeout',type=int,default=3600)
    ap.add_argument('--backend-timeout',type=int,default=14400)
    ap.add_argument('--epba-root',type=Path,default=Path.home()/'neuromorphic_slam_milestone_b_test/ports/epba_ros2_port')
    ap.add_argument('--ground-truth',type=Path)
    ap.add_argument('--gt-tolerance',type=float,default=0.025)
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--reuse-frontends-from',type=Path,help='Existing trial_01 directory containing validated frontends')
    ap.add_argument('--run-dir',type=Path,help='For audit: existing trial directory')
    args=ap.parse_args()
    if args.action=='audit':
        if not args.run_dir:ap.error('audit requires --run-dir')
        path=args.run_dir.resolve();r={'run_directory':str(path),'pipelines':{}}
        for p in PIPELINES:
            if p in ('cmax','eroam'):r['pipelines'][p]={'trajectory':trajectory(path/p/'frontend/trajectory.tum')}
            else:r['pipelines'][p]=inspect_backend(path/p/'backend',args.solver if p.endswith('_epba') else None)
        print(json.dumps(r,indent=2,allow_nan=False));return
    cfg=config(args)
    if args.reuse_frontends_from and args.action!='run':
        ap.error('--reuse-frontends-from requires run action')
    mod=cli();mod.manifest()
    root=ROOT/'runs'/args.run_id
    if root.exists():raise FileExistsError(f'Refusing to overwrite existing run: {root}')
    trials=[];issues=[];warnings=[]
    for i in range(1,cfg['repeats']+1):
        trial_dir=root/f'trial_{i:02d}'
        front,back=plan_one(mod,cfg['selected'],trial_dir,cfg)
        x,y=preflight(mod,front,back,cfg,args.action=='run' and args.execute)
        issues+=x;warnings+=y
        trials.append((trial_dir,front,back))
    plan={'schema':'nuro-full-plan-v2','run_id':args.run_id,'config':cfg,'issues':sorted(set(issues)),
          'warnings':sorted(set(warnings)),'trials':[
            {'directory':str(d),'frontend_commands':f,'backend_commands':b} for d,f,b in trials]}
    if args.action=='plan' or not args.execute:
        print(json.dumps(plan,indent=2,allow_nan=False))
        print('DRY RUN — no algorithms executed',file=sys.stderr)
        return
    if issues:raise RuntimeError('Preflight failed: '+'; '.join(sorted(set(issues))))
    reuse=None
    if args.reuse_frontends_from:
        source,sources=validate_frontend_reuse(args.reuse_frontends_from,cfg,trials[0][1].keys())
        reuse={'source_trial':str(source),'sources':sources}
        plan['frontend_reuse']={'source_trial':str(source),'trajectory_sha256':{k:v['trajectory_sha256'] for k,v in sources.items()}}
    root.mkdir(parents=True,exist_ok=False)
    write_json(root/'plan.json',plan)
    states=[]
    for trial_dir,front,back in trials:
        states.append(execute_trial(mod,trial_dir,front,back,cfg,mod.manifest(),reuse=reuse))
        summarize(root,states,cfg)
        if states[-1]['status']!='COMPLETE':break
    summary=summarize(root,states,cfg)
    print('RESULTS:',root/'summary.json')
    if any(x['status']!='COMPLETE' for x in states):sys.exit(1)

if __name__=='__main__':main()
