#!/usr/bin/env python3
"""Offline PI step-limit scan with run-owned configs and fail-fast simulation."""
import argparse
import json
from pathlib import Path
import tempfile
try:
    from pi_experiment import run,PROJECT
    from experiment_plan import verify_config
except ImportError:
    from tools.pi_experiment import run,PROJECT
    from tools.experiment_plan import verify_config

def run_scan(binary,output_dir,limits,seconds=5,*,dry_run=False):
    if not limits or len(set(limits))!=len(limits) or any(not isinstance(x,int) or not 1<=x<=8000 for x in limits):
        raise ValueError("limits must be unique integers in [1,8000]")
    root=Path(output_dir).resolve()
    if root.exists(): raise ValueError("scan output already exists")
    root.mkdir(parents=True)
    original=(PROJECT/"config.txt").read_text(encoding="utf-8")
    result=dict(schema="phase-stabilization-pi/step-scan/v1",data_origin="simulation",controller="PI",runs=[])
    for limit in limits:
        lines=[]
        for line in original.splitlines():
            key=line.split("=",1)[0]
            lines.append(f"{key}={limit}" if key in ("MAX_STEP_PER_CYCLE","MAX_PHYSICAL_DELTA") else line)
        config="\n".join(lines)+"\n"
        verify_config(config,limit=limit)
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/"config.txt";path.write_text(config,encoding="utf-8")
            try:
                report=run(binary,root/f"pi-{limit}",seconds,path,dry_run=dry_run)
            except Exception as exc:
                result.update(status="stopped_after_failure",error=str(exc))
                (root/"scan_manifest.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
                raise
        result['runs'].append(dict(limit=limit,report=report))
        (root/"scan_manifest.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    result['status']="dry_run" if dry_run else "completed"
    (root/"scan_manifest.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary",required=True)
    parser.add_argument("--output-dir",required=True)
    parser.add_argument("--limits",type=int,nargs="+",default=[5000,5500])
    parser.add_argument("--seconds",type=float,default=5)
    parser.add_argument("--dry-run",action="store_true")
    args=parser.parse_args(argv)
    try:
        print(json.dumps(run_scan(args.binary,args.output_dir,args.limits,args.seconds,dry_run=args.dry_run),indent=2))
        return 0
    except (ValueError,RuntimeError,OSError) as exc:
        print(f"ERROR: {exc}");return 1

if __name__=="__main__": raise SystemExit(main())
