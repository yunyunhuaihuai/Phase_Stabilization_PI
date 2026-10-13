#!/usr/bin/env python3
"""Run the offline PI binary and summarize simulation data; no instrument imports."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import subprocess

PROJECT=Path(__file__).resolve().parents[1]

def summarize(path, target_mv=0):
    with Path(path).open(encoding="utf-8",newline="") as stream:
        rows=list(csv.DictReader(stream))
    if not rows:
        raise ValueError("simulation produced no samples")
    errors=[target_mv-float(row["V_PHS_mV"]) for row in rows]
    if not all(math.isfinite(x) for x in errors):
        raise ValueError("nonfinite simulation sample")
    return dict(data_origin="simulation",measurement_scope="in_loop",
                samples=len(rows),rms_error_mv=math.sqrt(statistics.fmean(x*x for x in errors)),
                peak_abs_error_mv=max(map(abs,errors)),
                accepted_cycles=sum(row["Send_accepted"]=="1" for row in rows),
                fault_rows=sum(row["State"]=="FAULT" for row in rows),
                physical_position_evidence="simulated command/driver values; no mechanical measurement",
                control_period_note="nominal 0.05 s; timing is recorded, no hard real-time claim")

def plot_simulation(csv_path, output):
    # Optional plotting dependency is imported only for an explicitly requested plot.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    with Path(csv_path).open(encoding="utf-8",newline="") as stream:
        rows=list(csv.DictReader(stream))
    t=[float(r["Elapsed_s"]) for r in rows]
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,layout="constrained")
    axes[0].plot(t,[float(r["V_PHS_mV"]) for r in rows],label="Simulated in-loop voltage")
    axes[0].axhline(0,color="gray",linewidth=.8)
    axes[0].set_ylabel("Voltage (mV)");axes[0].legend(loc="upper right")
    axes[1].plot(t,[float(r["Proposed_physical_command"]) for r in rows],label="Controller proposal")
    axes[1].plot(t,[float(r["Last_accepted_physical_command"]) for r in rows],linestyle="--",label="Software accepted")
    axes[1].set_ylabel("Absolute target (pulses)");axes[1].set_xlabel("Simulation host elapsed time (s)")
    axes[1].legend(loc="upper right")
    for ax in axes: ax.grid(alpha=.2)
    fig.suptitle("PI phase stabilization — SIMULATION")
    fig.savefig(output,dpi=150,metadata={"Description":"Synthetic simulation; no instrument measurement"})
    plt.close(fig)

def run(binary,output_dir,seconds=5,config=None,*,dry_run=False,plot=False,disturbance_mv=5):
    if not math.isfinite(seconds) or seconds<=0 or seconds>86400:
        raise ValueError("seconds must be finite and in (0,86400]")
    if not math.isfinite(disturbance_mv) or not 0<=disturbance_mv<=10:
        raise ValueError("simulation disturbance amplitude must be in [0,10] mV")
    output=Path(output_dir).resolve()
    if output.exists():
        raise ValueError("output directory already exists; choose a new run directory")
    output.mkdir(parents=True)
    config_path=output/"config.txt"
    shutil.copyfile(config or PROJECT/"config.txt",config_path)
    executable=Path(binary).resolve()
    command=[str(executable),str(config_path),"--simulation-seconds",str(seconds)]
    manifest=dict(schema="phase-stabilization-pi/simulation/v1",data_origin="simulation",
                  command=command,status="dry_run" if dry_run else "running",
                  disturbance_mv=disturbance_mv,disturbance_hz=.4,
                  assumptions="gain .0025 mV/pulse, first-order lag .04 s, 150000 pulses/s cap, .05 mV Gaussian noise")
    manifest_path=output/"manifest.json"
    manifest_path.write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    if dry_run:
        return manifest
    # The mandatory simulation duration flag is rejected before initialization
    # by a real-hardware build, so this runner cannot launch that build's devices.
    env=dict(os.environ,PI_SIM_DISTURBANCE_MV=str(disturbance_mv))
    result=subprocess.run(command,cwd=output,env=env,capture_output=True,text=True,encoding="utf-8",errors="replace",
                          timeout=seconds+90)
    (output/"controller.log").write_text(result.stdout+result.stderr,encoding="utf-8")
    manifest.update(returncode=result.returncode,status="completed" if result.returncode==0 else "failed")
    manifest_path.write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"simulation failed ({result.returncode}); see {output/'controller.log'}")
    logs=list(output.glob("Data_*.csv"))
    raw=[p for p in logs if not p.name.endswith((".diagnostics.csv",".safety.csv"))]
    if len(raw)!=1:
        raise RuntimeError("expected exactly one simulation raw log")
    metadata=Path(str(raw[0])+".metadata.txt").read_text(encoding="utf-8")
    if "DATA_ORIGIN=simulation" not in metadata:
        raise RuntimeError("log is not labeled simulation")
    values=dict(line.split("=",1) for line in metadata.splitlines() if "=" in line)
    manifest['summary']=summarize(raw[0],float(values['TARGET_VPHS_MV']))
    (output/"summary.json").write_text(json.dumps(manifest['summary'],indent=2),encoding="utf-8")
    if plot:
        plot_simulation(raw[0],output/"simulation.png")
    manifest_path.write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return manifest

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary",required=True)
    parser.add_argument("--output-dir",required=True)
    parser.add_argument("--seconds",type=float,default=5)
    parser.add_argument("--config",type=Path)
    parser.add_argument("--disturbance-mv",type=float,default=5)
    parser.add_argument("--dry-run",action="store_true")
    parser.add_argument("--plot",action="store_true",help="requires optional matplotlib")
    args=parser.parse_args(argv)
    try:
        result=run(args.binary,args.output_dir,args.seconds,args.config,dry_run=args.dry_run,plot=args.plot,disturbance_mv=args.disturbance_mv)
        print(json.dumps(result,indent=2));return 0
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}");return 1

if __name__=="__main__":
    raise SystemExit(main())
