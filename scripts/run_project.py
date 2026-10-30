#!/usr/bin/env python3
"""One command to build, test, run and plot. Install documented dependencies first."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from make_smoke_data import make

ROOT=Path(__file__).resolve().parents[1]

def command(args):
    print('+', ' '.join(map(str,args)), flush=True)
    subprocess.run(list(map(str,args)), cwd=ROOT, check=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, help='EuRoC mav0 directory; omit for synthetic smoke data')
    p.add_argument('--output', type=Path, default=ROOT/'output')
    p.add_argument('--max-frames', type=int, default=300)
    p.add_argument('--start-frame', type=int, default=0)
    p.add_argument('--sim3', action='store_true')
    p.add_argument('--skip-build', action='store_true', help='use an existing executable')
    p.add_argument('--executable', type=Path)
    args=p.parse_args()
    build=ROOT/'build';out=args.output.resolve()
    if not args.skip_build:
        command(['cmake','-S',ROOT,'-B',build,'-DCMAKE_BUILD_TYPE=Release'])
        command(['cmake','--build',build,'--config','Release','-j','2'])
        command(['ctest','--test-dir',build,'-C','Release','--output-on-failure'])
    command([sys.executable,ROOT/'scripts/test_evaluation.py','-v'])
    exe=args.executable.resolve() if args.executable else build/('Release/vio_lite.exe' if os.name=='nt' else 'vio_lite')
    def run(dataset, real):
        command([exe,'--dataset',dataset,'--config',ROOT/'config/default.yaml','--output',out,
                 '--max-frames',args.max_frames,'--start-frame',args.start_frame])
        evaluate=[sys.executable,ROOT/'scripts/evaluate.py','--run',out]
        if real:
            evaluate+=['--dataset',dataset]
        if args.sim3:
            evaluate+=['--sim3']
        command(evaluate)
    if args.dataset:
        run(args.dataset.resolve(), True)
    else:
        print('SYNTHETIC SMOKE RUN; no EuRoC benchmark or recovered-scale claim.', flush=True)
        with tempfile.TemporaryDirectory(prefix='vio-smoke-') as temp:
            run(make(Path(temp)/'mav0'), False)
