#!/usr/bin/env python3
"""Small synthetic EuRoC-shaped fixture; NEVER a EuRoC benchmark."""
import argparse
import csv
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw


def make(root):
    root = Path(root)
    for d in ['cam0/data', 'imu0']:
        (root/d).mkdir(parents=True, exist_ok=True)
    (root/'cam0/sensor.yaml').write_text('''camera_model: pinhole
distortion_model: radial-tangential
intrinsics: [400.0,400.0,320.0,240.0]
distortion_coefficients: [0.0,0.0,0.0,0.0]
resolution: [640,480]
T_BS:
  rows: 4
  cols: 4
  data: [1,0,0,0.1,0,1,0,0,0,0,1,0,0,0,0,1]
''')
    (root/'imu0/sensor.yaml').write_text('''T_BS:
  rows: 4
  cols: 4
  data: [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]
''')
    rng = np.random.default_rng(42)
    points = np.column_stack((rng.uniform(-2, 2, 600), rng.uniform(-1.4, 1.4, 600), rng.uniform(4, 10, 600)))
    shades = rng.integers(100, 256, len(points))
    # First 0.5 seconds stationary for initialization; then constant-speed translation.
    with (root/'cam0/data.csv').open('w', newline='') as f:
        writer = csv.writer(f);writer.writerow(['#timestamp [ns]', 'filename'])
        for i in range(31):
            stamp = 1000000000+i*50000000
            image = Image.new('L', (640,480));draw = ImageDraw.Draw(image)
            shift = max(i-10, 0)*.075
            for (x,y,z),shade in zip(points,shades):
                u,v = round(400*(x-shift)/z+320),round(400*y/z+240)
                draw.rectangle((u-2,v-2,u+2,v+2), fill=int(shade))
            image.save(root/f'cam0/data/{stamp}.png');writer.writerow([stamp, f'{stamp}.png'])
    with (root/'imu0/data.csv').open('w', newline='') as f:
        writer=csv.writer(f);writer.writerow(['#timestamp', 'wx','wy','wz','ax','ay','az'])
        for i in range(301):
            writer.writerow([1000000000+i*5000000,0,0,0,0,0,9.81])
    return root


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('destination');args=p.parse_args();print(make(args.destination))
