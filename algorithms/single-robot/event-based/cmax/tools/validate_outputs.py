#!/usr/bin/env python3
import argparse
from decimal import Decimal
import gzip
import json
import math
from pathlib import Path
import struct

def validate(results, manifest):
    root=Path(results); meta=json.loads(Path(manifest).read_text()); previous=-1; count=0
    for line in (root/'trajectory.tum').read_text().splitlines():
        if not line.strip() or line.startswith('#'): continue
        cols=line.split()
        if len(cols)!=8: raise ValueError('TUM row must have 8 fields')
        ts=int(Decimal(cols[0])*1_000_000_000); nums=[float(x) for x in cols[1:]]
        if ts<=previous: raise ValueError('trajectory timestamp not strictly increasing')
        if not meta['first_event_ns']<=ts<=meta['last_event_ns']: raise ValueError('pose timestamp outside recording')
        if not all(math.isfinite(x) for x in nums): raise ValueError('non-finite pose')
        if any(x!=0 for x in nums[:3]): raise ValueError('rotational model must have zero translation')
        if abs(sum(x*x for x in nums[3:])-1)>1e-6: raise ValueError('quaternion not unit length')
        previous=ts; count+=1
    if count<2: raise ValueError('too few estimated poses')
    png=(root/'panorama.png').read_bytes()
    if png[:8]!=b'\x89PNG\r\n\x1a\n': raise ValueError('invalid panorama PNG')
    width,height=struct.unpack('>II',png[16:24])
    if width!=2*height: raise ValueError('expected 2:1 equirectangular panorama')
    raw=gzip.open(root/'panorama.raw.yml.gz','rt').read()
    if 'IG:' not in raw: raise ValueError('raw IG matrix not saved')
    report=dict(poses=count,panorama_width=width,panorama_height=height,
                status='structural checks passed; this does not establish published accuracy or algorithm parity')
    print(json.dumps(report,indent=2)); return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('results');p.add_argument('manifest');a=p.parse_args();validate(a.results,a.manifest)
