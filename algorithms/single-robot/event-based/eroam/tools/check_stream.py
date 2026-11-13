#!/usr/bin/env python3
"""Inspect a CMAXEV1 event stream without third-party dependencies."""
import argparse,struct,pathlib
p=argparse.ArgumentParser();p.add_argument('file',type=pathlib.Path);a=p.parse_args()
with a.file.open('rb') as f:
    h=f.read(24)
    if len(h)!=24 or h[:7]!=b'CMAXEV1':raise SystemExit('Not a CMAXEV1 file')
    print('Header bytes:',h.hex())
    print('File size:',a.file.stat().st_size)
    print('Payload records:',(a.file.stat().st_size-24)//13)
    if (a.file.stat().st_size-24)%13:raise SystemExit('Invalid record alignment')
    for pos in [24,a.file.stat().st_size-13]:
        f.seek(pos);print('Sample:',struct.unpack('<HHqB',f.read(13)))
