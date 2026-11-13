#!/usr/bin/env python3
"""Read ROS1 bag V2 without ROS1. Preserve each selected event's integer timestamp."""
import argparse
import hashlib
import json
import math
import struct
from pathlib import Path
import yaml
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

MAGIC=b'CMAXEV1\n'
RECORD=struct.Struct('<HHqB')

def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()

def calibration_file(path, repair=False):
    data=yaml.safe_load(Path(path).read_text())
    c={'camera_width':int(data['image_width']),'camera_height':int(data['image_height']),
       'camera_distortion_model':data['distortion_model']}
    for target,source in [('camera_k','camera_matrix'),('camera_d','distortion_coefficients'),('camera_r','rectification_matrix'),('camera_p','projection_matrix')]:
        c[target]=[float(x) for x in data[source]['data']]
    if repair:
        k=c['camera_k']; c['camera_p']=[k[0],k[1],k[2],0.,k[3],k[4],k[5],0.,0.,0.,1.,0.]
    validate_calibration(c)
    return c

def validate_calibration(c):
    if not (0<c['camera_width']<=65536 and 0<c['camera_height']<=65536): raise ValueError('invalid camera dimensions')
    for key,size in [('camera_k',9),('camera_r',9),('camera_p',12)]:
        if len(c[key])!=size or not all(math.isfinite(x) for x in c[key]): raise ValueError(f'invalid {key}')
    if c['camera_k'][0]<=0 or c['camera_k'][4]<=0: raise ValueError('invalid focal length')
    if c['camera_distortion_model'] not in ('plumb_bob','rational_polynomial'): raise ValueError('unsupported camera distortion model')
    if any(abs(x-y)>1e-10 for x,y in zip(c['camera_p'][8:],[0,0,1,0])):
        raise ValueError('invalid pinhole P last row. ECRot DVS128 YAML has a known anomalous row; explicitly use --repair-projection-from-k to record a K-derived P.')

def calibration_msg(m):
    # ROS1 CameraInfo has uppercase D/K/R/P; rosbags retains those field names.
    c=dict(camera_width=int(m.width),camera_height=int(m.height),camera_distortion_model=m.distortion_model,
           camera_k=[float(x) for x in m.K],camera_r=[float(x) for x in m.R],camera_p=[float(x) for x in m.P],camera_d=[float(x) for x in m.D])
    if m.binning_x or m.binning_y or m.roi.x_offset or m.roi.y_offset or m.roi.width or m.roi.height:
        raise ValueError('non-default CameraInfo ROI/binning requires explicit calibrated full-resolution YAML')
    validate_calibration(c); return c

def convert(bag, output, topic=None, calibration=None, repair=False, inspect=False):
    bag=Path(bag); output=Path(output) if output else None
    with Reader(bag) as reader:
        inventory=[dict(topic=c.topic,type=c.msgtype,count=c.msgcount) for c in reader.connections]
        if inspect:
            print(json.dumps(inventory,indent=2)); return inventory
        candidates=[c for c in reader.connections if c.msgtype.endswith('/EventArray') and (topic is None or c.topic==topic)]
        names={c.topic for c in candidates}
        if len(names)!=1: raise ValueError(f'select exactly one EventArray topic with --topic; found {names}')
        store=get_typestore(Stores.ROS1_NOETIC)
        for conn in reader.connections:
            store.register(get_types_from_msg(conn.msgdef.data,conn.msgtype))
        source={}
        if calibration:
            camera=calibration_file(calibration,repair)
            source=dict(file=str(Path(calibration).resolve()),sha256=digest(calibration),projection_rebuilt_from_K=repair)
        else:
            if repair: raise ValueError('--repair-projection-from-k requires --calibration')
            cameras=[c for c in reader.connections if c.msgtype=='sensor_msgs/msg/CameraInfo']
            camera_names={c.topic for c in cameras}
            if len(camera_names)!=1: raise ValueError('supply --calibration: bag does not contain one unambiguous CameraInfo topic')
            camera=None
            for conn,ts,raw in reader.messages(connections=cameras):
                camera=calibration_msg(store.deserialize_ros1(raw,conn.msgtype)); break
            if camera is None: raise ValueError('empty camera topic; supply --calibration')
            source=dict(bag_topic=next(iter(camera_names)))
        if output is None: raise ValueError('--output required')
        targets=[output,Path(str(output)+'.camera.yaml'),Path(str(output)+'.json')]
        if any(x.exists() for x in targets): raise FileExistsError('output exists; choose a new output path')
        output.parent.mkdir(parents=True,exist_ok=True)
        partial=Path(str(output)+'.partial')
        count=0; packets=0; previous=-1; first=None
        try:
            with partial.open('xb+') as out:
                out.write(MAGIC); out.write(struct.pack('<IIQ',camera['camera_width'],camera['camera_height'],0))
                for conn,bag_time,raw in reader.messages(connections=candidates):
                    m=store.deserialize_ros1(raw,conn.msgtype)
                    if m.width!=camera['camera_width'] or m.height!=camera['camera_height']:
                        raise ValueError(f'calibration size differs from {conn.topic}: {m.width}x{m.height}')
                    block=bytearray()
                    for e in m.events:
                        ns=int(e.ts.sec)*1_000_000_000+int(e.ts.nanosec)
                        if not (0<=ns<=2_147_483_647_999_999_999): raise ValueError('event outside ROS2 time range')
                        if not (0<=e.x<m.width and 0<=e.y<m.height): raise ValueError('event pixel out of bounds')
                        if ns<previous: raise ValueError('events out of time order: refusing to silently sort')
                        if first is None: first=ns
                        previous=ns; block.extend(RECORD.pack(int(e.x),int(e.y),ns,bool(e.polarity))); count+=1
                    out.write(block); packets+=1
                if count==0: raise ValueError('empty event topic')
                out.seek(16); out.write(struct.pack('<Q',count))
            partial.rename(output)
        except BaseException:
            if partial.exists(): partial.unlink()
            raise
        targets[1].write_text(yaml.safe_dump({'/**':{'ros__parameters':camera}},sort_keys=False))
        manifest=dict(format='CMAXEV1',record='<HHqB',header_bytes=24,record_bytes=13,
            bag=str(bag.resolve()),bag_sha256=digest(bag),event_topic=next(iter(names)),
            event_count=count,event_packets=packets,first_event_ns=first,last_event_ns=previous,
            timestamps='absolute event timestamps; no rebasing, sorting, thinning or bag-time substitution',
            calibration_source=source,event_stream_sha256=digest(output),inventory=inventory)
        targets[2].write_text(json.dumps(manifest,indent=2)+'\n')
        print(json.dumps(manifest,indent=2)); return manifest

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path); p.add_argument('--output',type=Path)
    p.add_argument('--topic'); p.add_argument('--calibration',type=Path)
    p.add_argument('--repair-projection-from-k',action='store_true')
    p.add_argument('--inspect',action='store_true'); a=p.parse_args()
    convert(a.bag,a.output,a.topic,a.calibration,a.repair_projection_from_k,a.inspect)
if __name__=='__main__': main()
