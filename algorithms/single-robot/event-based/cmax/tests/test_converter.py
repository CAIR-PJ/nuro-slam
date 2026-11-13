import contextlib
import importlib.util
import io
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import yaml
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores,get_typestore,get_types_from_msg

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('converter',ROOT/'tools/convert_ecrot.py')
c=importlib.util.module_from_spec(spec); spec.loader.exec_module(c)
EVENT='uint16 x\nuint16 y\ntime ts\nbool polarity\n'
ARRAY='std_msgs/Header header\nuint32 height\nuint32 width\ndvs_msgs/Event[] events\n'

def bag(path, times=(1,2,3), camera=False, compression=None):
    store=get_typestore(Stores.ROS1_NOETIC)
    store.register(get_types_from_msg(EVENT,'dvs_msgs/msg/Event'))
    store.register(get_types_from_msg(ARRAY,'dvs_msgs/msg/EventArray'))
    T=store.types['builtin_interfaces/msg/Time']; H=store.types['std_msgs/msg/Header']
    E=store.types['dvs_msgs/msg/Event']; A=store.types['dvs_msgs/msg/EventArray']
    with Writer(path) as writer:
        # Writer compression must be set before open, so compressed tests use separate helper below.
        conn=writer.add_connection('/cam0/events','dvs_msgs/msg/EventArray',typestore=store)
        if camera:
            cc=writer.add_connection('/cam0/camera_info','sensor_msgs/msg/CameraInfo',typestore=store)
            roi=store.types['sensor_msgs/msg/RegionOfInterest'](0,0,0,0,False)
            msg=store.types['sensor_msgs/msg/CameraInfo'](H(0,T(0,0),''),128,128,'plumb_bob',np.zeros(5),np.array([91.,0,64,0,91,64,0,0,1]),np.eye(3).flatten(),np.array([91.,0,64,0,0,91,64,0,0,0,1,0]),0,0,roi)
            writer.write(cc,0,store.serialize_ros1(msg,cc.msgtype))
        events=[E(i+1,i+2,T(10,t),i%2==0) for i,t in enumerate(times)]
        msg=A(H(1,T(10,0),''),128,128,events)
        writer.write(conn,10_000_000_000,store.serialize_ros1(msg,conn.msgtype))

class ConverterTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        self.cal=self.root/'camera.yaml'
        self.cal.write_text(yaml.safe_dump(dict(image_width=128,image_height=128,distortion_model='plumb_bob',camera_matrix={'data':[91.,0,64,0,91,64,0,0,1]},distortion_coefficients={'data':[0]*5},rectification_matrix={'data':[1,0,0,0,1,0,0,0,1]},projection_matrix={'data':[91.,0,64,0,0,91,64,0,0,0,1,0]})))
    def tearDown(self): self.tmp.cleanup()
    def convert(self,*args,**kwargs):
        with contextlib.redirect_stdout(io.StringIO()): return c.convert(*args,**kwargs)
    def test_exact_integer_event_roundtrip(self):
        b=self.root/'test.bag'; bag(b); out=self.root/'events.bin'
        m=self.convert(b,out,calibration=self.cal)
        data=out.read_bytes(); self.assertEqual(data[:8],c.MAGIC)
        self.assertEqual(struct.unpack('<IIQ',data[8:24]),(128,128,3))
        self.assertEqual(c.RECORD.unpack(data[24:37]),(1,2,10_000_000_001,1))
        self.assertEqual(c.RECORD.unpack(data[-13:]),(3,4,10_000_000_003,1))
        self.assertEqual(m['event_count'],3); self.assertEqual(m['last_event_ns'],10_000_000_003)
        with self.assertRaises(FileExistsError): self.convert(b,out,calibration=self.cal)
    def test_embedded_camera_info(self):
        b=self.root/'test.bag'; bag(b,camera=True)
        m=self.convert(b,self.root/'events.bin'); self.assertEqual(m['calibration_source']['bag_topic'],'/cam0/camera_info')
    def test_out_of_order_is_rejected_and_partial_removed(self):
        b=self.root/'test.bag'; bag(b,(2,1)); out=self.root/'events.bin'
        with self.assertRaisesRegex(ValueError,'time order'): self.convert(b,out,calibration=self.cal)
        self.assertFalse(out.exists()); self.assertFalse(Path(str(out)+'.partial').exists())
    def test_dimensions_must_agree(self):
        b=self.root/'test.bag'; bag(b); data=yaml.safe_load(self.cal.read_text()); data['image_width']=640; self.cal.write_text(yaml.safe_dump(data))
        with self.assertRaisesRegex(ValueError,'calibration size'): self.convert(b,self.root/'events.bin',calibration=self.cal)
    def test_ecrot_projection_repair_is_explicit(self):
        original=ROOT/'docs/ecrot_calibration_original/synth_data/DVS128-synthetic.yaml'
        with self.assertRaisesRegex(ValueError,'P last row'): c.calibration_file(original)
        cal=c.calibration_file(original,True); self.assertEqual(cal['camera_p'][8:],[0.,0.,1.,0.])
    def test_camera_required_if_absent(self):
        b=self.root/'test.bag'; bag(b)
        with self.assertRaisesRegex(ValueError,'supply --calibration'): self.convert(b,self.root/'events.bin')
    def test_inventory(self):
        b=self.root/'test.bag'; bag(b)
        inventory=self.convert(b,None,inspect=True); self.assertEqual(inventory[0]['type'],'dvs_msgs/msg/EventArray')
if __name__=='__main__': unittest.main()
