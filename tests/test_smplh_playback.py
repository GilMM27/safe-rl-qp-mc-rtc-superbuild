"""Synthetic, unrestricted fixtures exercise the real SMPL-H and MuJoCo tooling."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from smplh_playback.runtime import Playback, SurfaceDistance, robot_geometry
from smplh_playback.prepare import REGIONS, segment


def fixture(directory):
    # Artificial SMPL-H parameters, NOT a redistributed human model.
    n=52; nv=n*4
    parents=np.array([-1]+[0]*51)
    positions=np.array([[i*.1, (i%3)*.2, 1+(i%5)*.05] for i in range(n)],dtype=np.float32)
    tetra=np.array([[0,0,0],[.03,0,0],[0,.03,0],[0,0,.03]],dtype=np.float32)
    v=(positions[:,None]+tetra).reshape(-1,3)
    faces=np.concatenate([np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])+4*i for i in range(n)])
    weights=np.repeat(np.eye(n),4,axis=0).astype(np.float32)
    regressor=np.zeros((n,nv),dtype=np.float32)
    for i in range(n): regressor[i,4*i]=1
    path=directory/'synthetic.npz'
    np.savez(path,v_template=v,f=faces,shapedirs=np.zeros((nv,3,16),np.float32),
             posedirs=np.zeros((nv,3,459),np.float32),J_regressor=regressor,weights=weights,
             kintree_table=np.stack([parents,np.arange(n)]),
             hands_componentsl=np.eye(45,dtype=np.float32),hands_componentsr=np.eye(45,dtype=np.float32),
             hands_meanl=np.zeros(45,np.float32),hands_meanr=np.zeros(45,np.float32))
    poses=np.zeros((3,156),np.float32); poses[1:,3+16*3:3+16*3+3]=[0,0,.2]
    # Pose 16 is upper arm; direct pose offset is 16*3.
    poses[1:,48:51]=[0,0,.2]
    np.savez(directory/'motion.npz',poses=poses,trans=np.array([[0,0,0],[.2,0,0],[.4,0,0]],np.float32),
             betas=np.zeros(16,np.float32),gender=np.array('male'),mocap_framerate=np.array(10.))
    subprocess.run([sys.executable,'-m','smplh_playback.prepare','--model',str(path),
                    '--amass',str(directory/'motion.npz'),'--output',str(directory/'cache')],check=True)
    return directory/'cache',positions


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory(); cls.root=Path(cls.tmp.name)
        cls.cache,cls.rest=fixture(cls.root); cls.playback=Playback(cls.cache)

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def test_load_and_frame_zero(self):
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(self.cache/'human.xml'))
        data=mujoco.MjData(model); data.qpos[:]=self.playback.qpos[0]
        mujoco.mj_forward(model,data)
        joints=np.load(self.cache/'joints.npy')
        for i in range(52):
            bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_body_{i}')
            np.testing.assert_allclose(data.xpos[bid],joints[0,i],atol=1e-6)
        self.assertEqual(model.nq,211)
        np.testing.assert_allclose(self.playback.vertices[0][::4],self.rest,atol=1e-6)
        mid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_MESH,'smplh_surface')
        self.assertEqual(model.mesh_vertnum[mid],len(self.playback.vertices[0]))
        self.assertEqual(model.mesh_normalnum[mid],len(self.playback.vertices[0]))

    def test_advance_stop_loop_and_reset(self):
        self.assertEqual(self.playback.frame(0),0)
        self.assertEqual(self.playback.frame(.1),1)
        self.assertEqual(self.playback.frame(10),2)
        self.assertEqual(Playback(self.cache,True).frame(.3),0)
        self.assertEqual(self.playback.frame(0),0)

    def test_mesh_skeleton_consistency(self):
        import mujoco
        model=mujoco.MjModel.from_xml_path(str(self.cache/'human.xml')); data=mujoco.MjData(model)
        joints=np.load(self.cache/'joints.npy')
        for frame in range(3):
            data.qpos[:]=self.playback.qpos[frame]; mujoco.mj_forward(model,data)
            for i in range(52):
                bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,f'smplh_body_{i}')
                np.testing.assert_allclose(data.xpos[bid],joints[frame,i],atol=1e-6)
                np.testing.assert_allclose(self.playback.vertices[frame,4*i],joints[frame,i],atol=1e-6)

    def test_regions(self):
        self.assertEqual(set(self.playback.regions),set(REGIONS))
        ids=np.concatenate(list(self.playback.regions.values()))
        self.assertEqual(len(np.unique(ids)),len(self.playback.faces))
        self.assertTrue(all(len(x)>0 for x in self.playback.regions.values()))

    def test_distance_approach_and_direction(self):
        import fcl
        engine=SurfaceDistance(self.playback); engine.update(0)
        robot=fcl.CollisionObject(fcl.Sphere(.02),fcl.Transform(np.array([10.,0.,1.])))
        far=engine.query({'robot':robot})['head']
        self.assertGreater(far['distance'],0)
        robot.setTransform(fcl.Transform(np.array([9.,0.,1.])))
        near=engine.query({'robot':robot})['head']
        self.assertLess(near['distance'],far['distance'])
        ph=np.array(near['human_point']); pr=np.array(near['robot_point'])
        np.testing.assert_allclose(np.linalg.norm(pr-ph),near['distance'],atol=1e-6)
        np.testing.assert_allclose((pr-ph)/np.linalg.norm(pr-ph),near['direction'])
        self.assertGreater(near['direction'][0],0)
        human_objects=engine.objects; engine.update(0); self.assertIs(human_objects,engine.objects)

    def test_worker_protocol_and_reset(self):
        import mujoco
        import socket
        import struct
        import xml.etree.ElementTree as ET
        human=ET.parse(self.cache/'human.xml').getroot()
        body=ET.SubElement(human.find('worldbody'),'body',name='Kinova_link',pos='8 0 1')
        ET.SubElement(body,'geom',name='robot_box',type='box',size='.03 .03 .03',group='2',contype='0',conaffinity='0')
        xml=self.cache/'scene.xml'; ET.ElementTree(human).write(xml)
        model=mujoco.MjModel.from_xml_path(str(xml)); data=mujoco.MjData(model)
        scene=self.cache/'scene.mjb'; mujoco.mj_saveModel(model,str(scene))
        socket_path=self.cache/'worker.sock'; log=self.cache/'results.jsonl'
        proc=subprocess.Popen([sys.executable,'-m','smplh_playback.serve',
            '--cache',str(self.cache),'--socket',str(socket_path),
            '--robot-prefix','Kinova_','--log',str(log),'--distance-every','2',
            '--log-every','1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        try:
            import time
            for _ in range(100):
                if socket_path.exists(): break
                if proc.poll() is not None: raise RuntimeError(proc.communicate()[1].decode())
                time.sleep(.01)
            with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as sock:
                sock.connect(str(socket_path))
                path=str(scene).encode();sock.sendall(struct.pack('=I',len(path))+path)
                counts=sock.recv(8); ngeom,nvert=struct.unpack('=II',counts)
                self.assertEqual(ngeom,model.ngeom)
                self.assertEqual(nvert,len(self.playback.vertices[0]))
                previous_frame=None; previous_payload=None
                def exchange(t):
                    nonlocal previous_frame, previous_payload
                    data.time=t;mujoco.mj_forward(model,data)
                    transforms=np.concatenate([data.geom_xpos,data.geom_xmat.reshape(-1,9)],axis=1).astype('float64')
                    sock.sendall(struct.pack('=d',t)+transforms.tobytes())
                    frame=struct.unpack('=I',sock.recv(4))[0]
                    if frame == previous_frame: return previous_payload
                    wanted=(211+6*nvert)*4; chunks=[]
                    while wanted:
                        part=sock.recv(wanted)
                        if not part: raise EOFError('Worker disconnected')
                        chunks.append(part);wanted-=len(part)
                    previous_frame=frame
                    previous_payload=np.frombuffer(b''.join(chunks),dtype=np.float32)
                    return previous_payload
                first=exchange(0); same_frame=exchange(.01); next_frame=exchange(.1); reset=exchange(0)
                np.testing.assert_array_equal(first,same_frame)
                np.testing.assert_allclose(first,reset)
                self.assertFalse(np.allclose(first[:3],next_frame[:3]))
                np.testing.assert_allclose(first[211:211+3*nvert].reshape(-1,3),self.playback.vertices[0])
            records=[json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual([r['frame'] for r in records],[0,1])
            for record in records:
                self.assertEqual(len(record['regions']),14)
                self.assertGreater(record['regions']['head']['distance'],0)
        finally:
            proc.wait(timeout=10)
            stdout, stderr = proc.communicate()
            if proc.returncode: raise RuntimeError(stderr.decode())

    def test_robot_mesh_world_transform(self):
        import mujoco
        import fcl
        xml='<mujoco><worldbody><body name="Kinova_link" pos="3 0 0"><geom type="box" size=".1 .1 .1" group="2"/></body></worldbody></mujoco>'
        m=mujoco.MjModel.from_xml_string(xml); d=mujoco.MjData(m); mujoco.mj_forward(m,d)
        robots,ids=robot_geometry(m,'Kinova_')
        for i in ids: robots[str(i)].setTransform(fcl.Transform(d.geom_xmat[i].reshape(3,3),d.geom_xpos[i]))
        engine=SurfaceDistance(self.playback); engine.update(0)
        self.assertGreater(engine.query(robots)['head']['distance'],0)
        with self.assertRaises(ValueError): robot_geometry(m,'missing')

if __name__=='__main__': unittest.main()
