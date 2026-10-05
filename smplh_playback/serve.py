"""Local synchronous mc_mujoco bridge. Never advances its own playback clock."""
import argparse
import json
import socket
import struct
from pathlib import Path
import numpy as np
from .runtime import Playback, SurfaceDistance, robot_geometry


def receive(sock,n):
    chunks=[]
    while n:
        chunk=sock.recv(n)
        if not chunk: raise EOFError('Simulator disconnected')
        chunks.append(chunk); n-=len(chunk)
    return b''.join(chunks)


def main():
    import mujoco
    import fcl
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cache',required=True); p.add_argument('--socket',required=True)
    p.add_argument('--robot-prefix',required=True,help='Compiled Kinova body name prefix')
    p.add_argument('--robot-group',type=int,default=2,help='2: Menagerie visual surface; 3: collision geometry comparison')
    p.add_argument('--loop',action='store_true'); p.add_argument('--log',required=True)
    args=p.parse_args(); playback=Playback(args.cache,args.loop); distance=SurfaceDistance(playback)
    path=Path(args.socket)
    if path.exists(): raise FileExistsError(f'Remove stale socket explicitly: {path}')
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    try:
        server.bind(str(path)); path.chmod(0o600); server.listen(1)
        print(f'Ready: {path}',flush=True)
        conn,_=server.accept()
        with conn, open(args.log,'w') as log:
            size=struct.unpack('=I',receive(conn,4))[0]
            if size > 4096: raise ValueError('Scene path too long')
            model=mujoco.MjModel.from_binary_path(receive(conn,size).decode())
            robots,ids=robot_geometry(model,args.robot_prefix,args.robot_group)
            conn.sendall(struct.pack('=II',model.ngeom,len(playback.vertices[0])))
            while True:
                try: time=struct.unpack('=d',receive(conn,8))[0]
                except EOFError: break
                transforms=np.frombuffer(receive(conn,model.ngeom*12*8),dtype=np.float64).reshape(-1,12)
                index=playback.frame(time); distance.update(index)
                for i in ids:
                    robots[str(i)].setTransform(fcl.Transform(transforms[i,3:].reshape(3,3),transforms[i,:3]))
                results=distance.query(robots)
                record=dict(time=time,frame=index,regions=results)
                log.write(json.dumps(record,allow_nan=False)+'\n'); log.flush()
                v=np.asarray(playback.vertices[index],dtype=np.float64)
                # Mesh normals are in the fixed world-aligned visual geom frame.
                normals=np.zeros_like(v); triangles=v[playback.faces]
                fn=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
                for corner in range(3): np.add.at(normals,playback.faces[:,corner],fn)
                norms=np.linalg.norm(normals,axis=1); normals/=np.maximum(norms[:,None],1e-15)
                values=np.concatenate([playback.qpos[index],v.ravel(),normals.ravel()])
                conn.sendall(values.astype('float64').tobytes())
    finally:
        server.close()
        if path.exists(): path.unlink()

if __name__=='__main__': main()
