"""Deterministic surface playback and FCL body-region closest points."""
import json
from pathlib import Path
import numpy as np


class Playback:
    def __init__(self, cache, loop=False):
        cache = Path(cache)
        self.meta = json.loads((cache/'metadata.json').read_text())
        self.vertices = np.load(cache/'vertices.npy', mmap_mode='r')
        self.qpos = np.load(cache/'qpos.npy', mmap_mode='r')
        self.faces = np.load(cache/'faces.npy')
        with np.load(cache/'regions.npz') as regions:
            self.regions = {k: regions[k] for k in regions.files}
        self.loop = loop
        if len(self.vertices) != len(self.qpos) or not len(self.qpos):
            raise ValueError('Invalid cache frame count')
        if self.qpos.ndim != 2 or self.qpos.shape[1] != 211:
            raise ValueError('Expected one free-root and 51 ball-joint poses per frame')
        if self.vertices.ndim != 3 or self.vertices.shape[0] != len(self.qpos) or self.vertices.shape[2] != 3:
            raise ValueError('Expected vertices with shape (frames, vertices, 3)')
        if self.faces.ndim != 2 or self.faces.shape[1] != 3:
            raise ValueError('Expected triangular faces with shape (faces, 3)')
        if self.meta.get('cache_format', 1) != 1:
            raise ValueError(f"Unsupported SMPL-H cache format {self.meta['cache_format']}")
        if self.meta.get('frames', len(self.qpos)) != len(self.qpos):
            raise ValueError('Cache metadata frame count mismatch')
        if self.meta.get('vertices', self.vertices.shape[1]) != self.vertices.shape[1]:
            raise ValueError('Cache metadata vertex count mismatch')
        if self.meta.get('faces', len(self.faces)) != len(self.faces):
            raise ValueError('Cache metadata face count mismatch')
        if not np.isfinite(self.meta['fps']) or self.meta['fps'] <= 0:
            raise ValueError('Invalid cache FPS')
        for ids in self.regions.values():
            if not len(ids) or np.any(ids < 0) or np.any(ids >= len(self.faces)):
                raise ValueError('Invalid or empty region')
        if self.faces.size and (self.faces.min() < 0 or self.faces.max() >= self.vertices.shape[1]):
            raise ValueError('Face vertex index is outside the cached surface')

    def frame(self, time):
        if not np.isfinite(time) or time < 0: raise ValueError('Invalid simulation time')
        index = int(np.floor(time*self.meta['fps'] + 1e-9))
        return index % len(self.qpos) if self.loop else min(index,len(self.qpos)-1)


def bvh(vertices, faces):
    import fcl
    mesh = fcl.BVHModel()
    mesh.beginModel(len(faces),len(vertices))
    mesh.addSubModel(np.asarray(vertices,dtype=np.float64),np.asarray(faces,dtype=np.int32))
    mesh.endModel()
    return mesh


class SurfaceDistance:
    """Triangle surface to robot geometry; human simulation geoms are never read."""
    def __init__(self, playback):
        self.playback = playback
        self.index = None
        self.objects = {}

    def update(self, index):
        import fcl
        if index == self.index: return
        # Python FCL does not expose BVH refit. Rebuild only on a new mocap frame.
        v = self.playback.vertices[index]
        self.objects = {name:fcl.CollisionObject(bvh(v,self.playback.faces[ids]))
                        for name,ids in self.playback.regions.items()}
        self.index = index

    def query(self, robots):
        import fcl
        results = {}
        for name, human in self.objects.items():
            best = None
            for robot_name,robot in robots.items():
                # Collisions have zero unsigned surface clearance. Nearest points are undefined there.
                collision = fcl.CollisionResult()
                if fcl.collide(human,robot,fcl.CollisionRequest(),collision):
                    candidate = dict(distance=0.,human_point=None,robot_point=None,direction=None,
                                     robot_geometry=robot_name,human_normal=None,intersecting=True)
                else:
                    result = fcl.DistanceResult()
                    d = fcl.distance(human,robot,fcl.DistanceRequest(enable_nearest_points=True),result)
                    if d < 0 or not np.isfinite(d): raise RuntimeError('FCL distance query failed')
                    ph,pr = map(np.asarray,result.nearest_points)
                    # FCL 0.7 returns primitive nearest points in local coordinates.
                    if robot.getNodeType() != 5:  # 5 = BVH_MODEL
                        pr = robot.getRotation() @ pr + robot.getTranslation()
                    normal = None
                    if 0 <= result.b1 < len(self.playback.regions[name]):
                        triangle = self.playback.vertices[self.index, self.playback.faces[self.playback.regions[name][result.b1]]]
                        cross = np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0])
                        length = np.linalg.norm(cross)
                        if length > 1e-12: normal = (cross/length).tolist()
                    delta = pr-ph; norm = np.linalg.norm(delta)
                    candidate = dict(distance=float(d),human_point=ph.tolist(),robot_point=pr.tolist(),
                                     direction=(delta/norm).tolist() if norm > 1e-12 else None,
                                     robot_geometry=robot_name,human_normal=normal,intersecting=False)
                if best is None or candidate['distance'] < best['distance']: best=candidate
            if best is None: raise ValueError('No robot measurement geometry selected')
            results[name]=best
        return results


def robot_geometry(model, prefix, group=2):
    """Use compiled MuJoCo mesh triangles and analytic primitives in their geom frames."""
    import fcl
    import mujoco
    robots = {}; ids=[]
    for i in range(model.ngeom):
        body_name = mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[i])) or ''
        if not prefix or not body_name.startswith(prefix) or int(model.geom_group[i]) != group: continue
        t=int(model.geom_type[i]); s=model.geom_size[i]
        if t == mujoco.mjtGeom.mjGEOM_MESH:
            mid=int(model.geom_dataid[i]); va=int(model.mesh_vertadr[mid]); vn=int(model.mesh_vertnum[mid])
            fa=int(model.mesh_faceadr[mid]); fn=int(model.mesh_facenum[mid])
            geom=bvh(model.mesh_vert[va:va+vn],model.mesh_face[fa:fa+fn])
        elif t == mujoco.mjtGeom.mjGEOM_SPHERE: geom=fcl.Sphere(s[0])
        elif t == mujoco.mjtGeom.mjGEOM_CAPSULE: geom=fcl.Capsule(s[0],2*s[1])
        elif t == mujoco.mjtGeom.mjGEOM_CYLINDER: geom=fcl.Cylinder(s[0],2*s[1])
        elif t == mujoco.mjtGeom.mjGEOM_BOX: geom=fcl.Box(*(2*s))
        else: raise ValueError(f'Unsupported robot geometry type {t} on {body_name}')
        robots[str(i)]=fcl.CollisionObject(geom); ids.append(i)
    if not ids: raise ValueError(f'No robot geometry for body prefix {prefix!r}')
    return robots,ids
