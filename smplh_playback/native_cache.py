"""Portable little-endian array bundle consumed by the C++ mc_mujoco bridge."""
import json
from pathlib import Path
import struct

import numpy as np

MAGIC = b'SMPLHC1\0'


def write_native_cache(directory):
    directory = Path(directory)
    meta = json.loads((directory / 'metadata.json').read_text())
    vertices = np.load(directory / 'vertices.npy', mmap_mode='r')
    qpos = np.load(directory / 'qpos.npy', mmap_mode='r')
    faces = np.load(directory / 'faces.npy', mmap_mode='r')
    with np.load(directory / 'regions.npz') as data:
        regions = [(name, np.asarray(data[name], dtype='<u4')) for name in data.files]
    frames, nvertices, _ = vertices.shape
    with (directory / 'smplh_cache.bin').open('wb') as stream:
        stream.write(struct.pack('<8sIIIII d', MAGIC, frames, nvertices, len(faces), len(regions), 211,
                                 float(meta['fps'])))
        for name, ids in regions:
            encoded = name.encode('utf-8')
            if len(encoded) > 63:
                raise ValueError(f'region name is too long for native cache: {name}')
            stream.write(encoded + bytes(64 - len(encoded)))
            stream.write(struct.pack('<I', len(ids)))
            stream.write(ids.tobytes())
        np.asarray(qpos, dtype='<f4').tofile(stream)
        np.asarray(vertices, dtype='<f4').tofile(stream)
        np.asarray(faces, dtype='<u4').tofile(stream)
