"""Validate and upgrade a legacy SMPL-H cache in place or into a new directory."""
import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from .native_cache import write_native_cache


def convert(source, output):
    source, output = Path(source), Path(output)
    metadata = json.loads((source / 'metadata.json').read_text())
    vertices = np.load(source / 'vertices.npy', mmap_mode='r')
    qpos = np.load(source / 'qpos.npy', mmap_mode='r')
    joints = np.load(source / 'joints.npy', mmap_mode='r')
    faces = np.load(source / 'faces.npy', mmap_mode='r')
    with np.load(source / 'regions.npz') as archive:
        regions = {name: archive[name] for name in archive.files}
    if vertices.ndim != 3 or vertices.shape[2] != 3 or not len(vertices):
        raise ValueError('vertices.npy must have shape (frames, vertices, 3)')
    frames, vertex_count, _ = vertices.shape
    if qpos.shape != (frames, 211) or joints.shape != (frames, 52, 3):
        raise ValueError('qpos.npy/joints.npy dimensions do not match the 52-joint SMPL-H layout')
    if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces):
        raise ValueError('faces.npy must have shape (faces, 3)')
    if faces.min() < 0 or faces.max() >= vertex_count:
        raise ValueError('face vertex index is outside vertices.npy')
    fps = float(metadata['fps'])
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError('cache FPS must be finite and positive')
    if not regions:
        raise ValueError('cache contains no named regions')
    for name, ids in regions.items():
        if not name or ids.ndim != 1 or not len(ids) or ids.min() < 0 or ids.max() >= len(faces):
            raise ValueError(f'invalid or empty region: {name}')
    if output.resolve() != source.resolve():
        output.mkdir(parents=True, exist_ok=True)
        for filename in ('vertices.npy', 'qpos.npy', 'joints.npy', 'faces.npy', 'regions.npz'):
            shutil.copy2(source / filename, output / filename)
        for filename in ('human.xml', 'surface.obj', 'smplh.yaml'):
            if (source / filename).exists():
                shutil.copy2(source / filename, output / filename)
        if (output / 'human.xml').exists():
            tree = ET.parse(output / 'human.xml')
            for mesh in tree.getroot().findall('./asset/mesh'):
                if mesh.get('name') == 'smplh_surface':
                    mesh.set('file', str((output / 'surface.obj').resolve()))
            tree.write(output / 'human.xml')
        if (output / 'smplh.yaml').exists():
            (output / 'smplh.yaml').write_text(
                'xmlModelPath: ' + json.dumps(str((output / 'human.xml').resolve())) + '\n')
    metadata.update(cache_format=1, frames=frames, vertices=vertex_count, faces=len(faces),
                    regions=list(regions), region_faces={name: len(ids) for name, ids in regions.items()})
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    write_native_cache(output)
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cache', help='Existing cache directory')
    parser.add_argument('--output', help='Destination; defaults to updating metadata in place')
    args = parser.parse_args()
    result = convert(args.cache, args.output or args.cache)
    print(f"Validated cache format {result['cache_format']}: {result['frames']} frames, "
          f"{result['vertices']} vertices, {result['faces']} faces")


if __name__ == '__main__':
    main()
