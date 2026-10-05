# SMPL-H / AMASS human playback

This directory adds a playback path with three separate representations:

```
AMASS sequence -> SMPL-H cache -> kinematic MuJoCo human
                              -> SMPL-H triangle regions -> FCL distance log
                              -> optional small MuJoCo spheres for display/debug
```

`mc_mujoco` remains the only simulator. The small Python worker reconstructs
SMPL-H frames and performs triangle-mesh distance queries. The `mujoco` Python
package in `requirements.txt` is its Python binding, pinned to the same 3.5.0
release already downloaded and used by the C++ simulator; it does not build or
run a second simulation.

## External data and Python environment

Obtain an AMASS sequence and the SMPL-H neutral/male/female model from their
respective licensors. Do not add either to this repository. Create a separate
virtual environment and install the worker dependencies:

```sh
python3 -m venv ~/.venvs/smplh-playback
~/.venvs/smplh-playback/bin/pip install -r smplh_playback/requirements.txt
```

For SMPL-H model files, use the path to the corresponding `.pkl` file (for
example `SMPLH_MALE.pkl`) or its containing model directory. AMASS input must
be an `.npz` sequence containing `poses` with 156 values per frame, `trans`,
`betas`, `gender`, and `mocap_framerate`.

If the licensed files are not yet available, create a synthetic fixture to test
the complete scene, bridge, worker, and logging path. It is deliberately not a
human surface and must not be used for distance research:

```sh
python -m smplh_playback.make_smoke_fixture --output /tmp/smplh-smoke-cache
```

Prepare a cache outside the repository. This resolves SMPL-H once and keeps
the high-resolution vertices and faces for every frame:

```sh
SMPLH_MODEL=/secure/models/SMPLH_MALE.pkl \
AMASS_SEQUENCE=/secure/amass/sequence_01.npz \
~/.venvs/smplh-playback/bin/python -m smplh_playback.prepare \
  --output /tmp/amass-cache --translation 1.2 0 0
```

Set `--fps` to change playback rate, `--rotation RX RY RZ` and
`--translation X Y Z` to align AMASS coordinates with the Kinova scene, and
`--regions regions.json` to replace the joint-influence body-region mapping.
The generated `smplh.yaml` exposes `human.xml` to `mc_mujoco` as an object.

## Build and run

Configure the superbuild with `-DWITH_SMPLH_PLAYBACK=ON` and rebuild. This
applies a small, idempotent patch to the external `mc_mujoco` source that is
only active when `SMPLH_SOCKET` is set. No controller changes are needed.

Create `~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml` (or use your local
mc_mujoco configuration directory) with the generated object module:

```yaml
objects:
  human:
    module: smplh
    init_pos: [0, 0, 0, 1, 0, 0, 0]
```

Copy `/tmp/amass-cache/smplh.yaml` to the same configuration directory as
`smplh.yaml`. Start the worker before `mc_mujoco`:

```sh
export PYTHONPATH=$PWD
export SMPLH_SOCKET=/tmp/smplh-mujoco.sock
export SMPLH_SCENE=/tmp/kinova-smplh-scene.mjb
~/.venvs/smplh-playback/bin/python -m smplh_playback.serve \
  --cache /tmp/amass-cache --socket "$SMPLH_SOCKET" \
  --robot-prefix kinova_ --robot-group 2 --loop \
  --log /tmp/kinova-smplh-distances.jsonl

mc_mujoco --sync
```

`kinova_smoke_mc_mujoco.yaml` and `kinova_smoke_mc_rtc.yaml` are minimal
Kinova settings for an isolated smoke run. They are provided so an existing
mc_mujoco object configuration need not be replaced.

`SMPLH_SCENE` must be a writable, local `.mjb` filename. The C++ bridge saves
the compiled combined scene there at startup so the Python binding sees the
same compiled Kinova geometry and frame conventions. `--robot-group 2` measures
the Menagerie visual robot mesh. Run a second worker with `--robot-group 3` to
record a comparison against the MuJoCo collision geoms. This selection affects
only robot geometry: every human distance is always against SMPL-H triangles.

## Playback and measurements

The bridge sends MuJoCo `mjData::time` to the worker. A frame is
`floor(time * fps)`, so simulation reset deterministically restores frame zero;
`--loop` wraps frame indices and the default holds the final frame. The worker
returns the free-root pose and all 51 ball-joint poses to MuJoCo. The human is
kinematic: velocities are reset to zero and its display/debug geoms have no
contact bits.

Robot-to-surface distances are updated every physics step. The articulated
pose, mesh vertices, and normals are transferred only when the AMASS frame
changes; normals are computed once for that frame. This avoids repeatedly
transferring the full mesh at simulation rates above the mocap FPS. Distance
JSONL is sampled every ten physics steps by default; set `--log-every 1` to log
every step.

Regions are assigned by the largest average SMPL-H linear-blend-skinning
influence over each triangle. The defaults provide head, torso, upper arm,
forearm, hand, thigh, lower leg, and foot on both sides. `regions.npz` stores
the exact face IDs, and a custom JSON object mapping region names to SMPL-H
joint indexes changes the policy without changing code.

For each region and selected Kinova geom, FCL queries a triangle BVH. The JSONL
record retains the minimum distance, human point, robot point, unit direction
`(robot_point - human_point) / distance`, selected robot geom, and the closest
triangle normal when available. The worker rebuilds a region BVH only when the
mocap frame changes. Intersections are recorded as zero clearance; FCL does not
provide reliable closest points or a separation direction in that case.

The SMPL-H surface is a reconstructed body estimate, with clothing, soft tissue,
and model-fitting errors absent. Treat its distances as a research measurement,
then validate any ISO/TS 15066 region policy and margins for the intended
system.
