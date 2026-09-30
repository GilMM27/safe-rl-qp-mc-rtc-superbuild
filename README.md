# safe-rl-qp-mc-rtc-superbuild

Acc-CBF-QP was introduced in:

> **Safe Execution of RL Policies via Acceleration-based CBF-QP Constraint Enforcement for Real-World Robotic Deployments**
> Bastien Muraccioli, Alice Cariou, Pierre-Alexandre Leziart, Mathieu Celerier, Arnaud Demont, Gentiane Venture, Mehdi Benallegue
> IROS 2026 — [Paper](https://hal.science/hal-05362571) · [Project page](https://safe-rl-qp.github.io/)

Part of the Acc-CBF-QP ecosystem: [paper implementation](https://github.com/safe-rl-qp/mc-safe-rl-qp) · superbuild (this repo) · [controller template](https://github.com/bastien-muraccioli/new-rl-qp-controller) · [community controllers](https://github.com/safe-rl-qp/awesome-safe-rl-qp)

[![CI](https://github.com/safe-rl-qp/safe-rl-qp-mc-rtc-superbuild/actions/workflows/ubuntu24-install-test.yml/badge.svg)](https://github.com/safe-rl-qp/safe-rl-qp-mc-rtc-superbuild/actions/workflows/ubuntu24-install-test.yml)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420)](https://github.com/safe-rl-qp/safe-rl-qp-mc-rtc-superbuild#installation)
[![Project Page](https://img.shields.io/badge/Project_Page-Visit-blue.svg)](https://safe-rl-qp.github.io/)
[![PDF](https://img.shields.io/badge/PDF-arXiv-b31b1b.svg)](https://arxiv.org/abs/2607.14488)
[![Award](https://img.shields.io/badge/IEEE%20IES-SYPA%20Award-gold)](https://2026.ieee-iros.org/attend/ies-sypa/)
[![Watch the video](https://img.shields.io/badge/YouTube-IROS%202026%20Introduction-red?logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=MPlcEFUSASg)

## Contents

- [Overview](#overview)
- [Installation](#installation)
  - [Fork the superbuild](#fork-the-superbuild)
  - [Installing the requirements (bootstrapping)](#installing-the-requirements-bootstrapping)
  - [Git setup](#git-setup)
  - [Build](#build)
  - [Config superbuild (adding robots and options)](#config-superbuild-adding-robots-and-options)
  - [Bashrc](#bashrc)
- [Running a controller](#running-a-controller)
- [Adding your own RL-QP controller](#adding-your-own-rl-qp-controller)
  - [Fork the template](#fork-the-template)
  - [Add your controller to the superbuild](#add-your-controller-to-the-superbuild)
- [mc_rtc robots](#mc_rtc-robots)
- [Contributing and issues](#contributing-and-issues)

## Overview

This project is based on [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild): it will clone, update, build, and install all of the dependencies needed to run Acc-CBF-QP.

At the end of the installation you'll be able to run an example controller for the Unitree H1 with a walking policy in Mujoco. We also give the details needed to create your own controller for your own robot and policy.

You can check the [mc-rtc-superbuild README](https://github.com/mc-rtc/mc-rtc-superbuild) for additional background, but you don't need to read it to run the controller example.

This project was tested on Ubuntu 24.04. We do not officially support other operating systems at the moment.

## Installation

### Fork the superbuild

We recommend having your own superbuild by forking this one — as you add your own controllers and extensions, it's much easier to manage with your own fork rather than working directly off of ours.

Create a workspace folder in your home directory and clone your fork into it:

```bash
mkdir ~/workspace
cd ~/workspace
git clone git@github.com:{GIT_USERNAME}/safe-rl-qp-mc-rtc-superbuild.git
```

### Installing the requirements (bootstrapping)

You can install the requirements by running our bootstrap script:

```bash
cd ~/workspace/
./safe-rl-qp-mc-rtc-superbuild/utils/bootstrap-linux.sh
```

### Git setup

Make sure you've configured `git` first:

```sh
git config --global user.name "Full Name"
git config --global user.email "your.email@provider.com"
```

### Build

Run the superbuild from the terminal, or use VS Code's "CMake Tools" extension to select your desired build preset.

By default, the presets will:
- clone all projects into `~/workspace/src`
- build all projects into `~/workspace/build/`
- install all projects into `~/workspace/install`

```bash
cd ~/workspace/safe-rl-qp-mc-rtc-superbuild
# Build all projects
cmake --preset relwithdebinfo
cmake --build --preset relwithdebinfo
```

### Config superbuild (adding robots and options)

To add robots or modify the superbuild options, after your first build you can run:

```bash
cd ~/workspace/build/superbuild
# Configure the superbuild
ccmake .
```

Inside the menu, use your keyboard's arrow keys to navigate and Enter to edit an entry. To run the controller example, you'll need to set the `WITH_H1` option to `ON`.

Once done, press `[c]` to Configure, and once configuration is finished, press `[g]` to Generate and update the superbuild. When that's done, press `[q]` to quit.

You can now rebuild the superbuild to install the H1 robot module:

```bash
cd ~/workspace/safe-rl-qp-mc-rtc-superbuild
# Build all projects
cmake --build --preset relwithdebinfo
```

### Bashrc

At the end of the build, the superbuild will ask you to add the following line to your `.bashrc`:

```bash
source /home/$USERNAME/workspace/install/setup_mc_rtc.sh
```

This file is generated after the first configuration of the superbuild (the step above).

You can also add the following aliases to your `.bashrc` to simplify using the framework:

```bash
# Run the superbuild
alias mc_build='cd ~/workspace/safe-rl-qp-mc-rtc-superbuild; cmake --build --preset relwithdebinfo'

# Config the superbuild
alias mc_superbuild_config="cd ~/workspace/build/superbuild; ccmake ."

# Automatically update the superbuild and all associated projects via git pull
alias mc_update='cd ~/workspace/build/superbuild; cmake --build . --config RelWithDebInfo --target update'

# Open the mc_rtc rviz interface
alias mc_rviz="ros2 launch mc_rtc_ticker display.launch"

# Configure mc_rtc: robot and controller selection (replace gnome-text-editor with your preferred text editor)
alias mc_config="gnome-text-editor ~/.config/mc_rtc/mc_rtc.yaml &"
```

When you're done editing your `.bashrc`, don't forget to source it or open a fresh terminal.

## Running a controller

Use your new `mc_config` alias to create the `mc_rtc.yaml` config file, which tells mc_rtc which controller to run with which robot:

```bash
mc_config
```

Then add the following:

```yaml
MainRobot: H1               # Robot Name
Enabled: RLController        # Controller Name
Timestep: 0.0025             # Controller timestep
LogPolicy: threaded
```

Save the file, then `cd` into the folder containing the policy to run — `mc_mujoco` needs to be run from there, not from an arbitrary directory — and start the controller in Mujoco:

```bash
cd ~/workspace/src/rl_controller/policy/
mc_mujoco --sync
```

In another terminal, you can run this to access the RViz interface (optional with Mujoco):

```bash
mc_rviz
```

Congrats — you should now see H1 walking! If you have a gamepad plugged into your PC (such as a DS4 controller), you can control the robot with the joystick.

### Running the Kinova Gen3 in MuJoCo

Kinova support includes the ROS 2/Kortex interfaces by default only when enabled. The
superbuild can also install the Kinova Gen3 MuJoCo model from the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie). Enable ROS 2
and Kinova support during configuration:

```bash
cmake --preset relwithdebinfo \
  -DWITH_ROS_SUPPORT=ON \
  -DROS_IS_ROS2=ON \
  -DWITH_Kinova=ON
cmake --build --preset relwithdebinfo
```

The build installs the Kinova Gen3 model as `kinova` for `mc_mujoco`. Select a controller that is
compatible with a 7-DoF arm and use a 1 ms timestep:

```yaml
MainRobot: Kinova
Enabled: YourController
Timestep: 0.001
LogPolicy: threaded
```

After sourcing the installed environment, start `mc_mujoco` from the directory
containing the controller's assets:

```bash
source ~/workspace/install/setup_mc_rtc.sh
cd ~/workspace/src/<your-controller-directory>
mc_mujoco --sync
```

The Kinova module and MuJoCo model are separate pieces: `WITH_Kinova` builds the
`mc_rtc` robot model and Kortex support, while the extension downloads and installs
the Menagerie MJCF and its meshes. This provides simulation; `mc_kortex` remains the
separate interface for a physical Kinova Gen3.

For a stationary first test, this repository also installs the
`KinovaHoldController` plugin. It holds the Kinova Gen3 at its default posture and
does not command a trajectory:

```yaml
MainRobot: Kinova
Enabled: KinovaHoldController
Timestep: 0.001
LogPolicy: threaded
```

The controller is intentionally limited to the seven base Kinova joints. It is a
smoke-test controller, not a manipulation controller.

### Installing the tagged human MuJoCo asset

The optional `WITH_HUMAN_MOCAP` extension installs the Apache-2.0
MS-Human-700 locomotion model from a pinned MuJoCo Menagerie revision. Its
visible mesh geometry is organized into named body-part groups, and the
installed `human_mocap_parts.yaml` file provides the semantic-label metadata
used by the proximity tracker:

```bash
cmake --preset relwithdebinfo \
  -DWITH_ROS_SUPPORT=ON \
  -DROS_IS_ROS2=ON \
  -DWITH_Kinova=ON \
  -DWITH_HUMAN_MOCAP=ON
cmake --build --preset relwithdebinfo
```

After sourcing the installed environment, add the human as an object in
`~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml`:

```yaml
objects:
  human:
    module: human_mocap
    init_pos:
      translation: [1.0, 0.0, 0.0]
      rotation: [0, 0, 0]
```

The `HumanMocapTracker` library is installed with this option. It resolves
configured MuJoCo geometry IDs into semantic groups and uses MuJoCo's geometry
distance query to return the closest human part, distance, and closest points.
The library is deliberately independent of `KinovaHoldController`; a controller
or `mc_mujoco` integration can construct it after loading the model and expose
the result through its datastore.

For prerecorded motion, provide a CSV with one row per frame:
`time,qpos[0],qpos[1],...`. The installed tracker includes
`QposTrajectory::loadCsv`, which validates monotonic timestamps and qpos width,
and `sample`, which linearly interpolates at the simulation time with optional
looping. The resulting vector is intended to be copied into the human model's
MuJoCo `qpos` before `mj_forward`; this keeps playback deterministic and
independent of rendering.

The first tested recording candidate is **CMU subject 14, trial 07
(`14_07`)**, described as “jump up to grab, reach for, tiptoe.” It consists of
the subject skeleton `14.asf` and motion file `14_07.amc`. Download these files
without committing them to the repository:

```bash
./tools/fetch_cmu_mocap.sh
```

The downloaded ASF/AMC data still needs retargeting from the CMU skeleton to
the MS-Human-700 joint layout before it can become a MuJoCo `qpos` CSV. The
source listing is available at:
<http://mocap.cs.cmu.edu/search.php?subjectnumber=14&motion=%25%25%25&maincat=%25&subcat=%25&subtext=yes>.
Review the CMU database terms before redistributing downloaded recordings; this
project only provides the acquisition helper and does not bundle the data.

The installed MS-Human-700 model is the tagged geometry foundation for
prerecorded playback. A motion source must still provide compatible MuJoCo
joint positions; this repository does not silently convert arbitrary BVH/FBX
files or claim that a static model is a mocap trajectory.

## Adding your own RL-QP controller

### Fork the template

Fork [new-rl-qp-controller](https://github.com/bastien-muraccioli/new-rl-qp-controller) and follow its README. You can also check [awesome-safe-rl-qp](https://github.com/safe-rl-qp/awesome-safe-rl-qp) to see other controllers based on the template for more examples.

### Add your controller to the superbuild

In the superbuild's extension folder, add a CMake file to register your controller (the same approach works for adding mc_rtc plugins, interfaces, or any other mc_rtc project):

```bash
cd ~/workspace/safe-rl-qp-mc-rtc-superbuild/extensions
gnome-text-editor {controller name}.cmake
```

```cmake
AddProject({controller name}
  GITHUB {github username}/{controller name}
  GIT_TAG origin/{branch name}
  DEPENDS mc_rtc
)
```

The `GITHUB` command clones the repo over HTTPS; use `GITHUB_PRIVATE` instead if you want to clone over SSH. Check the [mc-rtc-superbuild README](https://github.com/mc-rtc/mc-rtc-superbuild) for more details.

Then rebuild the superbuild:

```bash
mc_build
```

And update your mc_rtc config file with your new controller:

```bash
mc_config
```

## mc_rtc robots

By default, mc_rtc is compatible with many robots, as you can see when configuring the superbuild via `mc_superbuild_config`. If you want to add a custom robot that isn't already in the list, follow this [tutorial](https://jrl.cnrs.fr/mc_rtc/tutorials/advanced/new-robot.html).

In short, each robot in mc_rtc has four modules:

- **Robot module** `mc_{robot name}` — e.g. [mc_h1](https://github.com/isri-aist/mc_h1). Contains the constraint definitions (joint limits, self-collision), joint order, and sensor declarations.
- **Robot URDF** `{robot name}_description` — e.g. [h1_description](https://github.com/isri-aist/h1_description). Installed in `src/catkin_data_ws`; contains the full robot description in the format used by ROS. This is what mc_rtc uses for QP constraints and the dynamic model, and what mc_rviz uses for display.
- **Robot Mujoco description** `{robot name}_mj_description` — e.g. [h1_mj_description](https://github.com/isri-aist/h1_mj_description). Used by Mujoco to display and simulate the robot's physics. Unless intentional, it's important to keep the same robot model in the URDF and the Mujoco XML description.
- **Robot driver interface** `mc_{robot driver name}` — e.g. [mc_unitree2](https://github.com/isri-aist/mc_unitree2). Installs the driver needed to run your mc_rtc controller on the real robot. The exact command differs per robot driver — for H1, running your controller on the real robot requires `MCControlUnitree`. Always check the README of the relevant driver repo, since most require additional information in your mc_rtc config file (`mc_config`), such as the robot's IP address.

The `WITH_H1` option in `mc_superbuild_config` adds all four modules at once — this isn't true for every robot. At minimum you'll need the robot module and the URDF; if the Mujoco description or robot driver are missing, check the [isri-aist](https://github.com/isri-aist) GitHub account and add the missing modules manually in the superbuild's extension folder.

## Contributing and issues

Since this repository is a fork of [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild), if you run into installation issues we recommend addressing them directly on the original repo. Likewise, since the superbuild is just a collection of CMake files, issues with a specific project are best reported directly on that project's repo.

The main useful way to contribute to this repo specifically is by improving this installation guide (this README) if you spot any issues.