"""Install the opt-in bridge into the superbuild's external mc_mujoco source."""
import argparse
from pathlib import Path
import shutil


def install(source):
    source=Path(source); here=Path(__file__).parent
    cpp=source/'src/mj_sim.cpp'; header=source/'src/mj_sim_impl.h'
    c=cpp.read_text(); h=header.read_text()
    if 'smplh_bridge_.update' in c:
        if 'SmplhBridge smplh_bridge_' not in h: raise RuntimeError('Incomplete bridge patch')
        shutil.copyfile(here/'SmplhBridge.h',source/'src/SmplhBridge.h'); return
    replacements=[
        ('  mj_forward(model, data);','  mj_forward(model, data);\n  smplh_bridge_.update(model, data);'),
        ('  mj_step(model, data);','  mj_step(model, data);\n  mj_forward(model, data);\n  smplh_bridge_.update(model, data);'),
        ('  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);',
         '''#ifdef USE_UI_ADAPTER
  smplh_bridge_.upload(model, &platform_ui_adapter->mjr_context());
#else
  smplh_bridge_.upload(model, &context);
#endif
  mjv_updateScene(model, data, &options, &pert, &camera, mjCAT_ALL, &scene);''')]
    for before,after in replacements:
        if c.count(before)!=1: raise RuntimeError(f'Unsupported mc_mujoco source: {before}')
        c=c.replace(before,after)
    marker='struct MjSimImpl'
    # Insert instance member after the actual class opening, allowing inheritance.
    import re
    match=re.search(r'(?:class|struct) MjSimImpl\b[^;{]*\{',h)
    if not match: raise RuntimeError('Cannot locate MjSimImpl definition')
    h=h[:match.end()]+'\n  SmplhBridge smplh_bridge_;\n'+h[match.end():]
    h='#include "SmplhBridge.h"\n'+h
    cpp.write_text(c); header.write_text(h)
    shutil.copyfile(here/'SmplhBridge.h',source/'src/SmplhBridge.h')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--source',required=True)
    install(p.parse_args().source)
