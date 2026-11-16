"""Deterministic Robotiq workload shared by both backends; no simulator imports."""
import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

SEED = 20261117
DT = .001
DECIMATION = 10
HOVER = (.38344342446, 2.53791743213, -2.94081775313, 0., .80445012192, .38344342446)
GRASP = (.38344342446, 2.77440756619, -1.84325682091, 0., 1.66552092008, .38344342446)
DEST_HOVER = (-.42634014811, *HOVER[1:5], -.42634014811)
DEST_GRASP = (-.42634014811, *GRASP[1:5], -.42634014811)
POSES = (HOVER, HOVER, GRASP, GRASP, HOVER, DEST_HOVER, DEST_GRASP, DEST_GRASP, DEST_HOVER, HOVER, HOVER)
GRIPS = (0., 0., 0., .75, .75, .75, .75, 0., 0., 0., 0.)
DURATIONS = (100, 250, 150, 250, 350, 250, 150, 250, 350, 50)
CYCLE = sum(DURATIONS)
ARM = tuple(f'kinova_joint_{i}' for i in range(1, 7))
DRIVER = 'robotiq_85_left_knuckle_joint'
FOLLOWERS = ('robotiq_85_right_knuckle_joint', 'robotiq_85_left_inner_knuckle_joint',
             'robotiq_85_right_inner_knuckle_joint', 'robotiq_85_left_finger_tip_joint',
             'robotiq_85_right_finger_tip_joint')
SIGNS = (-1., 1., -1., -1., 1.)
KP = (2000., 3000., 2500., 600., 600., 300., 10.)
KD = (60., 80., 70., 20., 20., 12., .2)


def bits(world, block, channel):
    x = (SEED ^ (world * 0x9e3779b9) ^ (block * 0x85ebca6b) ^ (channel * 0xc2b2ae35)) & 0xffffffff
    x = ((x ^ (x >> 16)) * 0x7feb352d) & 0xffffffff
    x = ((x ^ (x >> 15)) * 0x846ca68b) & 0xffffffff
    return x ^ (x >> 16)


def uniform(world, block, channel):
    return (bits(world, block, channel) & 0xffffff) / 16777216.


def object_spec(world):
    kind = bits(world, 0, 20) % 3
    size = tuple(.036 + .008 * uniform(world, 0, 21 + i) for i in range(3))
    return kind, size


def reset_period(world):
    return CYCLE + 150 + bits(world, 0, 30) % 301


def trajectory():
    rows = []
    for i, duration in enumerate(DURATIONS):
        for block in range(duration):
            s = block / duration
            blend = s**3 * (10 + s * (-15 + 6*s))
            rate = 30*s*s*(1-s)**2 / (duration * DT * DECIMATION)
            start, end = (*POSES[i], GRIPS[i]), (*POSES[i+1], GRIPS[i+1])
            rows.append(tuple(a + blend*(b-a) for a, b in zip(start, end)) +
                        tuple(rate*(b-a) for a, b in zip(start, end)))
    return rows


def prepare(sdk, output):
    """Extract only public, pinned robot XML and both BSD licenses, without checkout writes."""
    sdk, output = Path(sdk), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    def read(name):
        return subprocess.check_output(['git', '-C', str(sdk), 'show', f'HEAD:{name}'])
    source = read('rsc/robotiq_2f85/kinova_robotiq.urdf')
    root = ET.fromstring(source)
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
    if root.findall('.//collision/geometry/mesh'):
        raise ValueError('Expected primitive collision shapes; refuse to drop collision meshes')
    for capsule in root.findall('.//collision/geometry/capsule'):
        length = capsule.get('length')
        height = capsule.get('height')
        if length is None or (height is not None and float(height) != float(length)):
            raise ValueError('Expected an unambiguous capsule length')
        capsule.set('height', length)
    (output / 'robotiq.urdf').write_bytes(ET.tostring(root))
    for name in ('robotiq_2f85', 'kinova'):
        (output / f'{name}-LICENSE').write_bytes(read(f'rsc/{name}/LICENSE'))
    rows = trajectory()
    (output / 'trajectory.txt').write_text('\n'.join(' '.join(format(x, '.17g') for x in r) for r in rows) + '\n')
    manifest = dict(seed=SEED, dt=DT, decimation=DECIMATION, cycle_blocks=CYCLE,
                    source_sha256=hashlib.sha256(source).hexdigest(),
                    prepared_urdf_sha256=hashlib.sha256((output / 'robotiq.urdf').read_bytes()).hexdigest(),
                    trajectory_sha256=hashlib.sha256((output / 'trajectory.txt').read_bytes()).hexdigest(),
                    sdk_revision=subprocess.check_output(['git', '-C', str(sdk), 'rev-parse', 'HEAD'], text=True).strip())
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    return manifest
