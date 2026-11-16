import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import manipulation_common as m


class ManipulationContracts(unittest.TestCase):
    def test_deterministic_randomness(self):
        self.assertEqual(m.bits(0,0,0),m.bits(0,0,0))
        values = [m.uniform(w,b,50) for w in range(8) for b in range(20)]
        self.assertEqual(len(set(values)),len(values))
        self.assertTrue(all(0 <= x < 1 for x in values))
        self.assertTrue(all(abs(.25*(2*x-1)) <= .25 for x in values))

    def test_trajectory(self):
        rows = m.trajectory()
        self.assertEqual(len(rows),2150)
        self.assertEqual(rows[0][:7],(*m.HOVER,0.))
        self.assertEqual(rows[0][7:],(0.,)*7)
        offset = 0
        for i, duration in enumerate(m.DURATIONS):
            self.assertEqual(rows[offset][:7],(*m.POSES[i],m.GRIPS[i]))
            self.assertTrue(all(math.isfinite(x) for row in rows[offset:offset+duration] for x in row))
            offset += duration
        self.assertEqual(rows[-1][:7],(*m.HOVER,0.))

    def test_shapes_and_resets_are_diverse(self):
        specs = [m.object_spec(w) for w in range(100)]
        self.assertEqual({kind for kind,_ in specs},{0,1,2})
        self.assertEqual(len(set(specs)),100)
        self.assertTrue(all(.036 <= x < .044 for _,size in specs for x in size))
        for w in range(100):
            self.assertGreater(m.reset_period(w),m.CYCLE+m.bits(w,0,31)%101)

    def test_prepare_preserves_mimic_and_collisions_and_licenses(self):
        xml = b'<robot><link name="base"><visual/><collision><geometry><capsule radius=".032" length=".34"/></geometry></collision></link><joint name="f"><limit velocity="2"/><mimic joint="d" multiplier="-1"/></joint></robot>'
        def read(command,**kwargs):
            if 'rev-parse' in command:
                return 'revision\n'
            return xml if 'urdf' in command[-1] else b'BSD license'
        with tempfile.TemporaryDirectory() as temp, patch.object(subprocess,'check_output',side_effect=read):
            manifest = m.prepare('/unused',temp)
            root = ET.parse(Path(temp)/'robotiq.urdf').getroot()
            self.assertFalse(root.findall('.//visual'))
            self.assertEqual(len(root.findall('.//collision')),1)
            capsule = root.find('.//capsule')
            self.assertEqual(capsule.attrib['length'],'.34')
            self.assertEqual(capsule.attrib['height'],'.34')
            self.assertEqual(root.find('.//mimic').attrib['multiplier'],'-1')
            self.assertEqual(root.find('.//limit').attrib['velocity'],'2')
            self.assertEqual((Path(temp)/'kinova-LICENSE').read_bytes(),b'BSD license')
            self.assertEqual(manifest['cycle_blocks'],len(m.trajectory()))

    @unittest.skipUnless(os.environ.get('MANIPULATION_DRIVER'),'CPU binary not configured')
    def test_cpp_python_workload_parity(self):
        run = subprocess.run([os.environ['MANIPULATION_DRIVER'],'--dump-workload','--worlds','16','--steps','30000'],text=True,capture_output=True,check=True)
        data = json.loads(run.stdout.split('WORKLOAD ')[1])
        self.assertEqual(data['objects'],[[kind,*size] for kind,size in (m.object_spec(w) for w in range(16))])
        expected = sum(.25*(2*m.uniform(w,b,50)-1) for w in range(16) for b in range(3000))
        self.assertEqual(data['force_checksum'],expected)
        self.assertEqual(data['resets'],sum(2999//m.reset_period(w) for w in range(16)))
        bad = subprocess.run([os.environ['MANIPULATION_DRIVER'],'--dump-workload','--threads','0'],capture_output=True,text=True)
        self.assertNotEqual(bad.returncode,0)

    def test_cli_rejects_invalid_counts(self):
        root = Path(__file__).resolve().parents[1]
        for args in (['--steps','11'],['--worlds','1,1'],['--repeats','0'],['--threads','0']):
            result = subprocess.run([sys.executable,'-B',str(root/'manipulation.py'),*args],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)

    @unittest.skipUnless(os.environ.get('MANIPULATION_CPU'),'Set MANIPULATION_CPU for licensed simulation smoke test')
    def test_cpu_full_cycles(self):
        assets = os.environ['MANIPULATION_ASSETS']
        result = subprocess.run([os.environ['MANIPULATION_CPU'],'--assets',assets,'--worlds','6','--steps','30000','--repeats','1'],text=True,capture_output=True,check=True)
        data = json.loads(next(s[7:] for s in result.stdout.splitlines() if s.startswith('RESULT ')))
        self.assertTrue(data['quality_pass'])
        self.assertGreater(data['object_contact_samples'],0)
        self.assertGreater(data['bilateral_grasp_samples'],0)
        self.assertGreater(data['lifted_samples'],0)
        self.assertEqual(data['resets'],6)
        self.assertEqual(data['threads'],6)
        self.assertEqual(data['requested_threads'],30)
        self.assertLess(data['max_mimic_error_rad'],1e-7)

    @unittest.skipUnless(os.environ.get('MANIPULATION_GPU_SMOKE'),'Set MANIPULATION_GPU_SMOKE for CUDA overflow regression')
    def test_gpu_rejects_truncated_contacts(self):
        self.assertTrue(os.environ.get('MANIPULATION_ASSETS'),'Set MANIPULATION_ASSETS to prepared assets')
        script = """
import os
import types
from pathlib import Path
from unittest.mock import patch
import newton_manipulation as gpu
original = gpu.SolverMuJoCo
def limited_solver(*args, **kwargs):
    kwargs['nconmax'] = 1
    return original(*args, **kwargs)
args = types.SimpleNamespace(assets=Path(os.environ['MANIPULATION_ASSETS']),worlds=1,steps=6000,repeats=1)
with patch.object(gpu,'SolverMuJoCo',side_effect=limited_solver):
    gpu.run(args)
"""
        result = subprocess.run([sys.executable,'-B','-c',script],cwd=Path(__file__).resolve().parents[1],text=True,capture_output=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('failed validation',result.stderr)
        data = json.loads(next(s[7:] for s in result.stdout.splitlines() if s.startswith('RESULT ')))
        self.assertFalse(data['quality_pass'])
        self.assertNotEqual(data['overflow_bitmask'],0)

    @unittest.skipUnless(os.environ.get('MANIPULATION_CPU'),'Set MANIPULATION_CPU for licensed parallel parity test')
    def test_parallel_worlds_match_serial_diagnostics(self):
        records = []
        for threads in (1, 30):
            command = [os.environ['MANIPULATION_CPU'],'--assets',os.environ['MANIPULATION_ASSETS'],
                       '--worlds','31','--threads',str(threads),'--steps','30000','--repeats','2']
            result = subprocess.run(command,text=True,capture_output=True,check=True)
            records.append(json.loads(next(s[7:] for s in result.stdout.splitlines() if s.startswith('RESULT '))))
        self.assertEqual(records[1]['threads'],30)
        for field in ('force_checksum','resets','object_contact_samples','bilateral_grasp_samples',
                      'lifted_samples','max_mimic_error_rad','quality_pass','finite'):
            self.assertEqual(records[0][field],records[1][field],field)
        for record in records:
            seconds = sorted(record['seconds'])
            self.assertAlmostEqual(record['aggregate_fps'],31*30000/((seconds[0]+seconds[1])/2))
