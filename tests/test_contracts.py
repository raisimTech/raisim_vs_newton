import json
import math
import os
from pathlib import Path
import subprocess
import sys
import statistics
import shutil
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from benchmark_common import positive_price, prompt_prices
from benchmark import worker_count, result_from_log, package_versions, gpu_description
from scripts.compare_reference import compare
from scripts.prepare_assets import rotation, mv, multiply


class BenchmarkContracts(unittest.TestCase):
    def test_changed_sdk_is_rejected_before_preparing_assets(self):
        from benchmark import main
        with tempfile.TemporaryDirectory() as folder:
            build = Path(folder)
            (build/'benchmark_paths_Release.json').write_text(json.dumps(
                {'sdk': folder, 'sdk_revision': 'configured', 'sdk_dirty': False}))
            for revision, dirty in [('changed', ''), ('configured', ' M CMakeLists.txt')]:
                with mock.patch.object(sys, 'argv', ['benchmark.py', '--build', folder,
                        '--cpu-price', '100', '--gpu-price', '200']), \
                     mock.patch('benchmark.subprocess.check_output', side_effect=[revision, dirty]), \
                     mock.patch('benchmark.prepare') as prepare, \
                     mock.patch('sys.stderr'):
                    with self.assertRaises(SystemExit) as error:
                        main()
                    self.assertEqual(error.exception.code, 2)
                    prepare.assert_not_called()

    def test_prices_reject_invalid_values(self):
        for text in ('0','-1','nan','inf','-inf','1 USD',''):
            with self.subTest(text=text), self.assertRaises(ValueError):
                positive_price(text)
        self.assertEqual(positive_price('339.99'),339.99)

    def test_prompt_retry_and_independent_prices(self):
        entries=iter(['nan','0','339.99','899.99']); messages=[]
        self.assertEqual(prompt_prices(input_fn=lambda _:next(entries),output_fn=messages.append),(339.99,899.99))
        self.assertEqual(len(messages),2)
        self.assertEqual(prompt_prices(200,1000,input_fn=lambda _:self.fail('unexpected prompt')),(200,1000))

    def test_unattended_missing_prices_error(self):
        def eof(_):raise EOFError
        with self.assertRaisesRegex(ValueError,'unattended'):
            prompt_prices(input_fn=eof)

    def test_reference_worker_counts(self):
        with mock.patch('benchmark.os.cpu_count',return_value=20):
            self.assertEqual(worker_count(1),1)
            self.assertEqual(worker_count(64),min(16,len(os.sched_getaffinity(0))) if hasattr(os,'sched_getaffinity') else 16)
            self.assertEqual(worker_count(256,12),12)

    def test_cpu_only_metadata_needs_no_newton_packages(self):
        import importlib.metadata
        with mock.patch('benchmark.importlib.metadata.version', side_effect=importlib.metadata.PackageNotFoundError):
            self.assertTrue(all(v is None for v in package_versions().values()))

    def test_empty_comparison_is_not_success(self):
        with self.assertRaisesRegex(ValueError,'No measurements'):
            compare([],{'records':[]},5.)

    def test_duplicate_case_cannot_conceal_missing_case(self):
        reference = {'records':[{'model':'anymal','backend':'raisim','worlds':n} for n in (1,64)]}
        with self.assertRaisesRegex(ValueError,'Duplicate'):
            compare([reference['records'][0],reference['records'][0]],reference,5.)

    def test_cpu_only_metadata_handles_unavailable_gpu_driver(self):
        with mock.patch('benchmark.shutil.which',return_value='nvidia-smi'), mock.patch(
                'benchmark.subprocess.run',return_value=subprocess.CompletedProcess([],1,'','unavailable')):
            self.assertIsNone(gpu_description())

    def test_log_parser_keeps_exact_action_checksum(self):
        data=result_from_log('ACTION_CHECKSUM 0.30976193770766258\nRESULT {"last_action_checksum":0.309761937707663}\n')
        self.assertEqual(data['last_action_checksum'],0.30976193770766258)

    def test_model_transform_math(self):
        q=[math.sqrt(.5),0,math.sqrt(.5),0]
        vector=mv(rotation(q),[1,0,0])
        for actual,expected in zip(vector,[0,0,-1]):self.assertAlmostEqual(actual,expected)
        product=multiply(q,[q[0],-q[1],-q[2],-q[3]])
        for actual,expected in zip(product,[1,0,0,0]):self.assertAlmostEqual(actual,expected)

    def test_comparison_does_not_hide_timing_changes(self):
        old={'model':'anymal','backend':'raisim','worlds':1,'aggregate_fps':100,'seconds':[1,1,1],
             'steps':100,'threads':1,'action_seed':20261115,'action_decimation':10,
             'last_action_checksum':2.,'mean_contacts_per_world':3.}
        new={**old,'aggregate_fps':90,'fps_per_usd':9.}
        result=compare([new],{'records':[old]},5.)
        self.assertTrue(result['all_protocols_match'])
        self.assertFalse(result['all_within_tolerance'])
        self.assertFalse(result['all_timings_identical'])
        self.assertAlmostEqual(result['rows'][0]['change_percent'],-10.)

    def test_recorded_protocol_checks_action_scale(self):
        old={'model':'anymal','backend':'newton','worlds':1,'aggregate_fps':100.,'seconds':[1.,1.,1.],
             'steps':100,'action_seed':20261115,'action_decimation':10,'last_action_checksum':2.,
             'mean_contacts_per_world':3.,'action_scale':1.,'dt':.002}
        new={**old,'fps_per_usd':10.,'action_scale':0.}
        self.assertFalse(compare([new],{'records':[old]},5.)['all_protocols_match'])

    def test_public_dependency_boundary(self):
        for directory in ('src','cmake','scripts'):
            for path in (ROOT/directory).rglob('*'):
                if path.is_file() and '__pycache__' not in path.parts:
                    text=path.read_text()
                    for private in ('benchmarkCommon.hpp','include_internal','/home/jemin/','/tmp/raisim-'):
                        self.assertNotIn(private,text,str(path))
        self.assertIn('https://github.com/raisimTech/raisim2Lib.git',(ROOT/'cmake/PublicSdk.cmake').read_text())

    @unittest.skipUnless(shutil.which('git') and shutil.which('cmake'), 'CMake/Git unavailable')
    def test_sdk_subdirectory_is_not_mislabeled_with_parent_commit(self):
        with tempfile.TemporaryDirectory() as folder:
            base=Path(folder)
            subprocess.run(['git','init','-q',str(base)],check=True)
            sdk=base/'not-a-checkout';sdk.mkdir()
            command=['cmake','-S',str(ROOT),'-B',str(base/'configure'),
                     '-DBENCHMARK_SDK_SOURCE_DIR='+str(sdk)]
            if sys.platform.startswith('linux') and shutil.which('clang++-20'):
                command.append('-DCMAKE_CXX_COMPILER=clang++-20')
            result=subprocess.run(command,text=True,capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Git checkout root',result.stdout+result.stderr)

    @unittest.skipUnless(os.environ.get('BENCHMARK_CPU'),'CPU binary not configured')
    def test_cpu_action_golden_values_and_invalid_steps(self):
        executable=os.environ['BENCHMARK_CPU']
        args=[executable,'--dump-actions','--worlds','7','--steps','100','--model','strandbeest']
        run=subprocess.run(args,text=True,capture_output=True,check=True)
        actual=json.loads(run.stdout.split('ACTIONS ')[1])
        self.assertEqual(actual,[4.966187953948975,4.766554832458496,7.696200370788574,
                                 7.5439453125,5.626091003417969,6.969207286834717,4.8845109939575195])
        run=subprocess.run([executable,'--steps','17'],text=True,capture_output=True)
        self.assertNotEqual(run.returncode,0)
        self.assertNotIn('How much',run.stdout)

    @unittest.skipUnless(os.environ.get('RAISIM_ACTIVATION_KEY') and os.environ.get('BENCHMARK_CPU'),
                         'Licensed integration test requires RAISIM_ACTIVATION_KEY')
    def test_even_trial_count_uses_statistical_median(self):
        assets=Path(os.environ.get('BENCHMARK_ASSETS',ROOT/'build/assets'))
        if not (assets/'anymal.urdf').is_file():self.skipTest('Prepare model assets first')
        command=[os.environ['BENCHMARK_CPU'],'--assets',str(assets),'--model','anymal',
                 '--steps','100','--repeats','2','--cpu-price','200','--gpu-price','1000']
        process=subprocess.run(command,text=True,capture_output=True,check=True)
        record=result_from_log(process.stdout)
        self.assertAlmostEqual(record['aggregate_fps'],100/statistics.median(record['seconds']),places=6)
        self.assertAlmostEqual(record['fps_per_usd'],record['aggregate_fps']/200,places=8)

    @unittest.skipUnless(os.environ.get('PRICE_PROBE'),'Price probe not configured')
    def test_native_price_prompt(self):
        run=subprocess.run([os.environ['PRICE_PROBE']],input='nan\n0\n339.99\n899.99\n',text=True,capture_output=True)
        self.assertEqual(run.returncode,0,run.stderr)
        self.assertIn('How much did your CPU cost (USD)?',run.stdout)
        self.assertIn('How much did your GPU cost (USD)?',run.stdout)
        self.assertIn('PRICES 339.99000000000001 899.99000000000001',run.stdout)
        eof=subprocess.run([os.environ['PRICE_PROBE']],input='',text=True,capture_output=True)
        self.assertNotEqual(eof.returncode,0)


if __name__=='__main__':unittest.main()
