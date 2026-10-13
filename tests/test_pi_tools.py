import csv
import importlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from tools import pi_experiment,step_limit_scan,out_of_loop
from tools.experiment_plan import verify_config

class PIToolsTests(unittest.TestCase):
    def test_imports_and_dry_run_cannot_open_devices_or_spawn(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.dict('sys.modules',{'paramiko':None,'serial':None,'pyvisa':None}), \
                mock.patch('subprocess.run',side_effect=AssertionError('offline dry run must not spawn')):
            for module in ('pi_experiment','step_limit_scan','out_of_loop','scope_86100c','serial_board','board_session','analyze_out_of_loop'):
                importlib.import_module('tools.'+module)
            result=pi_experiment.run('missing-binary',Path(d)/'preview',dry_run=True)
            self.assertEqual(result['data_origin'],'simulation')
            scan=step_limit_scan.run_scan('missing-binary',Path(d)/'scan',[5000,5500],dry_run=True)
            self.assertEqual(scan['status'],'dry_run')
            for item in scan['runs']:
                config=(Path(d)/'scan'/f"pi-{item['limit']}"/'config.txt').read_text(encoding='utf-8')
                self.assertEqual(verify_config(config,limit=item['limit'])['MAX_PHYSICAL_DELTA'],str(item['limit']))
    def test_scan_rejects_invalid_or_duplicate_limits(self):
        for limits in ([],[0],[8001],[5000,5000]):
            with self.subTest(limits=limits), tempfile.TemporaryDirectory() as d, self.assertRaises(ValueError):
                step_limit_scan.run_scan('missing',Path(d)/'scan',limits,dry_run=True)
    def test_failed_run_stops_scan_and_preserves_manifest(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.object(step_limit_scan,'run',side_effect=RuntimeError('synthetic send failure')) as run:
            with self.assertRaises(RuntimeError):step_limit_scan.run_scan('missing',Path(d)/'scan',[5000,5500])
            self.assertEqual(run.call_count,1)
            self.assertEqual(json.loads((Path(d)/'scan'/'scan_manifest.json').read_text())['status'],'stopped_after_failure')
    def test_summary_uses_in_loop_error_and_rejects_nonfinite(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'synthetic.csv'
            p.write_text('V_PHS_mV,Send_accepted,State\n3,1,TRACK\n4,0,FAULT\n',encoding='utf-8')
            r=pi_experiment.summarize(p)
            self.assertAlmostEqual(r['rms_error_mv'],(12.5)**.5)
            self.assertEqual(r['fault_rows'],1)
            self.assertEqual(r['measurement_scope'],'in_loop')
            p.write_text('V_PHS_mV,Send_accepted,State\nnan,0,FAULT\n',encoding='utf-8')
            with self.assertRaises(ValueError):pi_experiment.summarize(p)
    def test_hardware_options_require_explicit_connections_and_conversion(self):
        with self.assertRaisesRegex(ValueError,'source-address'):out_of_loop.parse_args(['--execute'])
        with self.assertRaisesRegex(ValueError,'hz-per-mv'):
            out_of_loop.parse_args(['--execute','--source-address','SIM','--serial-port','TEST_CONSOLE'])
        with self.assertRaisesRegex(ValueError,'2 GHz'):out_of_loop.parse_args(['--dry-run','--base-freq-hz','1e9'])
        with self.assertRaisesRegex(ValueError,'positive'):out_of_loop.parse_args(['--dry-run','--hz-per-mv','nan'])

if __name__=='__main__':unittest.main()
