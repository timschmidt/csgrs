#!/usr/bin/env python3
"""Exercise child-output interleaving without changing any captured Cargo log."""
import json
from pathlib import Path
import tempfile
import unittest
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase

E = Path(__file__).resolve().parent
RESULT = 'test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s'


class ParseResults(unittest.TestCase):
    def parse(self, stdout):
        with tempfile.TemporaryDirectory(prefix='csgrs-libtest-parser-') as name:
            path = Path(name)
            (path/'stdout.log').write_text(stdout)
            (path/'stderr.log').write_text('Running unittests src/lib.rs (/tmp/deps/hyperreal-example)\n')
            return parse_phase(path)

    def test_parent_and_child_records(self):
        for marker in [RESULT, 'ok'+RESULT, RESULT.replace('result: ', 'result:\n'),
                       'ok'+RESULT.replace('result: ', 'result:\n')]:
            with self.subTest(marker=marker):
                data = self.parse('running 1 test\nrunning 1 test\ntest child ... ok\n'+marker+
                    '\ntest parent ... ok\n'+RESULT+'\n')
                self.assertEqual(data['top_level_passed'],1)
                self.assertEqual(data['child_process_passed'],1)
                self.assertEqual(data['top_level_failed']+data['child_process_failed'],0)

    def test_missing_child_summary_is_rejected(self):
        with self.assertRaises(AssertionError):
            self.parse('running 1 test\nrunning 1 test\n'+RESULT+'\n')

    def test_counter_mismatch_is_rejected(self):
        with self.assertRaises(AssertionError):
            self.parse('running 2 tests\n'+RESULT+'\n')

    def test_failed_child_stays_failed(self):
        child = RESULT.replace('ok. 1 passed; 0 failed;', 'FAILED. 0 passed; 1 failed;')
        data = self.parse('running 1 test\nrunning 1 test\n'+child+'\n'+RESULT+'\n')
        self.assertEqual(data['child_process_failed'],1)
        self.assertEqual(data['child_process_passed'],0)

    def test_four_retained_summaries_are_identical(self):
        for version in [1045,1046,1047,1103]:
            # Retained records name their matching source phase explicitly.
            records = list(E.glob(f'typed-api-prototype-v{version}-test-summary.json'))
            self.assertEqual(len(records),1)
            record = records[0]
            phases = [p for p in E.glob(f'typed-api-prototype-v{version}-*') if p.is_dir() and (p/'stdout.log').exists()]
            self.assertEqual(len(phases),1)
            with self.subTest(phase=phases[0].name):
                self.assertEqual(parse_phase(phases[0]),json.loads(record.read_text()))

    def test_actual_interleaved_stack_log(self):
        data = parse_phase(E/'typed-api-prototype-v1104-stack-tests')
        self.assertEqual(data['top_level_passed'],5811)
        self.assertEqual(data['child_process_passed'],3)
        self.assertEqual(data['top_level_ignored'],33)
        self.assertEqual(data['top_level_failed']+data['child_process_failed'],0)


if __name__ == '__main__':
    unittest.main()
