"""Accuracy-contract and evidence tests; synthetic timings are not benchmarks."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from campaign import digest
from competitive import BACKENDS, TARGET, artifacts, make_schedule, run, summarize, validate, verify
from evidence import ENCODING


def response(payload, backend, elapsed=9):
    return {'schema': 1, 'status': 'ok', 'backend': backend, 'recipe': {'input': payload},
            'actual_class': payload['bucket'], 'rational_encoding': ENCODING,
            'dispatch_trace_enabled': False, 'debug_assertions': False,
            'maximum_absolute_width': str(TARGET), 'elapsed_ns': elapsed,
            'oracle': {'precision_bits': 1024, 'interval': ['3/4', '5/4'], 'sign': 'Positive'},
            'steps': [{'precision_bits': 256, 'interval': ['1/1', '1/1'], 'width': '0/1'}]}


class ContractTests(unittest.TestCase):
    def setUp(self):
        self.payload = {'bucket': 'One', 'genome': [0] * 8}

    def test_both_backends_have_the_same_width_contract(self):
        for backend in BACKENDS:
            self.assertEqual(validate(response(self.payload, backend), self.payload, backend), 9)

    def test_width_and_precision_steps_cannot_be_relaxed(self):
        for field, value in [('interval', ['1/1', '9/8']), ('width', '1/1'), ('precision_bits', 384)]:
            bad = response(self.payload, 'hyperreal')
            bad['steps'][0][field] = value
            with self.assertRaises(ValueError):
                validate(bad, self.payload, 'hyperreal')
        bad = response(self.payload, 'mpfr')
        bad['steps'].append({'precision_bits': 384, 'interval': ['1/1', '1/1'], 'width': '0/1'})
        with self.assertRaisesRegex(ValueError, 'stop at the required width'):
            validate(bad, self.payload, 'mpfr')

    def test_input_build_and_backend_mismatches_are_rejected(self):
        for field, value in [('actual_class', 'Pi'), ('backend', 'mpfr'), ('debug_assertions', True),
                             ('dispatch_trace_enabled', True), ('maximum_absolute_width', '1/1')]:
            bad = response(self.payload, 'hyperreal'); bad[field] = value
            with self.assertRaises(ValueError):
                validate(bad, self.payload, 'hyperreal')

    def test_schedule_preserves_every_case_and_alternates_backend_order(self):
        inputs = {'a': self.payload, 'b': self.payload}
        schedule = make_schedule(inputs, 11, 29)
        self.assertEqual(len(schedule), 44)
        self.assertEqual(schedule, make_schedule(inputs, 11, 29))
        for key in inputs:
            for sample in range(11):
                pair = [row['backend'] for row in schedule if row['id'] == key and row['sample'] == sample]
                self.assertEqual(set(pair), set(BACKENDS))
                if sample:
                    previous = [row['backend'] for row in schedule if row['id'] == key and row['sample'] == sample - 1]
                    self.assertEqual(pair, previous[::-1])

    def test_statistics_retain_the_slow_outlier_and_all_repetitions(self):
        inputs = {'a': self.payload}
        values = [1] * 5 + [10] * 5 + [10000]
        journal = [{'id': 'a', 'backend': backend, 'sample': i, 'elapsed_ns': value}
                   for backend in BACKENDS for i, value in enumerate(values)]
        result = summarize(inputs, journal, 11)[0]
        for backend in BACKENDS:
            self.assertEqual(result['backends'][backend]['raw_elapsed_ns'], values)
            self.assertEqual(result['backends'][backend]['median_ns'], 10)
            self.assertEqual(result['backends'][backend]['maximum_ns'], 10000)
        with self.assertRaisesRegex(ValueError, 'missing or duplicate'):
            summarize(inputs, journal[:-1], 11)


class ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = Path(cls.temporary.name)
        campaign = cls.root / 'campaign'; campaign.mkdir()
        selected = {f'Bucket{i}': [{'bucket': f'Bucket{i}', 'genome': [0] * 8}] for i in range(22)}
        (campaign / 'selected-inputs.json').write_text(json.dumps(selected))
        (campaign / 'result.json').write_text(json.dumps({'complete': True}))
        binary = cls.root / 'synthetic-binary'; binary.write_bytes(b'not an executed binary')
        cls.original = cls.root / 'complete'

        def synthetic_execute(binary, arguments, prefix, timeout, expected_hash):
            backend, input_path = arguments
            payload = json.loads(Path(input_path).read_text())
            data = response(payload, backend, 1 + len(prefix.name))
            out, err = prefix.with_suffix('.stdout'), prefix.with_suffix('.stderr')
            out.write_text(json.dumps(data)); err.write_bytes(b'')
            record = {'status': 'exited', 'exit_code': 0, 'stdout': out.name, 'stderr': err.name,
                      'stdout_sha256': digest(out), 'stderr_sha256': digest(err)}
            prefix.with_suffix('.process.json').write_text(json.dumps(record))
            return data, record

        args = argparse.Namespace(binary=binary, campaign=campaign, output=cls.original,
                                  samples=3, seed=29, timeout=30)
        with patch('competitive.verify_campaign'), patch('competitive.execute', synthetic_execute), contextlib.redirect_stdout(io.StringIO()):
            run(args)

    def setUp(self):
        self.output = self.root / self.id().rsplit('.', 1)[-1]
        shutil.copytree(self.original, self.output)

    def rewrite(self, name, edit):
        path = self.output / name; value = json.loads(path.read_text())
        edit(value); path.write_text(json.dumps(value))

    def reject_rehashed(self, message):
        updated = artifacts(self.output); updated.pop('result.json')
        self.rewrite('result.json', lambda result: result.update(artifacts_sha256=updated))
        with patch('competitive.verify_campaign'), self.assertRaisesRegex(ValueError, message):
            verify(self.output)

    def test_complete_synthetic_evidence_replays(self):
        with patch('competitive.verify_campaign'), contextlib.redirect_stdout(io.StringIO()):
            verify(self.output)

    def test_omitted_selected_input_is_rejected(self):
        self.rewrite('inputs.json', lambda inputs: inputs.pop(next(iter(inputs))))
        self.reject_rehashed('omitted or changed')

    def test_missing_sample_is_rejected(self):
        path = self.output / 'samples.jsonl'
        path.write_text('\n'.join(path.read_text().splitlines()[:-1]) + '\n')
        self.reject_rehashed('sample budget')

    def test_failed_process_cannot_be_ignored(self):
        name = next(self.output.glob('case-*.process.json')).name
        self.rewrite(name, lambda process: process.update(exit_code=1))
        self.reject_rehashed('exit successfully')

    def test_changed_summary_cannot_drop_an_unfavorable_result(self):
        self.rewrite('summary.json', lambda rows: rows.pop())
        self.reject_rehashed('statistics differ')


if __name__ == '__main__':
    unittest.main()
