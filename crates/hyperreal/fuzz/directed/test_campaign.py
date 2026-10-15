"""Exercise journal replay with synthetic probe data, including rehashed tampering."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from campaign import digest, run, verify


class CampaignTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = Path(cls.temporary.name)
        binary = cls.root / 'synthetic-probe'
        # These tests exercise campaign bookkeeping, not arithmetic accuracy.
        binary.write_text('''#!/usr/bin/env python3
import json,sys
from pathlib import Path
sys.path.insert(0, ''' + repr(str(Path(__file__).resolve().parent)) + ''')
from test_evidence import CASES,response
if sys.argv[1] == '--list':
    print(json.dumps({'queries': CASES, 'representations': [
        {'name': 'Bucket'+str(i), 'actual_class': 'Bucket'+str(i)} for i in range(22)]}))
else:
    payload = json.loads(Path(sys.argv[1]).read_text())
    result = response()
    result['recipe']['input'] = payload
    for i, query in enumerate(result['queries']):
        score = 1 + payload['genome'][i]
        query['score_ns'] = score
        for sample in query['samples']:
            sample['elapsed_ns'] = score
    print(json.dumps(result))
''')
        binary.chmod(0o700)
        cls.original = cls.root / 'complete'
        args = argparse.Namespace(binary=binary, output=cls.original, seed=916,
                                  rounds=2, population=4, parents=2, timeout=10)
        with contextlib.redirect_stdout(io.StringIO()):
            run(args)

    def setUp(self):
        self.output = self.root / self.id().rsplit('.', 1)[-1]
        shutil.copytree(self.original, self.output)

    def rewrite(self, name, edit):
        path = self.output / name
        value = json.loads(path.read_text())
        edit(value)
        path.write_text(json.dumps(value) + '\n')

    def rehash_and_reject(self, message):
        # Recompute artifact hashes to make the semantic check decisive.
        self.rewrite('result.json', lambda result: result.update(artifacts_sha256={
            p.name: digest(p) for p in self.output.iterdir() if p.name != 'result.json'}))
        with self.assertRaisesRegex(ValueError, message):
            verify(self.output)

    def test_complete_campaign_replays(self):
        with contextlib.redirect_stdout(io.StringIO()):
            verify(self.output)

    def test_missing_bucket_is_rejected(self):
        self.rewrite('round-000-selection.json', lambda record: record['populations'].pop('Bucket0'))
        self.rehash_and_reject('representation bucket')

    def test_missing_parent_selection_is_rejected(self):
        self.rewrite('round-000-selection.json', lambda record: record['selections']['Bucket0'].pop())
        self.rehash_and_reject('parent-selection budget')

    def test_changed_random_stream_is_rejected(self):
        self.rewrite('bucket-seeds.json', lambda seeds: seeds.update(Bucket0=0))
        self.rehash_and_reject('representation seeds')

    def test_changed_plan_is_rejected(self):
        def edit(plan):
            genome = plan['Bucket0'][0]['input']['genome']
            genome[0] ^= 1
        self.rewrite('round-001-planned.json', edit)
        self.rehash_and_reject('planned population')

    def test_failed_probe_cannot_be_ignored(self):
        name = next(self.output.glob('round-000-Bucket0-*.process.json')).name
        self.rewrite(name, lambda process: process.update(exit_code=1))
        self.rehash_and_reject('exit successfully')

    def test_incomplete_journal_is_rejected(self):
        (self.output / 'completed-probes.jsonl').write_text('')
        self.rehash_and_reject('completed-probe journal')

    def test_selected_input_inventory_is_checked(self):
        self.rewrite('selected-inputs.json', lambda selected: selected.update(Bucket0=[]))
        self.rehash_and_reject('retained input inventory')


if __name__ == '__main__':
    unittest.main()
