"""Check complete ordered witnesses before interpreting preliminary timing."""
import hashlib
import json
import re
from pathlib import Path
W = Path(__file__).resolve().parent
O = W / 'preflight-comparison-v1'
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def read(p): return json.loads(Path(p).read_text())
records = {}
for label in ['baseline', 'candidate']:
    folder = W / (label + '-validation-v1')
    result = read(folder / 'result.json')
    assert result['complete'] and result['command_failures'] == [] and result['cargo_target_closed']
    rows = read(folder / 'commands.json')
    for row in rows:
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(folder / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
    assert '7 passed; 0 failed' in (folder / 'contracts.stdout').read_text()
    records[label] = {}
    for count in [1,16,128,512,2048]:
        text = (folder / ('cost-' + str(count) + '.stdout')).read_text()
        samples = [json.loads(m) for m in re.findall(r'CIRCLE_CACHE_SAMPLE (\{[^\n]*\})', text)]
        witnesses = [json.loads(m) for m in re.findall(r'CIRCLE_CACHE_ROWS (\[[^\n]*\])', text)]
        assert len(samples) == len(witnesses) == 2
        assert [s['phase'] for s in samples] == ['cold','warm']
        assert all(s['count'] == s['entries'] == count for s in samples)
        assert len(witnesses[0]) == len(witnesses[1]) == count and witnesses[0] == witnesses[1]
        for n, row in enumerate(witnesses[0]):
            assert row[0] == row[2] == str(n*32+3)
            assert {row[1],row[3]} == {'4','-4'}
        records[label][count] = {'samples':samples,'rows':witnesses[0], 'stdout_sha256':sha(folder / ('cost-' + str(count) + '.stdout'))}
comparison = []
for count in [1,16,128,512,2048]:
    a,b = records['baseline'][count],records['candidate'][count]
    assert a['rows']==b['rows']
    for i,phase in enumerate(['cold','warm']):
        comparison.append({'count':count,'phase':phase,'baseline_seconds':a['samples'][i]['seconds'],
            'candidate_seconds':b['samples'][i]['seconds'],'candidate_over_baseline':b['samples'][i]['seconds']/a['samples'][i]['seconds'],
            'ordered_exact_rows_equal':True,'rows_sha256':hashlib.sha256(json.dumps(a['rows'],separators=(',',':')).encode()).hexdigest()})
O.mkdir(exist_ok=False)
(O/'result.json').write_text(json.dumps({'complete':True,'rows':comparison,'all_ordered_exact_rows_equal':True,
    'runner_sha256':sha(__file__),'native_runtime_admitted':False,
    'scope':'One diagnostic sample per phase/count. Tiny rows are timer-scale; no final latency/allocation/size or full-workload admission claim.'},indent=2)+'\n')
print(json.dumps({'complete':True,'all_ordered_exact_rows_equal':True,'rows':comparison}),flush=True)
