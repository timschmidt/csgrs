"""Close the scalar unit while preserving earlier trials and the open workflow."""
from pathlib import Path
import gzip,hashlib,json,re,subprocess
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
MEMO={}


def sha(p):
    p=Path(p);st=p.stat();key=(st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)
    if key not in MEMO:
        h=hashlib.sha256()
        with p.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
        MEMO[key]=h.hexdigest()
    return MEMO[key]


def span(raw):
    start=re.search(r'"current_work"\s*:\s*',raw).end()
    value,count=json.JSONDecoder().raw_decode(raw[start:])
    return start,start+count,value


assert not (E/'scalar-closure-v1.json').exists()
a=json.loads((E/'scalar-assessment-v1.json').read_text())
d=json.loads((E/'scalar-decision-v1.json').read_text())
assert d['status']=='kept' and not a['strict_drilled_body_complete']
confirmation=json.loads((E/d['confirmation_evidence']).read_text())
assert confirmation['earlier_assessment_sha256']==sha(E/'scalar-assessment-v1.json')
assert confirmation['totals']['candidate']=={'certified':491520,'unknown':0}
assert (E/d['acceptance_record']).is_file()
assert all(sha(ROOT/p)==digest for p,digest in a['implementation_sha256'].items())
assert all(sha(ROOT/p)==digest for p,digest in a['pending_native_sha256'].items())
previous_path=ROOT/'docs/evidence/2026-10-15-section-proof-reuse/closure-v1.json'
previous=json.loads(previous_path.read_text())
assert previous['pending_native_sha256']==a['pending_native_sha256']
protected=dict(previous['preserved_artifacts_sha256'])
assert all(sha(ROOT/p)==digest for p,digest in previous['evidence_files_sha256'].items())
base=json.loads((E/'baseline-closure-v1.json').read_text())
assert all(sha(ROOT/p)==digest for p,digest in base['evidence_sha256'].items())
protected.update(base['artifacts_sha256'])
protected.update(json.loads((E/'tmp-consolidation-v1.json').read_text())['protected_artifacts_sha256'])
for result_path in sorted(E.glob('*/result.json')):
    folder=result_path.parent
    protocol=json.loads((folder/'protocol.json').read_text())
    result=json.loads(result_path.read_text())
    assert protocol['runner_sha256'] in [sha(E/'validate_v1.py'),sha(E/'validate_v2.py')]
    protected.update(protocol['preserved_prior_executables_sha256'])
    for row in result['commands']:
        assert row['sources_unchanged']
        for suffix in ['stdout','stderr','resources']:
            assert sha(folder/(row['name']+'.'+suffix))==row[suffix+'_sha256']
        protected.update(row['binaries_sha256'])
for root in [ROOT/'target/strict-drilled-workflows',Path('/tmp/csgrs-strict-drilled-workflows/cost-v1'),Path('/tmp/csgrs-strict-drilled-workflows/cost-v2'),Path('/tmp/csgrs-strict-drilled-workflows/affine-baseline-v1'),Path('/run/user/1000/csgrs-strict-drilled-workflows/qualification-v2')]:
    for p in root.rglob('*'):
        if p.is_file():protected[str(p)]=sha(p)
print('Verifying closed evidence and compiler artifacts.',flush=True)
assert all(sha(ROOT/p)==digest for p,digest in protected.items())
links=0
for name,row in json.loads(gzip.decompress((E/'linked-compiler-duplicates-v1.json.gz').read_bytes())).items():
    p=Path(name);origin=Path(row['retained_original'])
    assert sha(p)==sha(origin)==row['sha256'] and p.stat().st_ino==origin.stat().st_ino
    links+=1
ledger=ROOT/'docs/properties-status.json';raw=ledger.read_text();before=sha(ledger)
reference=subprocess.check_output(['git','show','9025de62:docs/properties-status.json'],cwd=ROOT,text=True)
start,end,current=span(raw);rstart,rend,_=span(reference)
assert (raw[:start],raw[end:])==(reference[:rstart],reference[rend:])
current['scalar_product_cancellation']={
 'status':'kept: opposite identical symbolic products construct certified exact zero',
 'baseline_commit':'f1092edb1',
 'implementation_commit':'775af8774',
 'evidence':'docs/evidence/2026-10-15-strict-drilled-workflows/scalar-assessment-v1.json',
 'decision':'docs/evidence/2026-10-15-strict-drilled-workflows/scalar-decision-v1.json',
 'acceptance_record':'docs/evidence/2026-10-15-strict-drilled-workflows/scalar-acceptance.md',
 'next_scalar_frontier':'docs/evidence/2026-10-15-strict-drilled-workflows/affine-baseline-v1/frontier.stdout',
 'closure':'docs/evidence/2026-10-15-strict-drilled-workflows/scalar-closure-v1.json',
 'validation':a['tests_by_configuration'],
 'scope':a['domain'],
 'cost':d['cost'],
 'workflow_frontier':a['workflow_frontier'],
 'limitations':a['limitations'],
 'next_step':'Extend the scalar proof of the captured affine coplanarity identity under a declared resource contract, preserving original 16/32-segment fixtures, all certified controls and genuine undecided/approximate outcomes. This scalar unit does not deliver strict drilling.',
 'full_plan_complete':False,
}
current['updated_on']='2026-10-15';current['plan_complete']=False
ledger.write_text(raw[:start]+json.dumps(current,indent=2).replace('\n','\n  ')+raw[end:])
updated=ledger.read_text();ustart,uend,_=span(updated)
assert (updated[:ustart],updated[uend:])==(reference[:rstart],reference[rend:])
x,y=json.loads(updated),json.loads(reference);x.pop('current_work');y.pop('current_work');assert x==y
files={str(p.relative_to(ROOT)):sha(p) for p in sorted(E.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
record={'assessment_sha256':sha(E/'scalar-assessment-v1.json'),'decision_sha256':sha(E/'scalar-decision-v1.json'),'previous_closure_sha256':sha(previous_path),'closure_runner_sha256':sha(Path(__file__)),'evidence_files_sha256':files,'preserved_artifacts_sha256':protected,'pending_native_sha256':a['pending_native_sha256'],'linked_paths_verified':links,'implementation_sha256':a['implementation_sha256'],'ledger_before_sha256':before,'ledger_after_sha256':sha(ledger),'only_current_work_changed_in_ledger':True,'strict_drilled_body_complete':False,'full_plan_complete':False}
(E/'scalar-closure-v1.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'protected_artifacts':len(protected),'evidence_files_including_closure':len(files)+1,'pending_native_unchanged':35,'full_plan_complete':False}),flush=True)
