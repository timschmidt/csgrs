"""Account for every finding and coverage change in complete fixed-path reports.

Usage: python3 audit_reports.py REPORT_DIRECTORY SOURCE_AUDIT_DIRECTORY
Raw full reports remain untouched. Per-finding matching uses canonical JSON
with every field retained, with no coordinate tolerance or field deletion.
The independent oracle validates source primitives, not every DRC heuristic.
"""
from collections import Counter,defaultdict,deque
from pathlib import Path
import gzip,hashlib,json,sys
record=Path(__file__).resolve().parent
reports,source_audit=map(Path,sys.argv[1:3])
info=lambda data:dict(bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
canonical=lambda value:json.dumps(value,sort_keys=True,separators=(',',':'))
def delta(before,after,path=''):
    if isinstance(before,dict) and isinstance(after,dict):
        return [row for key in sorted(before.keys()|after.keys()) for row in delta(before.get(key),after.get(key),path+'/'+key)]
    if isinstance(before,list) and isinstance(after,list) and len(before)==len(after):
        return [row for i,(a,b) in enumerate(zip(before,after)) for row in delta(a,b,path+'/'+str(i))]
    return [] if canonical(before)==canonical(after) else [dict(path=path,before=before,after=after)]
result={}
for label in ['cparti-fpga','hvp109a']:
    previous=record.parent/f'2026-10-09-hypercurve-retained-circular-area/report-replay/trial-1-{label}.json.gz'
    old_raw=gzip.decompress(previous.read_bytes());new_raw=gzip.decompress((reports/f'trial-1-{label}.json.gz').read_bytes())
    assert new_raw==gzip.decompress((reports/f'trial-2-{label}.json.gz').read_bytes())
    before=json.loads(old_raw);after=json.loads(new_raw)
    assert before['files']==after['files'] and before['inputs']==after['inputs'] and before['waived_count']==after['waived_count']
    old_findings=before['violations'];new_findings=after['violations'];lookup=defaultdict(deque)
    for i,value in enumerate(old_findings):lookup[canonical(value)].append(i)
    stable=[];added=[]
    for i,value in enumerate(new_findings):
        matches=lookup[canonical(value)]
        if matches:stable.append(dict(before=matches.popleft(),after=i))
        else:added.append(i)
    removed=sorted(i for queue in lookup.values() for i in queue)
    assert len(stable)+len(removed)==len(old_findings)==before['violation_count']
    assert len(stable)+len(added)==len(new_findings)==after['violation_count']
    counts=lambda findings:dict(sorted(Counter(v['check'] for v in findings).items()))
    changes=dict(stable=stable,removed=[dict(index=i,finding=old_findings[i]) for i in removed],added=[dict(index=i,finding=new_findings[i]) for i in added])
    data=(json.dumps(changes,sort_keys=True,separators=(',',':'))+'\n').encode();(reports/(label+'-finding-deltas.json.gz')).write_bytes(gzip.compress(data,mtime=0))
    top=delta({k:v for k,v in before.items() if k!='violations'},{k:v for k,v in after.items() if k!='violations'})
    (reports/(label+'-metadata-deltas.json')).write_text(json.dumps(top,indent=2)+'\n')
    source=json.loads((source_audit/'candidate'/f'{label}.json').read_bytes())
    assert source['all_admitted']
    result[label]=dict(previous=str(previous),before=info(old_raw),after=info(new_raw),identical_candidate_complete_bytes=True,files_inputs_waived_count_unchanged=True,before_counts=counts(old_findings),after_counts=counts(new_findings),canonical_identical_findings=len(stable),removed_or_changed=len(removed),added_or_changed=len(added),metadata_delta_count=len(top),complete_deltas=info(data),source_oracle={k:v for k,v in source.items() if not isinstance(v,list)},interpretation='Changed placement, exact source dimensions/ratios, outer-layer alias expansion and native report projection can change geometry, proximity, totals, finding identities and messages. Complete unmatched findings and all metadata deltas are retained for review. The independent source oracle validates primitive source geometry; it is not a separate implementation of every readiness rule.')
(reports/'finding-audit.json').write_text(json.dumps(dict(scope='Complete two-board fixed-path reports, twelve checks per board, unchanged full inputs and retained IPC sidecar; no candidate normalization and no universal DRC or path-independent determinism claim.',boards=result),indent=2)+'\n')
print(json.dumps({label:{k:v for k,v in data.items() if k in ['before_counts','after_counts','canonical_identical_findings','removed_or_changed','added_or_changed','metadata_delta_count']} for label,data in result.items()},indent=2))
