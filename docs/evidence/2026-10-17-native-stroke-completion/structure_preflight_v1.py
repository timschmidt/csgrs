"""Check the retained-tree parser on completed diagnostic snapshots only."""
import ast,hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'structure-preflight-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
source=W/'measure_structure_v1.py';tree=ast.parse(source.read_text())
definitions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['magnitude_bits','metrics']]
assert len(definitions)==2
scope={'json':json};exec(compile(ast.Module(body=definitions,type_ignores=[]),str(source),'exec'),scope)
assert scope['magnitude_bits']([])==0 and scope['magnitude_bits']([0,1])==33
assert scope['metrics']({'internal':{'Add':[{'internal':{'Int':[1,[0,1]]}},{'internal':'One'}]}})['max_computable_depth']==2
O.mkdir(exist_ok=False);rows=[];pins={str(source):sha(source),str(Path(__file__)):sha(__file__)}
for case in [0,2,4]:
    p=W/'cost-preflight-v2'/f'case-{case}.snapshot.json';pins[str(p)]=sha(p)
    phases=json.loads(p.read_text())
    for phase,state in enumerate(phases):
        for index,region in enumerate(state['regions']):
            selected={'area':region['area'].get('actual'),'bounds':[entry.get('actual') for entry in region['bounds']],'native':region['native']}
            rows.append({'case':case,'phase':phase,'region':index,**scope['metrics'](selected)})
assert len(rows)==12
record={'complete':True,'evidence_role':'Parser preflight only; no matched comparison and no timed kernel work executed',
    'input_sha256':pins,'observations':rows,'interpretation_checks':['u32 limb magnitude length','nested Computable depth'],
    'main_sources_modified':False}
(O/'result.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'complete':True,'observations':len(rows),'max_computable_depth':max(r['max_computable_depth'] for r in rows)}),flush=True)
