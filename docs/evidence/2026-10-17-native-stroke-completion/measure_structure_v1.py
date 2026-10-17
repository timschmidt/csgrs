"""Measure retained serialized expression trees without rerunning timed work."""
import hashlib,json,shutil
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];O=W/'structure-metrics-v1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(Path(p).read_text())
def dump(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')
def magnitude_bits(limbs):
    assert isinstance(limbs,list) and all(type(v)==int and 0<=v<2**32 for v in limbs)
    count=len(limbs)
    while count and limbs[count-1]==0:count-=1
    return 0 if not count else 32*(count-1)+limbs[count-1].bit_length()
def metrics(value):
    out={'real_values':0,'serialized_computable_nodes':0,'max_computable_depth':0,
         'max_integer_coefficient_bits':0,'max_rational_numerator_bits':0,'max_rational_denominator_bits':0,
         'max_absolute_binary_offset':0}
    def walk(node,depth=0):
        if isinstance(node,dict):
            if {'class','computable','rational'}<=node.keys():out['real_values']+=1
            if 'internal' in node:
                depth+=1;out['serialized_computable_nodes']+=1
                out['max_computable_depth']=max(out['max_computable_depth'],depth)
            if {'sign','numerator','denominator'}<=node.keys():
                out['max_rational_numerator_bits']=max(out['max_rational_numerator_bits'],magnitude_bits(node['numerator']))
                out['max_rational_denominator_bits']=max(out['max_rational_denominator_bits'],magnitude_bits(node['denominator']))
            if 'Int' in node:
                sign,limbs=node['Int'];assert sign in [-1,0,1]
                out['max_integer_coefficient_bits']=max(out['max_integer_coefficient_bits'],magnitude_bits(limbs))
            if 'Offset' in node:
                child,shift=node['Offset'];assert type(shift)==int
                out['max_absolute_binary_offset']=max(out['max_absolute_binary_offset'],abs(shift))
            for child in node.values():walk(child,depth)
        elif isinstance(node,list):
            for child in node:walk(child,depth)
    walk(value)
    out['compact_selected_json_bytes']=len(json.dumps(value,separators=(',',':')).encode())
    return out
assert magnitude_bits([])==0 and magnitude_bits([0,1])==33 and magnitude_bits([2**32-1])==32
comparison=read(W/'comparison-v1/result.json');assert comparison['complete']
O.mkdir(exist_ok=False);(O/'format-sources').mkdir()
sources=[ROOT/'Cargo.lock',ROOT/'crates/hyperreal/src/rational/arithmetic/representation.rs',
         ROOT/'crates/hyperreal/src/computable/node/representation.rs',
         Path('/home/tim/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6/src/biguint/serde.rs'),
         Path('/home/tim/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6/src/bigint/serde.rs')]
format_pins={}
for i,p in enumerate(sources):
    dest=O/'format-sources'/f'{i}-{p.name}';shutil.copy2(p,dest);assert sha(dest)==sha(p)
    format_pins[str(p)]={'sha256':sha(p),'retained':str(dest.relative_to(O))}
dump(O/'protocol.json',{'runner_sha256':sha(__file__),'comparison_sha256':sha(W/'comparison-v1/result.json'),
    'format_sources':format_pins,
    'scope':'Actual area, actual bounds and native segment scalar outputs for the original and rebuilt regions; independent expected expressions and point-query strings are excluded.',
    'depth':'Maximum nesting of serialized Computable internal fields; rational-only scalars have zero Computable depth.',
    'coefficients':'Magnitude bit lengths of little-endian u32 limbs from the locked num-bigint 0.4.6 format, separately for integer nodes and rational numerator/denominator fields.',
    'limitations':'Serialization expands shared expressions and omits cache/proof state. Counts are serialized trees, not unique live DAG nodes or peak intermediate work. No internal resource bound or total expression-growth claim follows.'})
rows=[]
for sample in comparison['all_samples']:
    path=W/'comparison-v1'/sample['snapshot'];key=path.name.removesuffix('.snapshot.json')
    assert sha(path)==comparison['retained_snapshot_sha256'][key]
    phase=0 if sample['phase']=='fresh_process' else 1
    state=read(path)[phase];regions=[]
    for region in state.get('regions',[]):
        selected={'area':region['area'].get('actual'),
                  'bounds':[entry.get('actual') for entry in region['bounds']] if isinstance(region['bounds'],list) else region['bounds'],
                  'native':region['native']}
        regions.append(metrics(selected))
    rows.append({k:sample[k] for k in ['pair','case','label','cap','variant','phase','certified_completion','snapshot']}|{'regions':regions})
assert len(rows)==240
paired=[]
for before in [r for r in rows if r['variant']=='baseline']:
    after=next(r for r in rows if r['variant']=='candidate' and all(r[k]==before[k] for k in ['pair','case','cap','phase']))
    paired.append({k:before[k] for k in ['pair','case','label','cap','phase']}|{'baseline':before['regions'],'candidate':after['regions'],
        'both_complete':before['certified_completion'] and after['certified_completion'],
        'deltas':[{'region':i,**{k:b[k]-a[k] for k in a}} for i,(a,b) in enumerate(zip(before['regions'],after['regions']))]})
dump(O/'result.json',{'complete':True,'phase_observations':len(rows),'all_observations':rows,'all_paired_rows':paired,
    'source_artifacts_unchanged':all(sha(p)==r['sha256'] for p,r in format_pins.items()),
    'timed_work_reexecuted':False,'unique_live_graph_nodes_measured':False,'peak_intermediate_expression_growth_measured':False})
assert all(sha(p)==r['sha256'] for p,r in format_pins.items())
print(json.dumps({'complete':True,'phase_observations':len(rows),'paired_rows':len(paired)}),flush=True)
