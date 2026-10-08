from pathlib import Path
import hashlib,json,shutil

A=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-selected-chamfer-scalar-replay-2026-09-23')
root=Path('/tmp/hyperreal-biquadratic-basis-v1-2026-09-23')
assert not root.exists()
old=json.loads((A/'selected-chamfer-scalar-20260923-probe1-sources.json').read_text())
bindings={name:sha for name,sha in old.items() if name.startswith('hyperreal/')}
for name,sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest()==sha,name
    (root/name).parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source/name,root/name)
for name in ['src/computable/node/quadratic_tower.rs','src/real/arithmetic/quadratic_tower_sign.rs']:
    p=Path('/home/tim/Documents/GitHub/workspace/hyperreal')/name
    shutil.copy2(p,root/'hyperreal'/name)
    bindings['hyperreal/'+name]=hashlib.sha256(p.read_bytes()).hexdigest()
(A/'biquadratic-basis-20260923-v1-sources.json').write_text(json.dumps(bindings,indent=2)+'\n')

# Preserve sharing when turning the exact serialized scalar into a replay.
# Each expression shape has one local binding; no numerical approximation is used.
p=json.loads((A/'selected-chamfer-scalar-20260923-replay.json').read_text())
nodes={}; statements=[]
def big(v): return sum(x<<(32*i) for i,x in enumerate(v))
def number(n):
    assert -(1<<63)<n<(1<<63)
    return f'Real::from({n}_i64)'
def rational(v):
    n=v['sign']*big(v['numerator']);d=big(v['denominator'])
    return f'({number(n)} / {number(d)}).unwrap()'
def visit(v):
    n=v.get('internal',v) if isinstance(v,dict) else v
    key=json.dumps(n,sort_keys=True,separators=(',',':'))
    if key in nodes:return nodes[key]
    if n=='One': expr='Real::one()'
    else:
        op,x=next(iter(n.items()))
        if op=='Ratio':expr=rational(x)
        elif op=='Int':expr=number(x[0]*big(x[1]))
        elif op=='Constant':assert x=='Sqrt3';expr='Real::from(3).sqrt().unwrap()'
        elif op=='Sqrt':expr=f'{visit(x)}.clone().sqrt().unwrap()'
        elif op=='Negate':expr=f'-&{visit(x)}'
        elif op=='Offset':
            v=visit(x[0]);factor=1<<abs(x[1]); expr=f'&{v} * {number(factor)}' if x[1]>=0 else f'(&{v} / {number(factor)}).unwrap()'
        elif op in ['Add','Multiply']:
            left=visit(x[0]);right=visit(x[1]);operator='+' if op=='Add' else '*';expr=f'&{left} {operator} &{right}'
        else:raise ValueError(op)
    name=f'v{len(nodes)}'; nodes[key]=name;statements.append(f'    let {name} = {expr};');return name
value=visit(p['computable'])
code='use hyperreal::{Real, RealSign};\nfn main() {\n'+'\n'.join(statements)+f'\n    let value = &{value} * ({rational(p["rational"])});\n    println!("shared replay tower={{:?}}", value.quadratic_tower_sign());\n    assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Zero));\n}}\n'
(A/'selected-chamfer-scalar-shared-20260923.rs').write_text(code)
print('Prepared',len(bindings),'Hyperreal inputs and',len(nodes),'shared scalar nodes.')
