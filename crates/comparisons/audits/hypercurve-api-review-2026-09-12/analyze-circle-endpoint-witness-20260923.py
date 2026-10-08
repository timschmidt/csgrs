from pathlib import Path
from collections import Counter
from functools import lru_cache
import hashlib, json, time
import sympy as sp

audit = Path(__file__).resolve().parent
path = audit/'circle-endpoint-witness-20260923-scalars.jsonl'
rows = [json.loads(line) for line in path.read_text().splitlines()]
operations = Counter()

def natural(digits):
    return sum(int(digit) << (32*index) for index,digit in enumerate(digits))

def rational(value):
    return sp.Rational(int(value['sign'])*natural(value['numerator']), natural(value['denominator']))

def key(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'))

@lru_cache(None)
def computable(encoded):
    value = json.loads(encoded)
    assert set(value) == {'internal'}, set(value)
    node = value['internal']
    if node == 'One':
        operations['One'] += 1
        return sp.Integer(1)
    assert isinstance(node,dict) and len(node)==1
    operation, data = next(iter(node.items()))
    operations[operation] += 1
    child = lambda item: computable(key(item))
    if operation=='Int': return sp.Integer(int(data[0])*natural(data[1]))
    if operation=='Ratio': return rational(data)
    if operation=='Add': return child(data[0])+child(data[1])
    if operation=='Multiply': return child(data[0])*child(data[1])
    if operation=='Inverse': return 1/child(data)
    if operation=='Negate': return -child(data)
    if operation=='Square': return child(data)**2
    if operation=='Sqrt': return sp.sqrt(child(data))
    if operation=='Offset': return child(data[0])*sp.Integer(2)**int(data[1])
    if operation=='LinearCombination3':
        return sum(child(c)*rational(v) for c,v in zip(data['coefficients'],data['values']))
    raise ValueError(f'Unsupported captured operator {operation}')

def real(value):
    factor = rational(value['rational'])
    if value['computable'] is None:
        assert value['class']=='One', value['class']
        return factor
    return factor*computable(key(value['computable']))

start = time.monotonic()
values = {}
summary = []
for row in rows:
    value = real(row['value'])
    values[row['name']] = value
    expression = str(value)
    summary.append(dict(name=row['name'],operations=int(sp.count_ops(value)),expression=expression if len(expression)<1200 else None,approximation=str(sp.N(value,25))))
    print(row['name'], 'ops',sp.count_ops(value),'approximation',sp.N(value,25),flush=True)

equalities = {}
for name in ['difference_x','difference_y']:
    expression = values[name]
    simplified = sp.simplify(expression)
    equalities[name] = dict(simplified=str(simplified),proved_zero=simplified==0)
    print(name,equalities[name],flush=True)

report = dict(input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),sympy_version=sp.__version__,node_operators=dict(operations),values=summary,equalities=equalities,elapsed_seconds=time.monotonic()-start)
(audit/'circle-endpoint-witness-20260923-symbolic-oracle.json').write_text(json.dumps(report,indent=2)+'\n')
