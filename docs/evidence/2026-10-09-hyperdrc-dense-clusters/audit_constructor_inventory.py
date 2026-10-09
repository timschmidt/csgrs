"""Syntactic inventory; each site may serve several public checks."""
from pathlib import Path
import json
import re
import subprocess

record = Path(__file__).resolve().parent
base = json.loads((record / 'baseline.json').read_text())['production_commit']
tokens = ['PointSpatialIndex::new', 'NativePointSpatialIndex::new', 'CopperSpatialIndex::new', 'NativeRegionSpatialIndex::new']
sites = {}
for name in ['base', 'candidate']:
    files = subprocess.check_output(['git', 'ls-tree', '-r', '--name-only', base, 'crates/hyperdrc/src'], text=True).splitlines() if name == 'base' else [str(p) for p in Path('crates/hyperdrc/src').rglob('*.rs')]
    result = {token: [] for token in tokens}
    for path in sorted(files):
        if not path.endswith('.rs') or any(part in Path(path).stem for part in ['test', 'regression']):
            continue
        text = subprocess.check_output(['git', 'show', f'{base}:{path}'], text=True) if name == 'base' else Path(path).read_text()
        text = re.split(r'#\[cfg\(test\)\]\s*mod \w+\s*\{', text, maxsplit=1)[0]
        for token in tokens:
            for match in re.finditer(r'(?<![A-Za-z0-9_])' + re.escape(token), text):
                result[token].append({'path': path, 'line': text[:match.start()].count('\n') + 1})
    sites[name] = result
counts = {name: {token: len(rows) for token, rows in values.items()} for name, values in sites.items()}
assert counts['base'] == dict(zip(tokens, [0, 19, 48, 0]))
assert counts['candidate'] == dict(zip(tokens, [0, 23, 43, 2]))
(record / 'constructor-inventory.json').write_text(json.dumps(dict(
    scope='Syntactic constructor-site inventory: source files excluding test/regression filenames, before their first inline cfg(test) module. Test-only imports and out-of-line module declarations do not truncate production code. Not a behavioral audit of all scalar/geometry consumers. Shared point constructors can serve several public checks; the dense pitch constructor serves all four dense checks.',
    base_revision=base, sites=sites, counts=counts), indent=2) + '\n')
print(json.dumps(counts))
