"""Syntactic inventory; each site may serve several public checks."""
from pathlib import Path
import json
import re
import subprocess

record = Path(__file__).resolve().parent
base = json.loads((record / 'baseline.json').read_text())['production_commit']
tokens = ['PointSpatialIndex::new', 'NativePointSpatialIndex::new', 'CopperSpatialIndex::new', 'NativeRegionSpatialIndex::new', 'NativeCircleSpatialIndex::new']
sites = {}
for name in ['base', 'candidate']:
    files = subprocess.check_output(['git', 'ls-tree', '-r', '--name-only', base, 'crates/hyperdrc/src'], text=True).splitlines() if name == 'base' else [str(p) for p in Path('crates/hyperdrc/src').rglob('*.rs')]
    result = {token: [] for token in tokens}
    for path in sorted(files):
        if not path.endswith('.rs') or any('regression' in part or part in ['tests', 'test_support.rs'] or part.endswith(('_tests.rs', '_test.rs')) for part in Path(path).parts):
            continue
        text = subprocess.check_output(['git', 'show', f'{base}:{path}'], text=True) if name == 'base' else Path(path).read_text()
        text = re.split(r'#\[cfg\(test\)\]\s*mod \w+\s*\{', text, maxsplit=1)[0]
        for token in tokens:
            for match in re.finditer(r'(?<![A-Za-z0-9_])' + re.escape(token), text):
                result[token].append({'path': path, 'line': text[:match.start()].count('\n') + 1})
    sites[name] = result
counts = {name: {token: len(rows) for token, rows in values.items()} for name, values in sites.items()}
assert counts['base'] == dict(zip(tokens, [0, 23, 43, 2, 0]))
assert counts['candidate'] == dict(zip(tokens, [0, 25, 40, 3, 2]))
(record / 'constructor-inventory.json').write_text(json.dumps(dict(
    scope='Syntactic constructor-site inventory: source files excluding regression paths, tests directories and test-module filenames (but retaining production testpoint modules), before their first inline cfg(test) module. Test-only imports and out-of-line module declarations do not truncate production code. Not a behavioral audit of all scalar/geometry consumers. Shared point constructors can serve several public checks; the native circle constructor is shared by existing drill scheduling and IPC probe spacing.',
    base_revision=base, sites=sites, counts=counts), indent=2) + '\n')
print(json.dumps(counts))
