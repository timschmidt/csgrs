"""Expose frozen external libraries without older workspace-crate versions."""
from pathlib import Path
import hashlib, json, os, sys

CORE = Path('/run/user/1000/csgrs-shared-scalar-proofs/scalar-and-drill-v1/release/deps')
LOCAL = ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypersolve', 'hypertri',
         'hypercurve', 'hypermesh', 'hyperphysics', 'csgrs']
out = Path(sys.argv[1]); out.mkdir(exist_ok=False)
records = {}
for path in sorted(CORE.iterdir()):
    if path.suffix not in {'.rlib', '.rmeta', '.so'}: continue
    if any(path.name.startswith('lib'+name+'-') or path.stem == 'lib'+name for name in LOCAL): continue
    assert path.is_file() and not path.is_symlink()
    os.symlink(path, out/path.name)
    records[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
(out/'external-libraries.json').write_text(json.dumps(records, indent=2)+'\n')
print(json.dumps({'read_only_external_aliases': len(records)}))
