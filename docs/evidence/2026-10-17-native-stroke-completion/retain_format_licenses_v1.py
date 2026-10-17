"""Retain notices for the locked dependency's serialization-format references."""
import hashlib,json,shutil
from pathlib import Path
W=Path(__file__).resolve().parent;O=W/'format-source-licenses-v1'
SOURCE=Path('/home/tim/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
O.mkdir(exist_ok=False);pins={}
for name in ['LICENSE-MIT','LICENSE-APACHE','Cargo.toml.orig']:
    source=SOURCE/name;dest=O/name;shutil.copy2(source,dest);assert sha(dest)==sha(source)
    pins[name]={'source':str(source),'sha256':sha(source)}
(O/'result.json').write_text(json.dumps({'complete':True,'package':'num-bigint','version':'0.4.6',
    'scope':'License notices for biguint/serde.rs and bigint/serde.rs retained as format references in structure-metrics-v1/format-sources',
    'runner_sha256':sha(__file__),'retained':pins},indent=2)+'\n')
print('Retained locked num-bigint format-reference license notices',flush=True)
