"""Verify all immutable lower files after the new phase."""
import gzip,hashlib,json,sys
from pathlib import Path
E=Path(__file__).resolve().parent
with gzip.open(E/sys.argv[1]/'overlay-lower-sha256.json.gz','rt') as f:
 expected=json.load(f)
for name,digest in expected.items():
 with Path(name).open('rb') as stream:
  assert hashlib.file_digest(stream,'sha256').hexdigest()==digest,name
print(json.dumps({'verified_lower_files':len(expected),'mismatches':0}))
