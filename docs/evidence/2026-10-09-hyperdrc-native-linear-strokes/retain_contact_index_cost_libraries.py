"""Preserve measured libraries before later Cargo commands can replace outputs."""
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[3]
EVIDENCE=Path(__file__).resolve().parent
OUT=EVIDENCE/sys.argv[1]
manifest=json.loads((OUT/'manifest.json').read_text())
assert len(manifest['commands'])==30 and all(row['exit_code']==0 for row in manifest['commands'])
candidate=Path(manifest['candidate_source_record']).parent.name
destination=ROOT/'target'/candidate/'measured-libraries'
destination.mkdir()
records={}
for variant,library in manifest['libraries'].items():
    source=Path(library['path'])
    assert hashlib.sha256(source.read_bytes()).hexdigest()==library['sha256']
    retained=destination/(variant+'.rlib')
    shutil.copyfile(source,retained)
    assert hashlib.sha256(retained.read_bytes()).hexdigest()==library['sha256']
    records[variant]={'original_path':str(source),'retained_path':str(retained),'bytes':library['bytes'],'sha256':library['sha256']}
record={'scope':'Copies made after all resource trials completed, before any further Cargo command, to preserve measured library bytes if later validation changes top-level build outputs. The executed manifest and logs are unchanged.','command':[sys.executable,str(Path(__file__).resolve()),sys.argv[1]],'libraries':records}
(OUT/'retained-libraries.json').write_text(json.dumps(record,indent=2)+'\n')
(OUT/'retain-libraries.py').write_bytes(Path(__file__).read_bytes())
print('Measured library bytes preserved:',candidate,flush=True)
