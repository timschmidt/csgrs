"""Check the exact external unzip dependency and complete archive entries used by both variants."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
WORK=ROOT/'target/properties/native-change-disposition'
OUT=WORK/'fixture-preflight-v1'


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


prep=json.loads((WORK/'preparation-v1/result.json').read_text())
OUT.mkdir(exist_ok=False)
unzip=shutil.which('unzip')
assert unzip
version=subprocess.run([unzip,'-v'],capture_output=True,check=True)
(OUT/'unzip-version.stdout').write_bytes(version.stdout)
(OUT/'unzip-version.stderr').write_bytes(version.stderr)
rows=[]
for variant in ['candidate','committed']:
    source=WORK/(variant+'-source-v1')
    for index,fixture in enumerate(prep['fixtures']):
        path=source/'crates/hyperdrc/docs'/Path(fixture['archive']).name
        assert sha(path)==fixture['sha256']
        command=[unzip,'-p',str(path),fixture['entry']]
        result=subprocess.run(command,capture_output=True,timeout=30)
        assert result.returncode==0
        digest=hashlib.sha256(result.stdout).hexdigest()
        assert digest==fixture['entry_sha256'] and len(result.stdout)==fixture['entry_bytes']
        (OUT/f'{variant}-{index}.kicad_pcb').write_bytes(result.stdout)
        (OUT/f'{variant}-{index}.stderr').write_bytes(result.stderr)
        rows.append({'variant':variant,'command':command,'exit_code':result.returncode,
                     'entry_bytes':len(result.stdout),'entry_sha256':digest})
record={'complete':True,'runner_sha256':sha(__file__),'unzip':unzip,'unzip_sha256':sha(unzip),
        'unzip_version_stdout_sha256':sha(OUT/'unzip-version.stdout'),'rows':rows,
        'all_original_archives_and_entries_present':True,'fixture_resolution_reduced':False}
(OUT/'result.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'complete':True,'exact_full_entry_extractions':len(rows)}),flush=True)
