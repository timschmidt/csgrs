"""Retain completed disposition logs while leaving physical executables and targets intact."""
import hashlib
import json
from pathlib import Path
import shutil

E=Path(__file__).resolve().parent
ROOT=E.parents[2]
WORK=ROOT/'target/properties/native-change-disposition'


def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def read(path):return json.loads(Path(path).read_text())

assert not (E/'collection-v1.json').exists()
folders=['preparation-v1','fixture-preflight-v1','candidate-validation-v1','committed-validation-v1',
         'wasm-validation-v1','committed-proposed-regressions-v1']
for folder in folders:assert read(WORK/folder/'result.json')['complete'],folder
retained={}
for folder in folders:
    destination=E/folder
    assert not destination.exists()
    for source in sorted((WORK/folder).rglob('*')):
        assert not source.is_symlink(),source
        if not source.is_file():continue
        if source.name=='native_linear_strokes' or source.suffix=='.kicad_pcb':continue
        output=destination/source.relative_to(WORK/folder)
        output.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,output)
        assert sha(output)==sha(source)
        retained[str(output.relative_to(E))]={'original':str(source),'sha256':sha(source)}
record={'complete':True,'runner_sha256':sha(__file__),'retained_completed_evidence':retained,
        'physical_executables_and_targets_modified':False,
        'full_extracted_boards_retained_in_physical_fixture_directory':True,
        'original_archive_bytes_preserved':True,'full_plan_complete':False}
(E/'collection-v1.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps({'retained_files':len(retained),'physical_targets_modified':False}),flush=True)
