"""Sample the unchanged full-board failure separately from acceptance timing."""
import hashlib,json,os,subprocess,sys,time
from pathlib import Path
E=Path(__file__).resolve().parent
ROOT=E.parents[2]
PHASE=sys.argv[1]
OUT=E/PHASE
TARGET=Path(os.environ['CARGO_TARGET_DIR'])
inputs=json.loads((E/'profile-inputs-v1.json').read_text())
sha=lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert all(sha(p)==h for p,h in inputs['dynamic_libraries_sha256'].items())
command=['perf','record','--no-buildid-cache','-e','cycles:u','-F','99','--call-graph','dwarf,16384','-o',str(TARGET/'full-board.perf.data'),'--',inputs['binary'],'zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite','--exact','--test-threads=1','--nocapture']
start=time.monotonic()
code=subprocess.run(command,cwd=ROOT/'crates/hyperdrc',env=os.environ|{'DEBUGINFOD_URLS':''}).returncode
record={'command':command,'cwd':str(ROOT/'crates/hyperdrc'),'exit_code':code,'seconds':time.monotonic()-start,'perf_version':subprocess.check_output(['perf','--version'],text=True),'binary_sha256':sha(inputs['binary']),'sampling':'99 Hz user-space cycles, DWARF stack capture 16 KiB; no timing or speedup claim','original_assertion_remains':True,'expected_existing_runtime_failure_is_not_admission':True}
raw=TARGET/'full-board.perf.data'
if raw.is_file():record.update({'raw_path':str(raw),'raw_bytes':raw.stat().st_size,'raw_sha256':sha(raw)})
assert all(sha(p)==h for p,h in inputs['dynamic_libraries_sha256'].items())
(OUT/'profile.json').write_text(json.dumps(record,indent=2)+'\n')
sys.exit(code)
