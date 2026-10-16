"""Run the original benchmark consumers from the just-completed Cargo build."""
import json
from pathlib import Path
import os
import re
import subprocess
import sys

E = Path(__file__).resolve().parent
TARGET = Path(sys.argv[1])
OUT = E / TARGET.name
build = (OUT / 'benchmark-build.stderr').read_text()
env = os.environ | {'CSGRS_BENCH_SAMPLES': '1', 'CSGRS_BENCH_WARMUP': '0',
                    'CSGRS_BENCH_ITERATIONS': '1'}
results = []
for name, selection in [
    ('feature_pipeline', 'feature/mesh_primitives/catalog,feature/profile_to_mesh/extrude_revolve_twist_sweep_loft,feature/hypermesh/buffers_hull_minkowski,feature/hypermesh/cube_minkowski'),
]:
    candidates = re.findall(r'Executable .*\(([^\n]*' + name + r'-[^\n]*)\)', build)
    assert len(candidates) == 1, (name, candidates)
    binary = Path(candidates[0])
    assert binary.is_file() and binary.resolve().is_relative_to(TARGET.resolve())
    result = subprocess.run([str(binary)], env=env | {'CSGRS_BENCH_FILTER': selection},
                            capture_output=True, text=True, timeout=240)
    (OUT / (name + '.stdout')).write_text(result.stdout)
    (OUT / (name + '.stderr')).write_text(result.stderr)
    results.append({'binary': str(binary), 'filter': selection, 'exit_code': result.returncode})
    (OUT / 'benchmark-consumers.json').write_text(json.dumps(results, indent=2) + '\n')
    print(name, result.returncode, flush=True)
assert all(row['exit_code'] == 0 for row in results)
