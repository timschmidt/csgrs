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
    ('kernel_comparison', 'kernel/distribute_linear/box_8,kernel/distribute_grid/box_4x4,kernel/distribute_arc/box_12_30_degree_steps'),
    ('feature_pipeline', 'feature/mesh_distribution/arc_linear_grid'),
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
