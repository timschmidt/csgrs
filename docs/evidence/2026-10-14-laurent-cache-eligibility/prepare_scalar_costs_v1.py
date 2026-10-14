"""Freeze references to original probes and binaries without changing qualified sources."""
import json
import os
from pathlib import Path
import subprocess

from eligibility_cost_common_v1 import E, ROOT, PARENT, COST_WORK, COST_PREP, INPUTS, SUITES, sha, dump, verify_qualification, verify_costs


def main():
    qualified = verify_qualification(full=True)
    original = json.loads((PARENT / 'cost-preparation-v1/protocol.json').read_text())
    libraries = json.loads((PARENT / 'library-costs-v1/libraries.json').read_text())
    binaries = json.loads((PARENT / 'library-costs-v1/binaries.json').read_text())
    assert qualified['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert qualified['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    COST_WORK.mkdir(exist_ok=False)
    COST_PREP.mkdir(exist_ok=False)
    variants = {group: {k: original['variants'][old][k] for k in ['source', 'target']}
                for group, old in [('main', 'main'), ('borrowed', 'candidate')]}
    variants['candidate'] = {'source': qualified['variants']['candidate']['source'],
                             'target': str(COST_WORK / 'candidate-target')}
    reused_libraries = {group: libraries[old] for group, old in [('main', 'main'), ('borrowed', 'candidate')]}
    reused_binaries = {group + '-' + suite + '-' + mode: binaries[old + '-' + suite + '-' + mode]
        for group, old in [('main', 'main'), ('borrowed', 'candidate')]
        for suite in SUITES for mode in ['time', 'memory']}
    prior_paths = [E / 'preparation-v1/protocol.json', E / 'qualification-v1/result.json',
        E / 'qualification-v1/library.json', E / 'qualification-assessment-v1.json',
        PARENT / 'cost-preparation-v1/protocol.json', PARENT / 'library-costs-v1/result.json',
        PARENT / 'library-costs-v1/binaries.json', PARENT / 'library-costs-v1/libraries.json',
        PARENT / 'library-assessment-v1.json', PARENT / 'library-interpretation-v1.json']
    prep = {'scope': __doc__, 'variants': variants,
        'reused_libraries': reused_libraries, 'reused_binaries': reused_binaries,
        'probes': {suite: original['probes'][suite + '_probe.rs'] for suite in SUITES},
        'inputs_sha256': {p: sha(E / p) for p in INPUTS},
        'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in prior_paths},
        'compiler': qualified['compiler'], 'inherited_rustflags': qualified['inherited_rustflags'],
        'new_runtime_sources': 0, 'original_probe_bytes_unchanged': True,
        'fresh_production_build_matches_prior_package_selection': True,
        'runtime_admitted': False, 'plan_complete': False}
    verify_costs(prep, full=True)
    dump(COST_PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'reused_binaries': 12, 'unchanged_probes': 3,
                      'new_runtime_sources': 0, 'runtime_admitted': False}), flush=True)


if __name__ == '__main__':
    main()
