"""Freeze a new diagnostic driver; preserve all four libraries and source trees."""
import gzip
import json
import os
from pathlib import Path
import subprocess

from rational_trace_common_v1 import E, ROOT, PARENT, COVERAGE, TRACE_PREP, TRACE_WORK, GROUPS, INPUTS, sha, dump, prior_guard, verify, identity


def main():
    prior_guard(full=True)
    assert not TRACE_WORK.exists() and not TRACE_PREP.exists()
    TRACE_WORK.mkdir()
    TRACE_PREP.mkdir()
    original = (PARENT / 'probes-v1/rational_probe.rs').read_bytes()
    suffix = original[original.index(b'fn fixture('):]
    driver = (E / 'rational_trace_header_v1.rs').read_bytes().rstrip(b'\n') + b'\n\n' + suffix
    (TRACE_PREP / 'probe.rs').write_bytes(driver)
    old = json.loads((E / 'library-costs-v1/libraries.json').read_text())
    libraries = {g: {k: v for k, v in old[g].items() if k in ['hyperreal', 'serde_json']} for g in GROUPS[:3]}
    libraries['cache-coverage'] = json.loads((COVERAGE / 'diagnostic-v1/libraries.json').read_text())
    assert all({k: v['features'] for k, v in group.items()} ==
               {k: v['features'] for k, v in libraries['main'].items()} for group in libraries.values())
    cost_protocol = json.loads((E / 'library-costs-v1/protocol.json').read_text())
    counter_protocol = json.loads((COVERAGE / 'diagnostic-v1/protocol.json').read_text())
    compiler = subprocess.check_output(['rustc', '-vV'], text=True)
    assert compiler == cost_protocol['compiler'] == counter_protocol['compiler']
    assert os.environ.get('RUSTFLAGS') == cost_protocol['inherited_rustflags'] == counter_protocol['inherited_rustflags']
    identities = []
    raw_paths = []
    for group in GROUPS[:3]:
        for repetition in range(1, 7):
            path = E / 'library-costs-v1' / f'sample-rational-time-{repetition}-{group}.stdout.gz'
            identities.append(identity(gzip.decompress(path.read_bytes())))
            raw_paths.append(path)
    assert len(identities) == 18 and len(identities[0]) == 24
    assert all(r == identities[0] for r in identities)
    reviewed_paths = [
        E / 'library-interpretation-v1.json', E / 'library-assessment-v1.json',
        E / 'library-costs-v1/protocol.json', E / 'library-costs-v1/result.json', E / 'library-costs-v1/libraries.json',
        COVERAGE / 'preparation-v1/protocol.json', COVERAGE / 'diagnostic-v1/protocol.json',
        COVERAGE / 'diagnostic-v1/result.json', COVERAGE / 'diagnostic-v1/libraries.json',
        COVERAGE / 'diagnostic-v1/qualification.json', COVERAGE / 'instrumentation-edits-v1.json',
        COVERAGE / 'cache_trace_v1.rs', COVERAGE / 'cache_trace_control_v1.rs',
        PARENT / 'probes-v1/rational_probe.rs', *raw_paths,
    ]
    prep = {'scope': __doc__, 'compiler': compiler, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
            'groups': GROUPS, 'libraries': libraries,
            'inputs_sha256': {p: sha(E / p) for p in INPUTS},
            'prior_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in reviewed_paths},
            'probe_sha256': sha(TRACE_PREP / 'probe.rs'), 'fixture_suffix_bytes': len(suffix),
            'identity': identities[0], 'all_18_closed_rational_timing_identities_match': True,
            'diagnostic_scalar_tests_reused': 875, 'production_candidate_scalar_tests_reused': 876,
            'new_library_builds': 0, 'new_runtime_sources': 0,
            'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    verify(prep, full=True)
    dump(TRACE_PREP / 'protocol.json', prep)
    print(json.dumps({'prepared': True, 'groups': GROUPS, 'new_library_builds': 0,
                      'original_fixture_suffix_bytes': len(suffix)}), flush=True)


if __name__ == '__main__':
    main()
