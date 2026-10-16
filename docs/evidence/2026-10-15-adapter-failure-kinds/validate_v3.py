"""Reuse the frozen serial evidence runner with the adapter-specific contract."""
import hashlib
import sys
sys.dont_write_bytecode = True
import importlib.util
from pathlib import Path

E = Path(__file__).resolve().parent
BASE = E.parent / '2026-10-15-factored-product-proofs/validate_v8.py'
assert hashlib.sha256(BASE.read_bytes()).hexdigest() == 'f3ee76657ed7c1e8792452ded7c2380c1ad03a41abb312d80f9adf68b4dc6f05'
spec = importlib.util.spec_from_file_location('preserved_serial_runner', BASE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
runner.E = E
original_dump = runner.dump


def dump(path, value):
    if path.name == 'protocol.json':
        value.update({
            'wrapper_sha256': runner.sha(Path(__file__)),
            'artifact_preparation': 'Explicit first-command copy-on-write overlay; see candidate-v3/overlay.json and storage-contract-v1.md. Ordinary copy seed is null. All lower artifacts are pinned before mounting and verified at closure; private upper storage receives writes.',
            'properties': ['P6', 'P8', 'P9', 'P18', 'P25', 'P30', 'P35', 'P37'],
            'owning_layers': ['CSGRS scalar adapter typed-error forwarding', 'C ABI category mapping and language consumers'],
            'domain': 'The four mesh Boolean operations and their context variants, across RawReal/F32/F64/I128. Native input validation, policies, geometry, completion and uncertainty are unchanged. Preserve the native HypermeshError through the existing ValidationError::Mesh category classifier and AdapterError::Construction, retaining display and both Error::source links.',
            'consumers': ['ScalarMesh public API', 'csgrs-ffi status_for_error', 'C/C++/Python/Go consumers'],
            'missing_acceptance_test': ['Reproduce text-only InvalidInput and Undecided adapter failures on the target revision before production edits.', 'Keep invalid operands visible when composed with empty, all scalar families and all eight entry points.', 'Run actual affected binding consumers, preserve ownership and typed failures, and verify subsequent recovery.'],
            'semantic_oracle': 'Independent regularized-set identities and rotation/translation identities define the fixtures. Native errors supply categories and text at the adapter boundary; equality to a native wrapper is a propagation test, not an independent geometry oracle.',
            'limitations': 'External wall/per-process address-space containment only; no internal full-query work or heap-byte bound. This repair is not a release or performance claim. E12 records the inherited 27093 missing artifact paths; no recovery is claimed.',
            'previous_goal_turn': '5080ba4b4 completes all original 8/16/24/32 strict drill chains and final focused qualification. The remaining adapter methods erase native Boolean error types. Pending native files remain separately unadmitted.',
        })
    original_dump(path, value)


runner.dump = dump
if __name__ == '__main__':
    runner.main()
