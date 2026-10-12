#!/usr/bin/env python3
"""Reject incorrect, weakened and premise-free production approximation bounds."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_verus import ROOT, PIN, REPORTS, verifier  # noqa: E402
from verus_dependencies import prepare_dependencies, check_dependency_identity  # noqa: E402

MUTATIONS = [('early-threshold-too-small', 'executable', 'if magnitude_bits <= 1 {', 'if magnitude_bits <= 0 {'), ('discard-certified-result', 'weaker-output', '            result\n', '            BoundInfo::Unknown\n'), ('all-ones-reversed', 'executable', 'let reaches_next_binade = trailing_ones == magnitude_bits;', 'let reaches_next_binade = trailing_ones != magnitude_bits;'), ('exponent-correction-reversed', 'executable', 'if reaches_next_binade { 0 } else { 1 },', 'if reaches_next_binade { 1 } else { 0 },'), ('discard-precision-offset', 'executable', '                false,\n                prec,', '                false,\n                0,'), ('discard-exact-exponent', 'weaker-output', 'BoundInfo::with_sign_msd(sign, msd, exact_msd)', 'BoundInfo::with_sign_msd(sign, msd, false)'), ('discard-exponent', 'weaker-output', 'BoundInfo::with_sign_msd(sign, msd, exact_msd)', 'BoundInfo::with_sign_msd(sign, None, exact_msd)'), ('force-positive-sign', 'executable', 'let sign = appr.sign();', 'let sign = Sign::Plus;'), ('power-of-two-test-reversed', 'executable', 'trailing_zeros != Some(magnitude_bits - 1)', 'trailing_zeros == Some(magnitude_bits - 1)'), ('exact-all-ones-reversed', 'executable', '                && !reaches_next_binade;', '                && reaches_next_binade;'), ('missing-normalization', 'premise', 'requires appr.metadata_normalized(), appr.metadata_storage_bits() <= u64::MAX,', 'requires appr.metadata_storage_bits() <= u64::MAX,'), ('missing-storage-bound', 'premise', 'requires appr.metadata_normalized(), appr.metadata_storage_bits() <= u64::MAX,', 'requires appr.metadata_normalized(),')]


def run_rejections(verus, dependency_args):
    with tempfile.TemporaryDirectory(prefix='hyperreal-approximation-rejections-') as temporary:
        root = Path(temporary)
        shutil.copytree(ROOT / 'src', root / 'src')
        (root / 'verification').mkdir()
        for path in (ROOT / 'verification').glob('*.rs'):
            shutil.copy2(path, root / 'verification' / path.name)
        path = root / 'src/computable/node/approximation_bound.rs'
        original = path.read_text()
        command = [str(verus), '--edition=2024', '--crate-type=lib', '--no-cheating', '--output-json',
                   *dependency_args, '--verify-only-module', 'computable_bounds::approximation',
                   '--verify-function', 'bound_from_approx', str(root / 'verification/lib.rs')]
        variants = [('baseline', 'baseline', original)]
        for name, kind, before, after in MUTATIONS:
            if original.count(before) != 1:
                raise RuntimeError('Producer mutation anchor is not unique: ' + name)
            variants.append((name, kind, original.replace(before, after)))
        for name, kind, text in variants:
            path.write_text(text)
            result = subprocess.run(command, capture_output=True, text=True)
            path.write_text(original)
            report = json.loads(result.stdout).get('verification-results', {})
            if kind == 'baseline':
                passed = (result.returncode == 0 and report.get('verified') == 1 and report.get('errors') == 0
                          and all(not line.strip() or line.startswith('note: verifying ')
                                  for line in result.stderr.splitlines()))
            else:
                failures = ['postcondition not satisfied', 'assertion failed', 'precondition not satisfied',
                            'requires not satisfied', 'possible arithmetic underflow/overflow']
                passed = (result.returncode != 0 and report.get('errors', 0) > 0
                          and not report.get('encountered-vir-error', True)
                          and any(message in result.stderr for message in failures))
            if not passed:
                raise RuntimeError('Unexpected producer control result for ' + name + ':\n'
                                   + result.stdout + result.stderr)
            print(('Verified ' if kind == 'baseline' else 'Rejected ') + 'approximation producer: ' + name,
                  flush=True)


def main():
    verus = verifier()
    arguments, evidence = prepare_dependencies(ROOT, PIN, REPORTS, verus_path=verus)
    run_rejections(verus, arguments)
    check_dependency_identity(evidence)


if __name__ == '__main__':
    main()
