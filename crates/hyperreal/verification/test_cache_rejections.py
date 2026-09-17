#!/usr/bin/env python3
"""Require actual cache codecs and their wire-format model to reject corruption."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_verus import ROOT, PIN, REPORTS, verifier  # noqa: E402
from verus_dependencies import prepare_dependencies, check_dependency_identity  # noqa: E402
from test_bound_rejections import mutate_body  # noqa: E402

MUTATIONS = [
    ('encode_bound', 'BoundCache::Valid(BoundInfo::Zero) => Self::TAG_ZERO,',
     'BoundCache::Valid(BoundInfo::Zero) => Self::TAG_UNKNOWN,'),
    ('encode_bound', 'Some(Sign::Plus) => 1,', 'Some(Sign::Plus) => 2,'),
    ('encode_bound', 'Self::TAG_NONZERO |', 'Self::TAG_ZERO |'),
    ('encode_bound', 'Self::MSD_PRESENT | ((msd as u32 as u64) << Self::MSD_SHIFT)',
     '((msd as u32 as u64) << Self::MSD_SHIFT)'),
    ('encode_bound', '((msd as u32 as u64) << Self::MSD_SHIFT)',
     '((msd as u32 as u64) >> Self::MSD_SHIFT)'),
    ('encode_bound', 'encoded |= Self::EXACT_MSD;', 'encoded |= 0;'),
    ('encode_exact_sign', 'ExactSignCache::Valid(Sign::NoSign) => 3,',
     'ExactSignCache::Valid(Sign::NoSign) => 0,'),
    ('encode_exact_sign', 'encoded << Self::EXACT_SIGN_SHIFT',
     'encoded >> Self::EXACT_SIGN_SHIFT'),
    ('decode_exact_sign', '3 => ExactSignCache::Valid(Sign::NoSign),',
     '3 => ExactSignCache::Valid(Sign::Plus),'),
]

MODEL_MUTATIONS = [
    ('zero wire tag', '2 => BoundCache::Valid(BoundInfo::Zero),',
     '2 => BoundCache::Valid(BoundInfo::Unknown),'),
    ('magnitude presence', 'msd: if value & 16 != 0', 'msd: if value & 16 == 0'),
    ('magnitude payload', 'Some(((value >> 32) as u32) as i32)', 'Some(0)'),
    ('exact magnitude flag', 'exact_msd: value & 32 != 0,', 'exact_msd: value & 32 == 0,'),
    ('bound update value',
     'ensures bound_word((current & AtomicFacts::NON_BOUND_MASK) | encoded) == value,',
     'ensures bound_word((current & AtomicFacts::NON_BOUND_MASK) | encoded) == BoundCache::Invalid,'),
    ('bound update retained fields', '== current & AtomicFacts::NON_BOUND_MASK,',
     '== encoded & AtomicFacts::NON_BOUND_MASK,'),
    ('sign update bound',
     'ensures bound_word((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) == bound_word(current),',
     'ensures bound_word((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) == BoundCache::Invalid,'),
    ('sign update value',
     'exact_sign_denotation((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) == value,',
     'exact_sign_denotation((current & !AtomicFacts::EXACT_SIGN_MASK) | encoded) == exact_sign_denotation(current),'),
    ('construction demand', '(demand as u16 as u64) << AtomicFacts::LINEAR_DEMAND_SHIFT',
     '(demand as u16 as u64) << AtomicFacts::MSD_SHIFT'),
    ('construction inverse flag',
     'if inverse_trig { AtomicFacts::CONTAINS_INVERSE_TRIG_OR_PI } else { 0 }',
     'if inverse_trig { 0 } else { AtomicFacts::CONTAINS_INVERSE_TRIG_OR_PI }'),
]


def main():
    verus = verifier()
    args, evidence = prepare_dependencies(ROOT, PIN, REPORTS / 'cache-rejections')
    with tempfile.TemporaryDirectory(prefix='hyperreal-cache-rejections-') as temporary:
        root = Path(temporary)
        (root / 'src/computable/node').mkdir(parents=True)
        (root / 'verification').mkdir()
        for name in ['bounds.rs', 'representation.rs']:
            shutil.copy2(ROOT / 'src/computable/node' / name, root / 'src/computable/node' / name)
        shutil.copy2(ROOT / 'src/verified/word.rs', root / 'word.rs')
        shutil.copy2(ROOT / 'src/structural.rs', root / 'structural.rs')
        for name in ['computable_bounds', 'bound_denotation', 'computable_cache', 'cache_encoding_model',
                     'magnitude_model', 'integer_approximation_model', 'real_approximation_model']:
            shutil.copy2(ROOT / 'verification' / (name + '.rs'), root / 'verification' / (name + '.rs'))
        modules = ['computable_bounds', 'computable_cache', 'magnitude_model',
                   'integer_approximation_model', 'real_approximation_model']
        (root / 'lib.rs').write_text(
            '#![feature(proc_macro_hygiene)]\nmod structural;\nmod verified {\n'
            f'#[path = "{root / "word.rs"}"] pub(crate) mod word;\n}}\n'
            + ''.join(f'#[path = "verification/{name}.rs"] mod {name};\n' for name in modules))
        command = [str(verus), '--edition=2024', '--crate-type=lib', '--no-cheating',
                   *args, str(root / 'lib.rs')]
        baseline = subprocess.run(command, capture_output=True, text=True)
        if baseline.returncode:
            raise RuntimeError('Unmutated cache proofs failed:\n' + baseline.stdout + baseline.stderr)
        production = root / 'src/computable/node/representation.rs'
        model = root / 'verification/cache_encoding_model.rs'
        originals = {path: path.read_text() for path in [production, model]}
        changes = [(production, method, mutate_body(originals[production], method, before, after))
                   for method, before, after in MUTATIONS]
        for name, before, after in MODEL_MUTATIONS:
            if originals[model].count(before) != 1:
                raise RuntimeError('Cache model mutation anchor is not unique: ' + name)
            changes.append((model, name, originals[model].replace(before, after)))
        for path, name, changed in changes:
            path.write_text(changed)
            result = subprocess.run(command, capture_output=True, text=True)
            path.write_text(originals[path])
            failures = ['postcondition not satisfied', 'assertion failed',
                        'requires not satisfied', 'precondition not satisfied']
            if result.returncode == 0 or not any(message in result.stderr for message in failures):
                raise RuntimeError(f'Expected cache rejection for {name}:\n' + result.stdout + result.stderr)
            print(f'Rejected incorrect cache codec/model: {name}', flush=True)
    check_dependency_identity(evidence)


if __name__ == '__main__':
    main()
