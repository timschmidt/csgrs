#!/usr/bin/env python3
"""Reject broken production metadata operations and false denotation claims."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from verify_verus import ROOT, PIN, REPORTS, verifier  # noqa: E402
from verus_dependencies import prepare_dependencies, check_dependency_identity  # noqa: E402

MUTATIONS = [
    ('with_sign', 'Self::with_sign_msd(sign, msd, true)', 'Self::with_sign_msd(sign, msd, false)'),
    ('with_sign_msd', 'Sign::NoSign => Self::Zero,', 'Sign::NoSign => Self::Unknown,'),
    ('with_sign_msd', 'sign: Some(sign),', 'sign: Some(Sign::NoSign),'),
    ('with_sign_msd', '                msd,', '                msd: None,'),
    ('map_msd', 'msd: msd.and_then(f),', 'msd: None,'),
    ('map_msd', 'msd: msd.and_then(f),\n                exact_msd,',
     'msd: msd.and_then(f),\n                exact_msd: false,'),
    ('map_msd', 'other => other,', 'other => Self::Unknown,'),
    ('negate', '} => Self::NonZero {\n                sign: Some(Sign::Minus),',
     '} => Self::NonZero {\n                sign: Some(Sign::Plus),'),
    ('inverse', '1_i32.checked_sub(value)', '1_i32.checked_add(value)'),
    ('square', 'match (exact_msd, msd)', 'match (true, msd)'),
    ('square', 'value.checked_mul(2)', 'value.checked_mul(3)'),
    ('square', 'sign: Some(Sign::Plus),', 'sign: Some(Sign::Minus),'),
    ('multiply', 'if left_exact_msd { left_msd } else { None }', 'left_msd'),
    ('multiply', 'if right_exact_msd { right_msd } else { None }', 'right_msd'),
    ('multiply', 'left.checked_add(right)', 'left.checked_sub(right)'),
    ('multiply', '| (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Plus),',
     '| (Some(Sign::Minus), Some(Sign::Minus)) => Some(Sign::Minus),'),
    ('add', '| (Some(Sign::NoSign), Some(Sign::NoSign)) => left_sign,',
     '| (Some(Sign::NoSign), Some(Sign::NoSign)) => None,'),
    ('add', 'if left_exact_msd && right_exact_msd', 'if true'),
    ('add', '(Some(left), Some(right)) if left > right => left_sign,',
     '(Some(left), Some(right)) if left >= right => left_sign,'),
    ('add', '(Some(left), Some(right)) if right > left => right_sign,',
     '(Some(left), Some(right)) if right > left => left_sign,'),
    ('add', 'exact_msd: false,', 'exact_msd: true,'),
    ('sqrt', 'msd.map(crate::verified::word::floor_half)', 'msd.map(|value| value / 2)'),
    ('known_msd', 'Self::NonZero { .. } => None,', 'Self::NonZero { .. } => Some(None),'),
    ('known_msd', '} => Some(Some(*msd)),', '} => None,'),
    ('planning_msd', 'Self::NonZero { msd: None, .. } => None,', 'Self::NonZero { msd: None, .. } => Some(None),'),
    ('planning_msd', 'Self::NonZero { msd: Some(msd), .. } => Some(Some(*msd)),',
     'Self::NonZero { msd: Some(_msd), .. } => None,'),
    ('known_sign', 'Self::Zero => Some(Sign::NoSign),', 'Self::Zero => Some(Sign::Plus),'),
    ('certified_sign_and_msd', '(self.known_sign(), self.known_msd())',
     '(self.known_sign(), self.planning_msd())'),
    ('certified_sign_and_msd', '(self.known_sign(), self.known_msd())',
     '(None, self.known_msd())'),
    ('certified_sign_and_msd', '(self.known_sign(), self.known_msd())',
     '(self.known_sign(), None)'),
    ('negate_sign', 'Sign::Plus => Sign::Minus,', 'Sign::Plus => Sign::Plus,'),
    ('magnitude_bits', 'msd: *msd,', 'msd: 0,'),
    ('magnitude_bits', 'exact_msd: *exact_msd,', 'exact_msd: true,'),
    ('magnitude_bits', '_ => None,', '_ => Some(MagnitudeBits { msd: 0, exact_msd: true }),'),
    ('public_sign', 'Sign::Minus => RealSign::Negative,', 'Sign::Minus => RealSign::Positive,'),
    ('public_sign', 'Sign::NoSign => RealSign::Zero,', 'Sign::NoSign => RealSign::Positive,'),
    ('private_sign', 'RealSign::Positive => Sign::Plus,', 'RealSign::Positive => Sign::Minus,'),
    ('private_sign', 'RealSign::Zero => Sign::NoSign,', 'RealSign::Zero => Sign::Plus,'),
]

MODEL_MUTATIONS = [
    ('sign certificate', 'Some(Sign::Plus) => value > 0real,',
     'Some(Sign::Plus) => value >= 0real,'),
    ('approximation separation', 'approximation > 1 || approximation < -1,',
     'approximation >= 1 || approximation <= -1,'),
    ('negation denotation', 'ensures bound_denotes(result, -value),',
     'ensures bound_denotes(result, value),'),
    ('reciprocal sign', 'ensures bound_denotes(result, 1real / value),',
     'ensures bound_denotes(result, -1real / value),'),
    ('square sign', 'ensures bound_denotes(result, value * value),',
     'ensures bound_denotes(result, -(value * value)),'),
    ('product sign', 'ensures bound_denotes(result, x * y),',
     'ensures bound_denotes(result, -(x * y)),'),
    ('sum sign', 'ensures bound_denotes(result, x + y),',
     'ensures bound_denotes(result, x - y),'),
    ('binade separation', 'ensures absolute(x) < absolute(y),',
     'ensures absolute(x) > absolute(y),'),
    ('square-root binade', 'ensures real_binade(root, exponent as int / 2),',
     'ensures real_binade(root, exponent as int / 2 + 1),'),
    ('zero certificate', 'ensures result == Some(None) ==> value == 0real,',
     'ensures result == Some(None) ==> value != 0real,'),
    ('binary scaling exponent', 'ensures real_binade(value * binary_unit(offset), exponent + offset),',
     'ensures real_binade(value * binary_unit(offset), exponent + offset + 1),'),
    ('mapped offset direction', 'ensures bound_denotes(result, value * binary_unit(offset)),',
     'ensures bound_denotes(result, value * binary_unit(-offset)),'),
    ('mapping callback', '==> mapped == checked_metadata_exponent(exponent as int + offset),',
     '==> mapped == checked_metadata_exponent(exponent as int),'),
    ('public positive sign', 'RealSign::Positive => value > 0real,',
     'RealSign::Positive => value >= 0real,'),
    ('exported magnitude nonzero', 'Some(bits) => value != 0real && (bits.exact_msd',
     'Some(bits) => value == 0real && (bits.exact_msd'),
    ('inexact magnitude', '(bits.exact_msd ==> real_binade(value, bits.msd as int))',
     'real_binade(value, bits.msd as int)'),
]


def mutate_body(source, method, before, after):
    anchor = 'fn ' + method + '('
    if source.count(anchor) != 1:
        raise RuntimeError('Bound mutation method is not unique: ' + method)
    start = source.index('{', source.index(anchor))
    end, depth = start + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    body = source[start:end]
    if body.count(before) != 1:
        raise RuntimeError('Bound mutation anchor is not unique: ' + method)
    return source[:start] + body.replace(before, after) + source[end:]


def run_rejections(verus, dependency_args):
    with tempfile.TemporaryDirectory(prefix='hyperreal-bound-rejections-') as temporary:
        root = Path(temporary)
        (root / 'src/computable/node').mkdir(parents=True)
        (root / 'verification').mkdir()
        bound = root / 'src/computable/node/bounds.rs'
        shutil.copy2(ROOT / 'src/computable/node/bounds.rs', bound)
        shutil.copy2(ROOT / 'src/verified/word.rs', root / 'word.rs')
        shutil.copy2(ROOT / 'src/structural.rs', root / 'structural.rs')
        shutil.copy2(ROOT / 'verification/computable_bounds.rs', root / 'verification/computable_bounds.rs')
        denotation = root / 'verification/bound_denotation.rs'
        shutil.copy2(ROOT / 'verification/bound_denotation.rs', denotation)
        for name in ['magnitude_model', 'integer_approximation_model', 'real_approximation_model']:
            shutil.copy2(ROOT / 'verification' / (name + '.rs'), root / (name + '.rs'))
        (root / 'lib.rs').write_text(
            '#![feature(proc_macro_hygiene)]\nmod verified {\n'
            f'#[path = "{root / "word.rs"}"] pub(crate) mod word;\n}}\n'
            'mod structural;\nmod magnitude_model;\nmod integer_approximation_model;\nmod real_approximation_model;\n'
            '#[path = "verification/computable_bounds.rs"] mod computable_bounds;\n')
        command = [str(verus), '--edition=2024', '--crate-type=lib', '--no-cheating',
                   *dependency_args, str(root / 'lib.rs')]
        baseline = subprocess.run(command, capture_output=True, text=True)
        if baseline.returncode:
            raise RuntimeError('Unmutated bound proofs failed:\n' + baseline.stdout + baseline.stderr)
        original = bound.read_text()
        for method, before, after in MUTATIONS:
            bound.write_text(mutate_body(original, method, before, after))
            result = subprocess.run(command, capture_output=True, text=True)
            bound.write_text(original)
            failures = ['postcondition not satisfied', 'unable to prove post-condition of closure']
            if result.returncode == 0 or not any(message in result.stderr for message in failures):
                raise RuntimeError(f'Expected contract rejection for {method}: {after}\n'
                                   + result.stdout + result.stderr)
            print(f'Rejected incorrect BoundInfo::{method}: {after}', flush=True)
        original = denotation.read_text()
        for name, before, after in MODEL_MUTATIONS:
            if original.count(before) != 1:
                raise RuntimeError('Bound denotation mutation anchor is not unique: ' + name)
            denotation.write_text(original.replace(before, after))
            result = subprocess.run(command, capture_output=True, text=True)
            denotation.write_text(original)
            failures = ['postcondition not satisfied', 'assertion failed',
                        'requires not satisfied', 'precondition not satisfied']
            if result.returncode == 0 or not any(message in result.stderr for message in failures):
                raise RuntimeError(f'Expected denotation rejection for {name}: {after}\n'
                                   + result.stdout + result.stderr)
            print(f'Rejected incorrect bound denotation: {name}', flush=True)


def main():
    verus = verifier()
    args, evidence = prepare_dependencies(ROOT, PIN, REPORTS / 'bound-rejections')
    run_rejections(verus, args)
    check_dependency_identity(evidence)


if __name__ == '__main__':
    main()
