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
    ('inverse', '0_i32.checked_sub(value)', '0_i32.checked_add(value)'),
    ('square', 'msd: match msd', 'msd: match None::<i32>'),
    ('square', '2 * (value as i64) + 1', '3 * (value as i64) + 1'),
    ('square', 'sign: Some(Sign::Plus),', 'sign: Some(Sign::Minus),'),
    ('multiply', 'match (left_msd, right_msd)', 'match (None::<i32>, right_msd)'),
    ('multiply', 'match (left_msd, right_msd)', 'match (left_msd, None::<i32>)'),
    ('multiply', 'left as i64 + right as i64 + 1', 'left as i64 - right as i64 + 1'),
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

# Carries are part of the upper-bound contract. These mutations also require
# the executable proof to reject dropping them or inventing reciprocal bounds.
MUTATIONS += [
    ('square', '2 * (value as i64) + 1', '2 * (value as i64)'),
    ('multiply', 'left as i64 + right as i64 + 1', 'left as i64 + right as i64'),
    ('add', '_ => 1,', '_ => 0,'),
    ('inverse', 'if exact_msd {', 'if true {'),
]

# The production domain predicate must preserve every sufficient certificate,
# including zero/unit boundaries and the distinction between upper and lower
# magnitudes. These edits change the actual included executable body.
MUTATIONS += [
    ('proves_domain', 'Some(Some(msd)) => msd < 0,', 'Some(Some(msd)) => msd <= 0,'),
    ('proves_domain', 'Some(Some(msd)) => msd < 0,', 'Some(Some(msd)) => msd < -1,'),
    ('proves_domain', 'Some(None) => true,', 'Some(None) => false,'),
    ('proves_domain', 'None => false,', 'None => true,'),
    ('proves_domain', 'Probability => matches!(self.known_sign(), Some(Sign::Plus)),',
     'Probability => matches!(self.known_sign(), Some(Sign::Plus | Sign::NoSign)),'),
    ('proves_domain', 'match self.known_msd()', 'match self.planning_msd()'),
    ('proves_domain', 'Some(Some(msd)) => msd >= 0,', 'Some(Some(msd)) => msd <= 0,'),
    ('proves_domain', 'Some(Some(msd)) => msd >= 0,', 'Some(Some(msd)) => msd > 0,'),
    ('proves_domain',
     'matches!(self.known_sign(), Some(Sign::Plus))\n                    && match self.known_msd()',
     'true\n                    && match self.known_msd()'),
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
    ('exported magnitude nonzero', 'Some(bits) => value != 0real && magnitude_upper_bound',
     'Some(bits) => value == 0real && magnitude_upper_bound'),
    ('inexact magnitude', '(bits.exact_msd ==> real_binade(value, bits.msd as int))',
     'real_binade(value, bits.msd as int)'),
]


MODEL_MUTATIONS += [
    ('upper exponent monotonicity',
     'requires magnitude_upper_bound(value, exponent), exponent <= upper,',
     'requires magnitude_upper_bound(value, exponent), exponent <= upper + 1,'),
    ('clamped exponent exactness',
     '(exact_msd && unclamped ==> real_binade(value, upper as int))',
     '(exact_msd ==> real_binade(value, upper as int))'),
    ('clamped upper certificate',
     'Some((upper, unclamped)) => magnitude_upper_bound(value, upper as int)',
     'Some((upper, unclamped)) => magnitude_upper_bound(value, upper as int - 1)'),
    ('product upper carry',
     'ensures magnitude_upper_bound(x * y, left + right + 1),',
     'ensures magnitude_upper_bound(x * y, left + right),'),
    ('sum upper carry',
     '(if left > right { left } else { right }) + if opposed { 0int } else { 1int }),',
     '(if left > right { left } else { right }) + if opposed { 0int } else { 0int }),'),
    ('square-root upper exponent',
     'ensures magnitude_upper_bound(root, exponent as int / 2),',
     'ensures magnitude_upper_bound(root, exponent as int / 2 - 1),'),
    ('upper binary scaling direction',
     'ensures magnitude_upper_bound(value * binary_unit(offset), exponent + offset),',
     'ensures magnitude_upper_bound(value * binary_unit(offset), exponent - offset),'),
    ('closed upper endpoint',
     'Some(e) => (absolute(approximation as real) + 1real) * unit < 2real * binary_unit(e as int)',
     'Some(e) => (absolute(approximation as real) + 1real) * unit <= 2real * binary_unit(e as int)'),
    ('closed exact lower endpoint',
     '&& (exact_msd ==> binary_unit(e as int) <= (absolute(approximation as real) - 1real) * unit),',
     '&& (exact_msd ==> binary_unit(e as int) <= (absolute(approximation as real) + 1real) * unit),'),
]

MODEL_MUTATIONS += [
    ('unit-domain strict upper exponent',
     'requires magnitude_upper_bound(value, exponent as int), exponent < 0,',
     'requires magnitude_upper_bound(value, exponent as int), exponent <= 0,'),
    ('at-least-one needs a lower binade',
     'requires value > 0real, real_binade(value, exponent as int), exponent >= 0,',
     'requires value > 0real, magnitude_upper_bound(value, exponent as int), exponent >= 0,'),
    ('domain requires a valid incoming bound',
     'requires bound_denotes(*bound, value),\n        call_ensures(BoundInfo::proves_domain,',
     'requires true,\n        call_ensures(BoundInfo::proves_domain,'),
    ('domain requires an accepted certificate',
     'call_ensures(BoundInfo::proves_domain, (bound, domain), result), result,',
     'call_ensures(BoundInfo::proves_domain, (bound, domain), result), !result,'),
]


# Selection must retain both inputs when compatible. The production wrapper's
# diagnostics and atomic loop remain separate from this included pure method.
MUTATIONS += [
    ('merge', '(Self::Invalid | Self::Valid(BoundInfo::Unknown), other) => other,',
     '(Self::Invalid | Self::Valid(BoundInfo::Unknown), other) => BoundInfo::Unknown,'),
    ('merge', '(Self::Valid(other), BoundInfo::Unknown) => other,',
     '(Self::Valid(other), BoundInfo::Unknown) => BoundInfo::Unknown,'),
    ('merge', '(Some(known), true, _, _) => (Some(known), true),',
     '(Some(known), true, _, _) => (Some(known), false),'),
    ('merge', '(_, _, Some(proved), true) => (Some(proved), true),',
     '(_, _, Some(proved), true) => (Some(proved), false),'),
    ('merge', 'if known < proved { known } else { proved }',
     'if known > proved { known } else { proved }'),
    ('merge', '(Some(known), false, None, _) => (Some(known), false),',
     '(Some(known), false, None, _) => (None, false),'),
    ('merge', '(None, _, proved, _) => (proved, false),',
     '(None, _, proved, _) => (None, false),'),
    ('merge', '(None, _, proved, _) => (proved, false),',
     '(None, _, proved, _) => (proved, true),'),
    ('merge', 'Some(_) => sign,', 'Some(_) => None,'),
    ('merge', 'None => proved_sign,', 'None => None,'),
    ('merge', '(Self::Valid(retained), _) => retained,',
     '(Self::Valid(retained), _) => BoundInfo::Unknown,'),
    ('merge', '(Self::Invalid | Self::Valid(BoundInfo::Unknown), other) => other,',
     '(Self::Invalid, _) => BoundInfo::Unknown,\n'
     '            (Self::Valid(BoundInfo::Unknown), other) => other,'),
]

MODEL_MUTATIONS += [
    ('exact binade permits equal upper exponent', 'ensures exact <= upper,', 'ensures exact < upper,'),
    ('compatible bounds require the retained denotation',
     'requires bound_denotes(left, value), bound_denotes(right, value),',
     'requires bound_denotes(right, value),'),
    ('compatible bounds require the incoming denotation',
     'requires bound_denotes(left, value), bound_denotes(right, value),',
     'requires bound_denotes(left, value),'),
    ('merge requires the retained denotation',
     'requires bound_denotes(retained, value), bound_denotes(incoming, value),',
     'requires bound_denotes(incoming, value),'),
    ('merge requires the incoming denotation',
     'requires bound_denotes(retained, value), bound_denotes(incoming, value),',
     'requires bound_denotes(retained, value),'),
    ('merge retention requires compatibility',
     'requires compatible_bounds(retained, incoming),', 'requires true,'),
    ('merged domain retention requires compatibility',
     'requires compatible_bounds(*retained, *incoming),', 'requires true,'),
    ('retained upper bounds may only tighten',
     'Some(a) => a <= b && (be ==> ae && a == b),',
     'Some(a) => a >= b && (be ==> ae && a == b),'),
    ('combined evidence may add a domain certificate',
     'ensures (retained_answer || incoming_answer) ==> merged_answer,',
     'ensures (retained_answer || incoming_answer) == merged_answer,'),
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
