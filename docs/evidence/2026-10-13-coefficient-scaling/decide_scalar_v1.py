"""Record scalar tradeoffs and the source review before native qualification."""
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / 'scalar-decision-v1.json'
    assert not out.exists()
    preparation = json.loads((E / 'preparation-v1/protocol.json').read_text())
    costs = json.loads((E / 'cost-assessment-v1.json').read_text())
    assert costs['source_parent_main_and_artifacts_unchanged']
    assert costs['all_mathematical_witnesses_and_representation_inventories_match']
    source = Path(preparation['variants']['both']['source'])
    paths = [
        'crates/hyperreal/src/computable/node/algebra.rs',
        'crates/hyperreal/src/computable/node/primitive_constructors.rs',
        'crates/hyperreal/src/computable/node/representation.rs',
        'crates/hyperreal/src/rational/arithmetic/comparison.rs',
        'crates/hyperreal/src/rational/arithmetic/queries_conversion.rs',
        'crates/hyperreal/src/rational/arithmetic/ops.rs',
    ]
    structural = (source / paths[1]).read_text()
    equality = structural.split('    pub(crate) fn internal_structural_eq', 1)[1].split(
        '    fn exact_shared_perturbation_order', 1)[0]
    assert 'let mut remaining = 64;' in equality and 'let mut compared = HashSet::new();' in equality
    assert all(token not in equality for token in ['.cache', '.facts', '.cached(', '.signal', 'strong_count'])
    algebra = (source / paths[0]).read_text()
    original = (Path(preparation['variants']['baseline']['source']) / paths[0]).read_text()
    right = (Path(preparation['variants']['right']['source']) / paths[0]).read_text()
    first = '''        if other.radicals.is_empty() && other.atoms.is_empty() {
            self.scale_in_place(&other.rational);
            return Some(self);
        }
'''
    second = '''        if self.radicals.is_empty() && self.atoms.is_empty() {
            let mut scaled = other.clone();
            scaled.scale_in_place(&self.rational);
            return Some(scaled);
        }
'''
    assert right.count(first) == algebra.count(first) == algebra.count(second) == 1
    assert right.replace(first, '').replace('fn multiply(mut self, other: &Self)', 'fn multiply(self, other: &Self)') == original
    assert algebra.replace(first + second, '').replace('fn multiply(mut self, other: &Self)', 'fn multiply(self, other: &Self)') == original
    variants = {}
    for variant in ['right', 'both']:
        timing = [row for row in costs['timing_rows'] if row['variant'] == variant]
        memory = [row for row in costs['memory_rows'] if row['variant'] == variant]
        assert len(timing) == len(memory) == 68
        assert all(field['increased_blocks'] == 0 for row in memory for field in row['fields'].values())
        assert all(field['block_deltas'] == [0] * 5 for row in memory
                   for name, field in row['fields'].items() if name == 'retained_bytes_delta')
        variants[variant] = {
            'decision': 'advance-to-isolated-native-qualification-with-scalar-tradeoffs-open',
            'faster_in_all_five_blocks': sum(row['faster_blocks'] == 5 for row in timing),
            'slower_in_all_five_blocks': costs['slower_in_all_five_blocks'][variant],
            'slower_in_at_least_four_blocks': [row for row in timing if row['slower_blocks'] >= 4],
            'slower_median_block_ratio_rows': sum(row['median_block_ratio'] > 1 for row in timing),
            'allocation_rows_with_any_reduction': sum(any(field['median_block_delta'] < 0
                                                         for field in row['fields'].values()) for row in memory),
            'memory_fields_increased_in_any_sample': 0,
            'retained_requested_bytes_unchanged_in_every_sample': True,
            'probe_binary_sizes': [row for row in costs['binary_sizes'] if row['variant'] == variant],
        }
    review = {
        'source': str(source),
        'files_sha256': {name: sha(source / name) for name in paths},
        'internal_structural_eq': 'Reads immutable Approximation variants, exact rational/integer leaves, and Arc identity. Its initial 64-visit recursion falls back to an iterative identity-pair walk; it does not return Unknown on budget exhaustion. It never reads approximation/fact caches, signals or owner counts. Warming cannot change the atom equality used by insert_atom.',
        'coefficient_invariants': 'The production constructors create empty or singleton vectors; insert_radical and insert_atom merge equal identities and remove zero coefficients. Radicals exclude one and are capped at eight. add and general multiply use those insertion methods. The single-radical inverse and graph-free retained-certificate round trip preserve these identities. Rational scaling cannot introduce a new identity, duplicate or radicand one; scale_in_place removes zero coefficients. Thus reinsertion is redundant for these scalar-only products under the existing invariants.',
        'arithmetic_and_owners': 'The existing MulAssign delegates to exact borrowed Rational multiplication. The right branch consumes owned vectors; the both branch clones the borrowed vectors once and reverses the rational-constant product operand order. This can alter allocation/cache work even when values agree. The four shared tests cover zero, negative and fractional scales in both directions, unsupported products, opaque owner release and cold/warm/cloned cancellation.',
        'unchanged_fallbacks': 'Removing only the two added branches and the mut binding exactly recovers the baseline algebra file. General multiplication, polynomial insertion order, 512-node passes, retained cache and mandatory original retry are unchanged.',
        'limits': 'Source reasoning plus the recorded regressions, not a universal formal proof of cache behavior. No high-precision quadratic-tower hypothesis is resolved by this review.',
    }
    decision = {
        'decision': 'advance-both-isolated-variants-to-native-qualification-no-runtime-admission',
        'assessor_sha256': sha(Path(__file__)),
        'cost_assessment_sha256': sha(E / 'cost-assessment-v1.json'),
        'correctness_sha256': sha(E / 'correctness-v1.json'),
        'source_review': review,
        'variants': variants,
        'reason': 'Both preserve the scalar witnesses and have repeatable allocation savings without any measured memory increase. Right scaling saves 2000 allocations and 192000 requested bytes in each of six opaque query rows per 1000 inputs. Bidirectional scaling saves 264000 bytes in those rows and reduces requested bytes in twelve additional rows. These results support testing real consumers; they do not settle the timing tradeoffs or select a production variant.',
        'timing_limits': 'The all-five slower rows are right wide_integer_affine_cloned (median block ratio 1.00630) and both wide_integer_affine_warm (1.01641). Both also have radical-Laurent query rows slower in four blocks, with ratios up to 1.19825 for right and 1.17512 for both; their median paired total differences are 1350 to 2700 ns per 4096 inputs. Every additional adverse row and all ranges remain in the cost assessment. Timing variation is substantial; no statistical or application-speedup inference is made.',
        'next_step': 'Qualify right and both separately with strict consumer Clippy, full all-feature curve and DRC tests/docs, and the unchanged complete-board 120-second worker-CPU and copper-neck 60-second check-wall-time assertions. Keep gate failures. Then run fixed native endpoint/broad controls and replay complete unnormalized reports against the same baseline; bind every binary to its source, target, features and hash. No extension of the completed scalar samples.',
        'runtime_admitted': False,
        'plan_complete': False,
    }
    out.write_text(json.dumps(decision, indent=2) + '\n')
    ledger = ROOT / 'docs/properties-status.json'
    status = json.loads(ledger.read_text())
    status['current_work']['isolated_coefficient_scaling'].update({
        'status': decision['decision'],
        'evidence': str(out.relative_to(ROOT)),
        'cost_evidence': str((E / 'cost-assessment-v1.json').relative_to(ROOT)),
        'cost_summary': 'All 90 fixed processes preserve witnesses and inventories. No allocation field increases in any sample; retained requested bytes match. Right saves allocations/bytes in six rows; both reduces bytes in eighteen. Each has one all-five slower timing row and further adverse four-block rows, retained in full.',
        'next_step': decision['next_step'],
        'runtime_admitted': False,
    })
    ledger.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'decision': decision['decision'], 'variants': list(variants), 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
