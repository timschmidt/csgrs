"""Index provably rational supports while preserving original match ordering."""
import gzip
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
S = W / 'candidate-source-v1'
O = W / 'candidate-preparation-v1'
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f, 'sha256').hexdigest()
def read(p): return json.loads(Path(p).read_text())
def dump(p, v): Path(p).write_text(json.dumps(v, indent=2) + '\n')
baseline = read(W / 'baseline-validation-v1/result.json')
assert baseline['complete'] and baseline['cargo_target_closed'] and baseline['command_failures'] == []
assert '7 passed; 0 failed' in (W / 'baseline-validation-v1/contracts.stdout').read_text()
with gzip.open(W / 'baseline-preparation-v1/source-hashes.json.gz', 'rt') as f: inputs = json.load(f)
assert all(sha(W / 'baseline-source-v1' / p) == h for p, h in inputs.items())
S.mkdir(exist_ok=False); O.mkdir(exist_ok=False)
for p, h in inputs.items():
    dest = S / p; dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(W / 'baseline-source-v1' / p, dest); assert sha(dest) == h
name = 'crates/hypercurve/src/curve_intersection.rs'
text = (S / name).read_text()
assert text.count('use hyperreal::Real;') == 1
text = text.replace('use hyperreal::Real;', 'use hyperreal::{Rational, Real};\nuse std::collections::HashMap;')
anchor = '    circular_support_relations: Vec<CircularSupportRelationCacheEntry>,\n'
assert text.count(anchor) == 1
text = text.replace(anchor, anchor + '''    exact_circular_support_index: HashMap<ExactCircularSupportPairKey, usize>,
    unkeyed_circular_support_relations: Vec<usize>,
''')
anchor = '''#[derive(Debug)]
struct CircularPointParameterCacheEntry {'''
assert text.count(anchor) == 1
text = text.replace(anchor, '''/// Ordered support values owned by this temporary intersection batch.
/// Rational hashing uses the scalar carrier's exact numeric equality, including
/// internally unreduced values. No geometry address or finite projection is a key.
#[derive(Debug, Eq, Hash, PartialEq)]
struct ExactCircularSupportPairKey([[Rational; 3]; 2]);

impl ExactCircularSupportPairKey {
    fn new(first: CircularSupportRef<'_>, second: CircularSupportRef<'_>) -> Option<Self> {
        let support = |circle: CircularSupportRef<'_>| {
            Some([
                circle.center.x().exact_rational_ref()?.clone(),
                circle.center.y().exact_rational_ref()?.clone(),
                circle.radius_squared.exact_rational_ref()?.clone(),
            ])
        };
        Some(Self([support(first)?, support(second)?]))
    }
}

''' + anchor)
old = '''        if let Some(entry) = self.circular_support_relations.iter().find(|entry| {
            entry.first.center == *first.center
                && entry.first.radius_squared == *first.radius_squared
                && entry.second.center == *second.center
                && entry.second.radius_squared == *second.radius_squared
        }) {
            return Ok(Some(entry.relation.clone()));
        }
'''
new = '''        let key = ExactCircularSupportPairKey::new(first, second);
        let indexed = key
            .as_ref()
            .and_then(|key| self.exact_circular_support_index.get(key).copied());
        let matches = |index: usize| {
            let entry = &self.circular_support_relations[index];
            entry.first.center == *first.center
                && entry.first.radius_squared == *first.radius_squared
                && entry.second.center == *second.center
                && entry.second.radius_squared == *second.radius_squared
        };
        let retained = if key.is_some() {
            // Unequal rational keys cannot match. Earlier symbolic entries
            // still get their original equality proof and priority; scalar
            // warming may have made one of those equalities recognizable.
            self.unkeyed_circular_support_relations
                .iter()
                .copied()
                .take_while(|&index| indexed.is_none_or(|cached| index < cached))
                .find(|&index| matches(index))
                .or(indexed)
        } else {
            (0..self.circular_support_relations.len()).find(|&index| matches(index))
        };
        if let Some(index) = retained {
            return Ok(Some(self.circular_support_relations[index].relation.clone()));
        }
'''
assert text.count(old) == 1
text = text.replace(old, new)
anchor = '''        self.circular_support_relations
            .push(CircularSupportRelationCacheEntry {'''
assert text.count(anchor) == 1
text = text.replace(anchor, '''        let index = self.circular_support_relations.len();
''' + anchor)
anchor = '''                relation: relation.clone(),
            });
        Ok(Some(relation))'''
assert text.count(anchor) == 1
text = text.replace(anchor, '''                relation: relation.clone(),
            });
        if let Some(key) = key {
            self.exact_circular_support_index.insert(key, index);
        } else {
            self.unkeyed_circular_support_relations.push(index);
        }
        Ok(Some(relation))''')
(S / name).write_text(text)
formatted = subprocess.run(['cargo','fmt','--all'], cwd=S, capture_output=True)
(O / 'format.stdout').write_bytes(formatted.stdout); (O / 'format.stderr').write_bytes(formatted.stderr)
assert formatted.returncode == 0
final = {p: sha(S / p) for p in inputs}
changed = {p: h for p, h in final.items() if inputs[p] != h}
assert set(changed) == {name}
with gzip.open(O / 'source-hashes.json.gz', 'wt') as f: json.dump(final, f, sort_keys=True)
dump(O / 'result.json', {'complete': True, 'base_commit': read(W / 'baseline-preparation-v1/result.json')['base_commit'],
    'source':str(S), 'source_inputs':len(final), 'source_hashes_sha256':sha(O / 'source-hashes.json.gz'),
    'changes_from_instrumented_baseline':changed, 'semantic_and_cost_probe_source_unchanged':True,
    'main_sources_modified':False, 'runner_sha256':sha(__file__), 'native_runtime_admitted':False,
    'scope':'Same ordered relation computation and retention. Only candidate lookup and owned indexing change; symbolic first-match order is preserved.'})
runner = (W / 'baseline_v1.py').read_text().replace("S = W / 'baseline-source-v1'", "S = W / 'candidate-source-v1'")
runner = runner.replace("O = W / 'baseline-validation-v1'", "O = W / 'candidate-validation-v1'")
runner = runner.replace("Path('/tmp/csgrs-circle-support-baseline-v1')", "Path('/tmp/csgrs-circle-support-candidate-v1')")
runner = runner.replace('baseline-preparation-v1', 'candidate-preparation-v1')
(W / 'candidate_v1.py').write_text(runner)
print(json.dumps({'candidate_production_paths': list(changed), 'test_sources_unchanged': True}), flush=True)
