"""Remove incidental whole-file formatting without changing the tested Rust."""
from pathlib import Path
import hashlib, json, subprocess
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda text: hashlib.sha256(text.encode()).hexdigest()
trial = json.loads((E/'qualification-v1/result.json').read_text())
assert trial['commands'][-1]['name'] == 'foundations' and trial['commands'][-1]['exit_code'] == 101
assert all(row['exit_code'] == 0 for row in trial['commands'][:-1])
record = {}
for name in ['facts.rs','linear_algebra.rs']:
    path = ROOT/'crates/hyperreal/src/real/arithmetic'/name
    current = path.read_text()
    original = subprocess.check_output(['git','show',f'f1092edb1:crates/hyperreal/src/real/arithmetic/{name}'],cwd=ROOT,text=True)
    if name == 'facts.rs':
        revised = original.replace('    /// scales and avoid introducing unrelated optimization passes.','    /// scales. Opposite identical two-factor products cancel before expansion,\n    /// including reversed factor order; structural equality is the proof.',1)
        start = current.index('    /// A fixed product identity,')
        end = current.index('    fn sin_pi_pythagorean_product_sum',start)
        helper = current[start:end]
        start = revised.index('    pub fn active_signed_product_sum')
        end = revised.index('    fn sin_pi_pythagorean_product_sum',start)
        body = revised[start:end].replace('        let mut total = None;', '        if Self::opposite_products_cancel(positive_terms, terms) {\n            return Real::zero();\n        }\n\n        let mut total = None;',1)
        revised = revised[:start]+body+helper+revised[end:]
    else:
        revised = original.replace('    /// Return `a * b - c * d`.\n','    /// Return `a * b - c * d`.\n    ///\n    /// Structurally identical products cancel exactly, including reversed\n    /// factor order, without refining the factors.\n',1)
        old = '    fn signed_product_sum2(signs: [bool; 2], terms: [[&Real; 2]; 2]) -> Real {\n'
        revised = revised.replace(old,old+'        if Self::opposite_products_cancel(signs, terms) {\n            return Real::zero();\n        }\n',1)
    formatted = []
    for text in [original,current,revised]:
        run = subprocess.run(['rustfmt','--edition','2024','--config-path',str(ROOT/'crates/.rustfmt.toml'),'--emit','stdout'],input=text,text=True,capture_output=True,check=True)
        formatted.append(run.stdout)
    assert formatted[1] == formatted[2]
    record[str(path.relative_to(ROOT))] = {'baseline_sha256':sha(original),'tested_v1_sha256':sha(current),'final_sha256':sha(revised),'formatted_rust_identical':True,'baseline_already_matches_standalone_formatter':original==formatted[0]}
    path.write_text(revised)
(E/'narrow-format-v1.json').write_text(json.dumps({'files':record,'scope':'Both included files already differ from standalone rustfmt on the baseline. Preserve existing unrelated formatting. New source produces exactly the same rustfmt output as the fully formatted candidate; Cargo workspace formatting remains required. Final checks run on the narrowed patch.'},indent=2)+'\n')
print(json.dumps(record,indent=2))
