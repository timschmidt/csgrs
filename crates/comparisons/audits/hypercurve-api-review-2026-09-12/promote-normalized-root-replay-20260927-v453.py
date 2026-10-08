from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent
report=json.loads((A/'normalized-root-reoffset-20260927-v452-terminal.json').read_text())
assert report['all_processes_reaped'] and not report['qualification_complete']
assert len(report['cases'])==1 and report['cases'][0]['returncode']==124
oracle=json.loads((A/'root-sign-tower-branches-20260927-v451-terminal.json').read_text())
assert oracle['qualification_complete'] and oracle['all_processes_reaped']
base=json.loads((A/report['source_manifest']).read_text());archive=Path(report['source_directory'])
files=['hyperreal/src/computable/node/quadratic_tower.rs','hypersolve/src/root_sign.rs','hypersolve/src/root_isolation.rs','hypercurve/tests/hypercurve_curve_region_promotion.rs']
for name in files:
 assert hashlib.sha256((W/name).read_bytes()).hexdigest()==base[name],name
 assert hashlib.sha256((archive/name).read_bytes()).hexdigest()==report['trial'][name],name
values={name:(archive/name).read_text() for name in files}
name=files[0];anchor='mod quadratic_tower_tests {\n    use super::*;\n';assert values[name].count(anchor)==1
values[name]=values[name].replace(anchor,anchor+(A/'quadratic-norm-regressions-20260927-v453.rs').read_text(),1)
name=files[1];anchor='\n#[cfg(test)]\nmod captured_quadratic_field_queries {';assert values[name].count(anchor)==1
values[name]=values[name].split(anchor)[0].rstrip()+'\n'
anchor='mod tests {\n    use super::*;\n    use proptest::prelude::*;\n';assert values[name].count(anchor)==1
values[name]=values[name].replace(anchor,anchor+(A/'normalized-query-regression-20260927-v453.rs').read_text(),1)
values[name]=values[name].replace('gcd_monic_normalize','monic_normalize')
anchor='    if second.len() >= first.len() {\n        second = polynomial_div_rem(second, &first, PredicatePolicy::STRICT)?.1;\n    }\n'
assert values[name].count(anchor)==1;values[name]=values[name].replace(anchor,'',1)
anchor='        if second.is_empty() {\n            return Some(chain);\n        }\n'
replacement=anchor+"        // The polynomial part of (P'Q)/P has no pole. Remove it before\n        // recording the first numerator, after resolving a zero query without\n        // imposing an unnecessary degree decision on the defining equation.\n        if chain.len() == 1 && second.len() >= first.len() {\n            second = polynomial_div_rem(second, &first, PredicatePolicy::STRICT)?.1;\n            continue;\n        }\n"
assert values[name].count(anchor)==1;values[name]=values[name].replace(anchor,replacement,1)
anchor='        let leading_sign = sign(second.last()?)?;'
replacement='''        // Divide by the absolute leading coefficient: retain every sign
        // while removing field units before they compound in later remainders.
'''+anchor
assert values[name].count(anchor)==1;values[name]=values[name].replace(anchor,replacement,1)
name=files[2];values[name]=values[name].replace('gcd_monic_normalize','monic_normalize')
old='''/// Monic-normalizes a GCD whose producing integer or Euclidean kernel has
/// already returned canonical coefficient storage.'''
new='''/// Monic-normalizes a polynomial whose producing kernel has already
/// returned canonical coefficient storage. The root set is preserved;
/// callers needing the polynomial's sign must restore its leading sign.'''
assert values[name].count(old)==1;values[name]=values[name].replace(old,new,1)
name=files[3]
a=values[name].index('    for (corner, candidate, result) in &filleted {');b=values[name].index('    for (corner, candidate, filleted) in filleted {',a)
values[name]=values[name][:a]+values[name][b:]
anchor='''        let fillet_circle_count = filleted.boundary_loops()
            .iter()
            .flat_map(|boundary| boundary.curves())'''
replacement='''        // An incident extension can regularize into several loops. The
        // inserted circle's chart ownership is independent of loop order.
'''+anchor
assert values[name].count(anchor)==1;values[name]=values[name].replace(anchor,replacement,1)
for name,value in values.items():
 (W/name).write_text(value)
 print(name)
