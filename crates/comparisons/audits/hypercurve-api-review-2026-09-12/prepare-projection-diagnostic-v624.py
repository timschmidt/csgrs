from pathlib import Path
import json

A = Path(__file__).resolve().parent
W = A.parent
assert json.loads((A / 'retained-structural-pair-diagnostic-20260928-v623-reaped.json').read_text())['outer_exit_code'] == 1
source = (A / 'retained-structural-pair-diagnostic-v623.rs').read_text()
start = source.index('fn project_parallel_intersection_system(')
end = source.index('\nfn bivariate_system_may_have_component(', start)
helper = source[start:end]
helper = helper.replace('let parallel = match resultant_parameter_projection(parallel_report, domains[0], policy)? {', '''eprintln!("PAIR_PROJECT first_report={:?} coefficients={}", parallel_report.status, parallel_report.resultant_coefficients.len());
    let parallel = match resultant_parameter_projection(parallel_report, domains[0], policy)? {''')
helper = helper.replace('let other = match resultant_parameter_projection(other_report, domains[1], policy)? {', '''eprintln!("PAIR_PROJECT second_report={:?} coefficients={}", other_report.status, other_report.resultant_coefficients.len());
    let other = match resultant_parameter_projection(other_report, domains[1], policy)? {''')
helper = helper.replace('Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),', '''Classification::Uncertain(reason) => {
            eprintln!("PAIR_PROJECT uncertain line={} reason={reason:?}", line!());
            return Ok(Classification::Uncertain(reason));
        },''')
source = source[:start] + helper + source[end:]
(A / 'retained-structural-pair-diagnostic-v624.rs').write_text(source)
source = (W / 'hypercurve/src/rational_bezier_general.rs').read_text()
start = source.index('pub(crate) fn resultant_parameter_projection(')
end = source.index('\n/// Extends an already certified finite projection', start)
helper = source[start:end]
helper = helper.replace('return Ok(Classification::Uncertain(reason));', 'eprintln!("RESULTANT_DOMAIN uncertain line={} reason={reason:?}", line!()); return Ok(Classification::Uncertain(reason));')
(A / 'retained-structural-rational-diagnostic-v624.rs').write_text(source[:start] + helper + source[end:])
source = (W / 'hypercurve/src/bezier_split.rs').read_text()
start = source.index('    pub(crate) fn finite_roots(')
end = source.index('\nfn parameter_is_in_ordered_range(', start)
helper = source[start:end]
helper = helper.replace('return Ok(Classification::Uncertain(reason));', 'eprintln!("FINITE_ROOTS uncertain line={} reason={reason:?}", line!()); return Ok(Classification::Uncertain(reason));')
helper = helper.replace('let mut retained = Vec::with_capacity(roots.len());', 'eprintln!("FINITE_ROOTS isolated count={}", roots.len());\n            let mut retained = Vec::with_capacity(roots.len());')
(A / 'retained-structural-split-diagnostic-v624.rs').write_text(source[:start] + helper + source[end:])
source = (W / 'hypercurve/src/bezier_parameter.rs').read_text()
start = source.index('fn isolate_roots_in_interval(')
end = source.index('\n/// Materializes roots already representable', start)
helper = source[start:end]
helper = helper.replace('None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),', '''None => {
                    eprintln!("INTERVAL_ROOTS endpoint_sign degree={}", coefficients.len().saturating_sub(1));
                    return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
                },''')
helper = helper.replace('return Ok(Classification::Uncertain(reason));', 'eprintln!("INTERVAL_ROOTS uncertain line={} reason={reason:?}", line!()); return Ok(Classification::Uncertain(reason));')
(A / 'retained-structural-parameter-diagnostic-v624.rs').write_text(source[:start] + helper + source[end:])
driver = (A / 'probe-retained-structural-pair-diagnostic-20260928-v623.py').read_text()
driver = driver.replace('retained-structural-pair-diagnostic-20260928-v623', 'retained-structural-pair-diagnostic-20260928-v624').replace('retained-structural-pair-diagnostic-v623.rs', 'retained-structural-pair-diagnostic-v624.rs')
driver = driver.replace("composition=prior", "assert json.loads((A/'retained-structural-pair-diagnostic-20260928-v623-reaped.json').read_text())['outer_exit_code']==1\ncomposition=prior")
extras = ['rational_bezier_general', 'bezier_split', 'bezier_parameter']
labels = ['rational', 'split', 'parameter']
extra_files = {f'hypercurve/src/{name}.rs':f'retained-structural-{label}-diagnostic-v624.rs' for name,label in zip(extras,labels)}
driver = driver.replace('guard=json.loads(', 'extra_files='+repr(extra_files)+'\nextra_sha={name:hashlib.sha256((A/path).read_bytes()).hexdigest()for name,path in extra_files.items()}\nguard=json.loads(', 1)
driver = driver.replace(" manifest[name]=hashlib.sha256(data).hexdigest()", " if name in extra_files:data=(A/extra_files[name]).read_bytes()\n manifest[name]=hashlib.sha256(data).hexdigest()")
driver = driver.replace('def verify():', 'def verify():\n for name,path in extra_files.items():assert hashlib.sha256((A/path).read_bytes()).hexdigest()==extra_sha[name]')
driver = driver.replace(", 'hypercurve':['bezier_offset::retained_structural_pair_domain_regression::retained_structural_correspondence_preserves_off_diagonal_contact_domains']", '')
driver = driver.replace("[cargo,'test','--lib','--test'", "[cargo,'test','--test'")
(A / 'probe-retained-structural-pair-diagnostic-20260928-v624.py').write_text(driver)
print('Prepared V624 integration-only projection diagnostic')
