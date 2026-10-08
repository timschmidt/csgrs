from pathlib import Path
import json

A = Path(__file__).resolve().parent
assert json.loads((A / 'retained-structural-pair-diagnostic-20260928-v621-reaped.json').read_text())['outer_exit_code'] == 1
test = (A / 'retained-structural-pair-domain-test-v615.rs').read_text()
old = '''            let ranges = [
                CurveParameterRange2::new_validated(Real::zero().into(), half.clone().into()),
                CurveParameterRange2::new_validated(half.clone().into(), Real::one().into()),
            ];'''
new = '''            let contact = &reference.contacts()[0];
            let first = contact.first_parameter();
            let second = contact.second_parameter();
            let (lower, upper) = match first.cmp_by_refinement(second, &policy).unwrap() {
                Classification::Decided(Ordering::Less) => (first, second),
                Classification::Decided(Ordering::Greater) => (second, first),
                _ => panic!("the reference contact must be off the diagonal"),
            };
            let Classification::Decided(split) =
                lower.strict_scalar_between_ordered(upper, &policy).unwrap()
            else {
                panic!("distinct crossing parameters must admit a separating cut");
            };
            let ranges = [
                CurveParameterRange2::new_validated(Real::zero().into(), split.clone().into()),
                CurveParameterRange2::new_validated(split.into(), Real::one().into()),
            ];'''
assert test.count(old) == 1
test = test.replace(old, new).replace('the two halves contain one off-diagonal crossing', 'the separated domains contain one off-diagonal crossing')
(A / 'retained-structural-pair-domain-test-v622.rs').write_text(test)
source = (A / 'retained-structural-pair-diagnostic-v620.rs').read_text()
old_test = (A / 'retained-structural-pair-domain-test-v615.rs').read_text()
assert source.count(old_test) == 1
source = source.replace(old_test, test)
start = source.index('fn project_parallel_pair_without_components_in_domain(')
end = source.index('\nfn project_unit_parallel_pair_intersection_system(', start)
helper = source[start:end]
helper = helper.replace('Classification::Uncertain(_) => return Ok(None),', '''Classification::Uncertain(reason) => {
            eprintln!("PAIR_SATURATION uncertain line={} reason={reason:?}", line!());
            return Ok(None);
        },''')
helper = helper.replace('return Ok(None);', 'eprintln!("PAIR_SATURATION none line={}", line!()); return Ok(None);')
helper = helper.replace('_ => return Ok(None),', '''candidate => {
                eprintln!("PAIR_SATURATION residual failure line={} decided={}", line!(), matches!(candidate, Classification::Decided(_)));
                return Ok(None);
            },''')
helper = helper.replace('let source_component_removed = source_residual.is_some();', '''let source_component_removed = source_residual.is_some();
    eprintln!("PAIR_SATURATION source_removed={source_component_removed}");''')
helper = helper.replace('let initial_candidates = match project(&residual_equations)? {', '''eprintln!("PAIR_SATURATION before_initial degrees={:?}", residual_equations.each_ref().map(bivariate_storage_bidegree_sum));
    let initial_candidates = match project(&residual_equations)? {''')
helper = helper.replace('let candidates = if matches!(', '''eprintln!("PAIR_SATURATION initial_kind={:?}", std::mem::discriminant(&initial_candidates));
    let candidates = if matches!(''')
helper = helper.replace('let constraint = match parameter_domain_constraint(', '''eprintln!("PAIR_SATURATION norm_constraint");
        let constraint = match parameter_domain_constraint(''')
helper = helper.replace('let selection = match select_parameter_component_in_domain(', '''eprintln!("PAIR_SATURATION select_component");
            let selection = match select_parameter_component_in_domain(''')
source = source[:start] + helper + source[end:]
(A / 'retained-structural-pair-diagnostic-v622.rs').write_text(source)
driver = (A / 'probe-retained-structural-pair-diagnostic-20260928-v621.py').read_text()
driver = driver.replace('retained-structural-pair-diagnostic-20260928-v621', 'retained-structural-pair-diagnostic-20260928-v622').replace('retained-structural-pair-diagnostic-v620.rs', 'retained-structural-pair-diagnostic-v622.rs')
driver = driver.replace("composition=prior", "assert json.loads((A/'retained-structural-pair-diagnostic-20260928-v621-reaped.json').read_text())['outer_exit_code']==1\ncomposition=prior")
(A / 'probe-retained-structural-pair-diagnostic-20260928-v622.py').write_text(driver)
journal = A / 'current-stationary-fillet-20260927.md'
journal.write_text('''## V621 reaped; bounded V622 diagnostic prepared

V617 outer23621 and V619 outer92647 reaped1; scoped trace locates offset regularization failure in adjacent same-image parallel pair. V621 outer84585 reaped1: structural correspondence is admitted for both construction and subsequent offset, but finite-domain saturation returns None only for the offset. The extra V615 domain fixture had an invalid half-domain assumption (expected one crossing but zero); V622 uses a certified separating cut between distinct reference crossing parameters. This is a fixture correction, not a production failure. V622 adds bounded branch diagnostics to the finite-domain saturation helper. Production remains unchanged22pendingfiles; V613 unpromoted. No V605 seal/commit. V600 remains deferred. NextunusedartifactV623.

''' + journal.read_text())
print('Prepared V622 copied-source diagnostic and corrected independent-domain guard')
