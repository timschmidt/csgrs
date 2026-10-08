from pathlib import Path
import shutil

audit=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-parameter-order-2026-09-23')
root=Path('/tmp/hypercurve-contact-isolation-2026-09-23')
assert not root.exists()
root.mkdir(); (root/'dumps').mkdir()
for path in source.iterdir():
    if path.name!='hypercurve' and path.is_dir(): (root/path.name).symlink_to(path.resolve(),target_is_directory=True)
repo=root/'hypercurve'; shutil.copytree(source/'hypercurve',repo)
(repo/'examples/nonph_parameter_order_probe_20260923.rs').unlink()
(repo/'examples/nonph_contact_isolation_probe_20260923.rs').write_bytes((audit/'boundary-api-20260923-nonph-probe.rs').read_bytes())
p=repo/'src/curve_region_boolean.rs'; s=p.read_text()
needle='''        for pair in &self.data.pairs {
            let result = self.pair_result(pair)?;
            if let Some(blocker) = result.blockers.first() {'''
assert s.count(needle)==1
s=s.replace(needle,'''        for pair in &self.data.pairs {
            eprintln!("PAIR begin carriers=({}, {}) fragments=({}, {}) families=({:?},{:?}) adjacent={}",
                pair.first_carrier_index, pair.second_carrier_index,
                self.data.carriers[pair.first_carrier_index].fragment_index,
                self.data.carriers[pair.second_carrier_index].fragment_index,
                self.data.carriers[pair.first_carrier_index].family,
                self.data.carriers[pair.second_carrier_index].family,
                self.authored_carriers_are_adjacent(pair));
            let result = self.pair_result(pair)?;
            eprintln!("PAIR complete carriers=({}, {}) contacts={} overlaps={} blockers={}",
                pair.first_carrier_index, pair.second_carrier_index,
                result.contacts.len(), result.overlaps.len(), result.blockers.len());
            if let Some(blocker) = result.blockers.first() {''')
p.write_text(s)
p=repo/'src/bezier_offset.rs'; s=p.read_text()
needle='''    let report = isolate_bivariate_fiber_roots_at_algebraic_parameter_complete(
        incidence,
        CurveResultantParameter::First,
        &retained_root,
        lower,
        upper,'''
assert s.count(needle)==1
trace='''    static TRACE_FIBER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let trace_id = TRACE_FIBER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let rational = |value: &Real| value.exact_rational_ref().map(ToString::to_string).unwrap_or_else(|| "nonrational".into());
    let coefficients = incidence.coefficients.iter().map(|row| row.iter().map(&rational).collect::<Vec<_>>()).collect::<Vec<_>>();
    let base = retained_root.polynomial_coefficients.iter().map(&rational).collect::<Vec<_>>();
    let data = format!("{{\\"base\\":{:?},\\"base_interval\\":{:?},\\"incidence\\":{:?},\\"fiber_interval\\":{:?}}}",
        base, [rational(&retained_root.interval.lower), rational(&retained_root.interval.upper)],
        coefficients, [rational(lower), rational(upper)]);
    std::fs::write(format!("/tmp/hypercurve-contact-isolation-2026-09-23/dumps/fiber-{trace_id}.json"), data).unwrap();
    eprintln!("FIBER begin id={trace_id} degree={} shape=({},{}) bounds=({:?},{:?})",
        retained_root.polynomial_coefficients.len()-1, incidence.coefficients.len(),
        incidence.coefficients.iter().map(Vec::len).max().unwrap_or(0), lower.to_f64_lossy(), upper.to_f64_lossy());
'''
s=s.replace(needle,trace+needle)
needle='''    if report.certainty == PredicateCertainty::Approximate {
        policy.observe_approximate_512();
    }
    Ok(match report.status {
        AlgebraicFiberRootIsolationStatus::Isolated => {'''
assert s.count(needle)==1
s=s.replace(needle,'''    eprintln!("FIBER complete id={trace_id} status={:?} roots={}", report.status, report.intervals.len());
'''+needle)
p.write_text(s)
runner=(audit/'run-nonph-order-trial-20260923.py').read_text()
runner=runner.replace('/tmp/hypercurve-parameter-order-2026-09-23','/tmp/hypercurve-contact-isolation-2026-09-23')
runner=runner.replace("'nonph-order-' + sys.argv[1] + '-20260923'", "'nonph-contact-isolation-20260923-' + sys.argv[1]")
runner=runner.replace('nonph_parameter_order_probe_20260923','nonph_contact_isolation_probe_20260923')
runner+='\nshutil.copytree(root / "dumps", audit / (prefix + "-dumps"))\n'
(audit/'run-contact-isolation-trial-20260923.py').write_text(runner)
print('Prepared isolated trace source; main Hypercurve and foreign Hyperreal working sources untouched.')
