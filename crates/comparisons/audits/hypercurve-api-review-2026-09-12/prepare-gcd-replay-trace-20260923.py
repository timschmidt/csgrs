from pathlib import Path
import shutil
root=Path('/tmp/hypercurve-gcd-replay-2026-09-23'); root.mkdir()
source=Path('/tmp/hypercurve-contact-isolation-2026-09-23')
for p in source.iterdir():
    if p.is_dir() and p.name not in ['hypercurve','hypersolve','dumps']:
        (root/p.name).symlink_to(p.resolve(),target_is_directory=True)
shutil.copytree(source/'hypercurve',root/'hypercurve')
shutil.copytree('/tmp/hypercurve-rational-sturm-2026-09-23/hypersolve',root/'hypersolve')
(root/'dumps').mkdir()
p=root/'hypercurve/src/bezier_offset.rs'
p.write_text(p.read_text().replace(str(source),str(root)))
p=root/'hypersolve/src/algebraic_fiber.rs'; s=p.read_text()
needle='''    let gcd = local_polynomial_greatest_common_divisor(first, second, field)?;
    count_local_polynomial_roots(gcd, fiber_lower, fiber_upper, endpoints, field)'''
assert s.count(needle)==1
s=s.replace(needle,'''    static TRACE_GCD: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let trace_id = TRACE_GCD.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let trace_high = field.modulus().len() >= 20;
    let start = std::time::Instant::now();
    if trace_high {
        let rational = |value: &Real| value.exact_rational_ref().map(ToString::to_string).unwrap_or_else(|| "nonrational".into());
        let matrix = |polynomial: &[LocalFieldElement]| polynomial.iter().map(|coefficient| coefficient.numerator.iter().map(&rational).collect::<Vec<_>>()).collect::<Vec<_>>();
        let denominators = first.iter().chain(&second).any(|c| c.denominator.is_some());
        let equal = first.len() == second.len() && first.iter().zip(&second).all(|(a,b)| a.numerator == b.numerator && a.denominator == b.denominator);
        let data = format!("{{\\"base\\":{:?},\\"base_interval\\":{:?},\\"first\\":{:?},\\"second\\":{:?},\\"fiber_interval\\":{:?},\\"denominators\\":{},\\"point_witness\\":{},\\"equal\\":{}}}",
            field.modulus().iter().map(&rational).collect::<Vec<_>>(),
            [rational(&field.root.interval.lower),rational(&field.root.interval.upper)],
            matrix(&first),matrix(&second),[rational(fiber_lower),rational(fiber_upper)],
            denominators,field.root.interval.exact_root.is_some(),equal);
        std::fs::write(format!("/tmp/hypercurve-gcd-replay-2026-09-23/dumps/gcd-{trace_id}.json"),data).unwrap();
        eprintln!("GCD begin id={trace_id} base={} first={} second={} same={equal} denominators={denominators}",field.modulus().len()-1,first.len()-1,second.len()-1);
    }
    let gcd = local_polynomial_greatest_common_divisor(first, second, field)?;
    if trace_high { eprintln!("GCD complete id={trace_id} degree={} elapsed={:?}",gcd.len()-1,start.elapsed()); }
    count_local_polynomial_roots(gcd, fiber_lower, fiber_upper, endpoints, field)''')
p.write_text(s)
audit=Path(__file__).resolve().parent
s=(audit/'run-contact-isolation-sturm-20260923.py').read_text().replace(str(source),str(root)).replace('nonph-contact-isolation-20260923-','nonph-gcd-replay-20260923-').replace('timeout=120','timeout=35')
(audit/'run-gcd-replay-trial-20260923.py').write_text(s)
print('Prepared owned diagnostic source with pinned dependency snapshots.')
