from pathlib import Path
import io,shutil,subprocess,tarfile
workspace=Path('/home/tim/Documents/GitHub/workspace');audit=Path(__file__).resolve().parent
source=Path('/tmp/hypercurve-gcd-replay-2026-09-23');root=Path('/tmp/hypercurve-endpoint-sign-2026-09-23');root.mkdir()
for p in source.iterdir():
    if p.is_dir() and p.name not in ['hypercurve','hypersolve','dumps']:
        (root/p.name).symlink_to(p.resolve(),target_is_directory=True)
shutil.copytree(source/'hypercurve',root/'hypercurve')
repo=root/'hypersolve';repo.mkdir();archive=subprocess.check_output(['git','archive','74ad6b857e042c4f8d18f67a89f08b6668a8ad41'],cwd=workspace/'hypersolve')
with tarfile.open(fileobj=io.BytesIO(archive)) as stream:stream.extractall(repo,filter='data')
(root/'dumps').mkdir()
p=root/'hypercurve/src/bezier_offset.rs';p.write_text(p.read_text().replace(str(source),str(root)))
p=repo/'src/algebraic_fiber.rs';s=p.read_text();traced=(source/'hypersolve/src/algebraic_fiber.rs').read_text()
a=s.index('fn count_common_fiber_roots(');b=s.index('\nfn local_fiber_polynomial(',a);x=traced.index('fn count_common_fiber_roots(');y=traced.index('\nfn local_fiber_polynomial(',x)
s=s[:a]+traced[x:y].replace(str(source),str(root))+s[b:]
a=s.index('            let degree = row.len();',s.index('fn rational_local_subresultant_rows'))
a+=len('            let degree = row.len();')
s=s[:a]+'''
            if degree == 2 && row.iter().any(|c| c.len()>20) && proof.modulus().len()>=20 {
                static TRACE_LINEAR: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
                let id=TRACE_LINEAR.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
                let rational=|value:&Real| value.exact_rational_ref().map(ToString::to_string).unwrap_or_else(|| "nonrational".into());
                let data=format!("{{\\"base\\":{:?},\\"interval\\":{:?},\\"row\\":{:?}}}",
                    proof.modulus().iter().map(&rational).collect::<Vec<_>>(),
                    [rational(&proof.root.interval.lower),rational(&proof.root.interval.upper)],
                    row.iter().map(|c|c.iter().map(&rational).collect::<Vec<_>>()).collect::<Vec<_>>());
                std::fs::write(format!("/tmp/hypercurve-endpoint-sign-2026-09-23/dumps/linear-{id}.json"),data).unwrap();
                eprintln!("LINEAR id={id} coefficient_degrees={:?}",row.iter().map(|c|c.len()-1).collect::<Vec<_>>());
            }
'''+s[a:];p.write_text(s)
p=repo/'src/root_sign.rs';s=p.read_text();a=s.index(') -> Option<Ordering> {',s.index('pub fn sign_at_selected_root('))+len(') -> Option<Ordering> {')
s=s[:a]+'''
    static TRACE_SIGN: std::sync::atomic::AtomicUsize=std::sync::atomic::AtomicUsize::new(0);
    let trace_id=TRACE_SIGN.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
    let trace_high=defining.len()>=20;
    let start=std::time::Instant::now();
    if trace_high {
        let rational=|value:&Real| value.exact_rational_ref().map(ToString::to_string).unwrap_or_else(|| "nonrational".into());
        let data=format!("{{\\"base\\":{:?},\\"predicate\\":{:?},\\"interval\\":{:?}}}",
            defining.iter().map(&rational).collect::<Vec<_>>(),predicate.iter().map(&rational).collect::<Vec<_>>(),
            [rational(&interval.lower),rational(&interval.upper)]);
        std::fs::write(format!("/tmp/hypercurve-endpoint-sign-2026-09-23/dumps/sign-{trace_id}.json"),data).unwrap();
        eprintln!("SIGN begin id={trace_id} degrees=({},{})",defining.len()-1,predicate.len().saturating_sub(1));
    }
'''+s[a:]
needle='''    query_sign_from_variations(lower, upper)
}''';assert s.count(needle)==1
s=s.replace(needle,'''    let result=query_sign_from_variations(lower, upper);
    if trace_high { eprintln!("SIGN complete id={trace_id} result={result:?} elapsed={:?}",start.elapsed()); }
    result
}''');p.write_text(s)
s=(audit/'run-gcd-replay-trial-20260923.py').read_text().replace(str(source),str(root)).replace('nonph-gcd-replay-20260923-','nonph-endpoint-sign-20260923-').replace('timeout=35','timeout=45')
(audit/'run-endpoint-sign-trial-20260923.py').write_text(s)
print('Prepared clean committed solver plus owned diagnostics; dependency snapshots remain pinned.')
