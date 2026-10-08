from pathlib import Path
import hashlib, json, shutil, subprocess

A = Path(__file__).resolve().parent
source = Path('/tmp/hypercurve-recognized-circle-evidence-v4-2026-09-23')
root = Path('/tmp/hypercurve-circle-closure-probe5-2026-09-23')
assert json.loads((A/'recognized-circle-evidence-20260923-v4-terminal.json').read_text())['all_processes_reaped']
assert not root.exists()
bindings = json.loads((A/'recognized-circle-evidence-20260923-v4-sources.json').read_text())
for name, sha in bindings.items():
    assert hashlib.sha256((source/name).read_bytes()).hexdigest() == sha, name
    (root/name).parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source/name, root/name)

p = root/'hypercurve/src/bezier_region.rs'
s = p.read_text()
a = s.index('    fn nonrepresented_chord_and_retained_rational_arc_share_the_fillet_kernel()')
b = s.index('\n    #[test]', a)
test = s[a:b]
test = test.replace('                            let fragments = filleted.boundary_loops()[0].fragments();\n', '')
test = test.replace('                            for (index, fragment) in fragments.iter().enumerate() {', '                            for boundary in filleted.boundary_loops() {\n                            let fragments = boundary.fragments();\n                            for (index, fragment) in fragments.iter().enumerate() {', 1)
test = test.replace('                            assert!(fillet_spans >= 1);\n                            assert!(chord_adjacencies >= 1);', '''                            }
                            let counts: Vec<_> = filleted.boundary_loops().iter().map(|boundary| {
                                let mut counts = [0usize; 5];
                                for fragment in boundary.fragments() {
                                    counts[match fragment {
                                        BezierSplitFragment2::Materialized { curve: BezierSubcurve2::RationalQuadratic(_), .. } => 0,
                                        BezierSplitFragment2::SelectedFiber(_) => 1,
                                        BezierSplitFragment2::AlgebraicChord(_) => 2,
                                        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => 3,
                                        _ => 4,
                                    }] += 1;
                                }
                                counts
                            }).collect();
                            eprintln!("FILLET-BOUNDARIES radius={:?} policy={policy:?} reversed={reversed} mode={mode:?} fillet={fillet_spans} chord-adjacencies={chord_adjacencies} counts={counts:?}", radius.to_f64_lossy());''')
s = s[:a]+test+s[b:]
p.write_text(s)

p = root/'hypersolve/src/algebraic_fiber/subresultant.rs'
s = p.read_text()
s = s.replace('    use AlgebraicFiberSubresultantError as Error;', '''    use AlgebraicFiberSubresultantError as Error;
    let report = |stage: &str, error: LocalFieldError| {
        eprintln!("SUBRESULTANT-FAIL stage={stage} order={order} first={} second={} error={error:?}", first.len(), second.len());
        error
    };''', 1)
s = s.replace('local_fiber_polynomial(fiber_equation, CurveResultantParameter::First, field)?', 'local_fiber_polynomial(fiber_equation, CurveResultantParameter::First, field).map_err(|e| report("fiber", e))?', 1)
s = s.replace('let first = reduce(first)?;', 'let first = reduce(first).map_err(|e| report("first-reduction", e))?;', 1)
s = s.replace('let second = reduce(second)?;', 'let second = reduce(second).map_err(|e| report("second-reduction", e))?;', 1)
s = s.replace('                    field,\n                )?);', '                    field,\n                ).map_err(|e| report("determinant", e))?);', 1)
s = s.replace('local_polynomial_clear_denominators(flattened, field)?', 'local_polynomial_clear_denominators(flattened, field).map_err(|e| report("denominators", e))?', 1)
p.write_text(s)

p = root/'hypersolve/src/algebraic_fiber.rs'
s = p.read_text()
a = s.index('    fn is_zero_polynomial(&mut self, polynomial: &[Real])')
b = s.index('\n    fn ', a+8)
method = s[a:b]
start = method.index('{')+1
end = method.rindex('}')
body = method[start:end]
method = method[:start]+'''\n        let result = (|| {'''+body+'''
        })();
        if result.is_err() {
            static PRINTED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            if PRINTED.fetch_add(1, std::sync::atomic::Ordering::Relaxed) < 8 {
                eprintln!("LOCAL-ZERO-FAIL result={result:?} length={} rational={} modulus={} root-witness={} values={:?}", polynomial.len(), polynomial.iter().filter(|x| x.exact_rational_ref().is_some()).count(), self.modulus().len(), self.root.exact_value.is_some(), polynomial.iter().take(8).map(Real::to_f64_lossy).collect::<Vec<_>>());
                for line in std::backtrace::Backtrace::force_capture().to_string().lines().take(28) { eprintln!("{line}"); }
            }
        }
        result
    }'''+method[end+1:]
# Avoid depending on the witness field's private spelling in this diagnostic.
method = method.replace(' root-witness={}', '').replace(', self.root.exact_value.is_some()', '')
s = s[:a]+method+s[b:]
p.write_text(s)

changed = ['hypercurve/src/bezier_region.rs', 'hypersolve/src/algebraic_fiber.rs', 'hypersolve/src/algebraic_fiber/subresultant.rs']
subprocess.run(['/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt', '--edition', '2024', '--config', 'skip_children=true', *(str(root/name) for name in changed)], check=True)
for name in changed: bindings[name] = hashlib.sha256((root/name).read_bytes()).hexdigest()
for name, sha in bindings.items(): assert hashlib.sha256((root/name).read_bytes()).hexdigest() == sha, name
(A/'circle-closure-20260923-probe5-sources.json').write_text(json.dumps(bindings, indent=2)+'\n')
runner = (A/'run-circle-closure-probe4-20260923.py').read_text().replace('probe4', 'probe5')
(A/'run-circle-closure-probe5-20260923.py').write_text(runner)
print('Prepared physical, bounded diagnostic snapshot:', len(bindings), 'files')
