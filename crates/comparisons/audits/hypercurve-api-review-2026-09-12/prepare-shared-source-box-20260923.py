from pathlib import Path
import shutil
source=Path('/tmp/hypercurve-endpoint-sign-2026-09-23');root=Path('/tmp/hypercurve-shared-source-box-2026-09-23');root.mkdir();audit=Path(__file__).resolve().parent
for p in source.iterdir():
 if p.is_dir() and p.name not in ['hypercurve','hypersolve','dumps']:(root/p.name).symlink_to(p.resolve(),target_is_directory=True)
for name in ['hypercurve','hypersolve']:shutil.copytree(source/name,root/name)
(root/'dumps').mkdir()
for name in ['hypercurve/src/bezier_offset.rs','hypersolve/src/algebraic_fiber.rs','hypersolve/src/root_sign.rs']:
 p=root/name;p.write_text(p.read_text().replace(str(source),str(root)))
p=root/'hypercurve/src/bezier_offset.rs';s=p.read_text();a=s.index('    fn interval_with_coefficient_precision(\n',s.index('impl BezierRecursiveQuadraticValue2'));b=s.index('\n    fn interval(&self, refinement_steps:',a)
s=s[:a]+'''    fn interval_with_coefficient_precision(
        &self,
        refinement_steps: usize,
        coefficient_precision: Option<i32>,
    ) -> Option<RealInterval> {
        // Every coefficient and radicand belongs to this tower's shared base.
        // Refine each selected source once, then evaluate the complete value
        // in that same certified box instead of refining every leaf again.
        let (base, _) = self.field().base_and_extension_path();
        let sources = base.sources.iter().zip(&base.source_real_witnesses)
            .map(|(source,witness)| {
                if witness.is_some() { source.clone() }
                else { refined_represented_root(source,refinement_steps) }
            }).collect::<Vec<_>>();
        self.interval_over_source_box(&sources,coefficient_precision)
    }
'''+s[b:];p.write_text(s)
s=(audit/'run-endpoint-sign-trial-20260923.py').read_text().replace(str(source),str(root)).replace('nonph-endpoint-sign-20260923-','nonph-shared-source-box-20260923-').replace('timeout=45','timeout=120')
(audit/'run-shared-source-box-trial-20260923.py').write_text(s)
print('Prepared a separate diagnostic trial; qualified snapshots remain unchanged.')
