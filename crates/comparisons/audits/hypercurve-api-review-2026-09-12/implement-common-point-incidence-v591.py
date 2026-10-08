from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent
assert json.loads((A/'algebraic-image-collapse-20260928-v590-reaped.json').read_text())['all_processes_reaped']
p=W/'hypercurve/src/bezier_offset.rs';s=p.read_text();base=json.loads((A/'recursive-point-incidence-broad-20260928-v587-sources.json').read_text());assert hashlib.sha256(p.read_bytes()).hexdigest()==base['hypercurve/src/bezier_offset.rs']
# Retain the domain assembly needed by the winding membership consumer.
q=s.index("impl BezierParallelPointQuery2<'_> {")
a=s.index('    fn visit_point_parameters(',q)
b=s.index('        let expanded = ',a);c=s.index('        let zero = Real::zero();',b)
domain=s[b:c].replace('incident.expanded_range(self.range, policy)','incident.expanded_range(&self.range, policy)').replace('unwrap_or(self.range)','unwrap_or(&self.range)')
# Remove the separate bivariate point enumerator; winding keeps its own equations.
a=s.index('    fn contains_parameter(',q);b=s.index("\n}\n\nimpl BezierParallelAlgebraicRay2",a)
s=s[:a]+s[b:]
s=s.replace('visit_recursive_point_parameters','visit_point_parameters')
a=s.index('    pub(crate) fn contains_point(',s.index('impl BezierParallelAlgebraicRay2'))
b=s.index('    /// Omits every transverse contact',a)
block=s[a:b]
old='        Ok(query\n            .visit_point_parameters(point, incident, policy, &mut |_| ControlFlow::Break(()))?\n            .map(|flow| flow.is_break()))'
new=domain+'        let retained = CurvePoint2::from(point.point_image().clone());\n        Ok(query\n            .visit_point_parameters(&retained, incident, domain, policy, &mut |_| ControlFlow::Break(()))?\n            .map(|flow| flow.is_break()))'
assert old in block;block=block.replace(old,new);s=s[:a]+block+s[b:]
a=s.index('    pub(crate) fn visit_point_incidence_evidence(');b=s.index('    fn source_circle_polynomial(',a)
block=s[a:b];c=block.index('        if let Classification::Uncertain(reason) = self.certify_source_frame_in_domain(')
block=block[:c]+'        let point = CurvePoint2::from(point.point_image().clone());\n        BezierParallelPointQuery2 {\n            parallel: self,\n            range,\n            frame: frame.as_deref(),\n        }\n        .visit_point_parameters(&point, incident, domain, policy, visitor)\n    }\n\n'
s=s[:a]+block+s[b:]
fixture=(A/'algebraic-image-collapsed-point-v589.rs').read_text()
fixture=fixture[:fixture.index('    #[test]\n    fn algebraic_image_incidence_retains_a_collapsed_parallel_domain()')]
fixture=fixture.replace('    fn check_algebraic_image_collapsed_parallel_domain(recursive: bool) {','    #[test]\n    fn algebraic_image_incidence_retains_a_collapsed_parallel_domain() {')
a=fixture.index('                let result = if recursive {');b=fixture.index('                assert_eq!(result.unwrap()',a)
fixture=fixture[:a]+'                let result = parallel.visit_point_incidence_evidence(&point, &range, None, false, &policy, &mut visitor);\n'+fixture[b:]
marker='    fn recursive_test_point(';assert s.count(marker)==1;s=s.replace(marker,fixture+'\n'+marker)
p.write_text(s)
print('Applied one common point enumerator and independent public collapsed-circle regression')
