from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent
manifest=json.loads((A/'finite-field-root-identity-20260928-v563-sources.json').read_text())
paths=['hypercurve/src/bezier_offset.rs','hypercurve/src/curve_fillet.rs']
for name in paths:assert hashlib.sha256((W/name).read_bytes()).hexdigest()==manifest[name],name
p=W/paths[0];s=p.read_text()
a=s.index('impl BezierParallelAlgebraicRay2 {');b=s.index('\nfn retained_point_evidence_equality_by_refinement(',a);ray=s[a:b]
markers=[
 ('    fn system(', '    fn projected_parameters('),
 ('    fn projected_parameters(', '    fn expression_sign_at_candidate('),
 ('    fn expression_sign_at_candidate(', '    fn parameter_orders('),
 ('    fn contains_parameter(', '    fn visit_point_parameters_from_systems('),
 ('    fn visit_point_parameters_from_systems(', '    pub(crate) fn contains_point('),
 ('    fn visit_point_parameters(', '    /// Omits every transverse contact'),
]
spans=[];methods=[]
for start,end in markers:
 i=ray.index(start);j=ray.index(end,i);spans.append((i,j));methods.append(ray[i:j])
for i,j in sorted(spans,reverse=True):ray=ray[:i]+ray[j:]
query=''.join(methods).replace('&self.range','self.range')
needle='        let differential = self.parallel.differential()?;'
assert query.count(needle)==1
query=query.replace(needle,needle+'''
        let (tangent_x, tangent_y) = self.frame
            .map(|frame| (&frame.x[..], &frame.y[..]))
            .unwrap_or((&differential.tangent_x, &differential.tangent_y));''')
query=query.replace('&differential.tangent_y, &(-factor_x.clone())','tangent_y, &(-factor_x.clone())').replace('&differential.tangent_x, factor_y','tangent_x, factor_y')
query=query.replace('&parallel_speed_squared_polynomial(differential)', '&polynomial_add(&polynomial_multiply(tangent_x, tangent_x), &polynomial_multiply(tangent_y, tangent_y))')
needle='''        Ok(self
            .visit_point_parameters(point, incident, policy, &mut |_| ControlFlow::Break(()))?'''
assert ray.count(needle)==1
ray=ray.replace(needle,'''        let query = BezierParallelAlgebraicPointQuery2 {
            parallel: &self.parallel,
            range: &self.range,
            frame: None,
        };
        Ok(query
            .visit_point_parameters(point, incident, policy, &mut |_| ControlFlow::Break(()))?''')
needle='        let side_x = -direction_y.clone();'
assert ray.count(needle)==1
ray=ray.replace(needle,'''        let query = BezierParallelAlgebraicPointQuery2 {
            parallel: &self.parallel,
            range: &self.range,
            frame: None,
        };
'''+needle)
ray=ray.replace('self.system(', 'query.system(').replace('Self::projected_parameters(', 'BezierParallelAlgebraicPointQuery2::projected_parameters(').replace('self.expression_sign_at_candidate(', 'query.expression_sign_at_candidate(')
assert 'self.visit_point_parameters' not in ray
s=s[:a]+'''/// Borrowed point-incidence equations shared by membership and winding.
/// A regular cell may supply its oriented primitive normal without requiring
/// endpoint geometry, traversal, or a winding-ray carrier.
struct BezierParallelAlgebraicPointQuery2<'a> {
    parallel: &'a BezierParallel2,
    range: &'a CurveParameterRange2,
    frame: Option<&'a BezierAnalyticParallelTangentField2>,
}

impl BezierParallelAlgebraicPointQuery2<'_> {
'''+query+'}\n\n'+ray+s[b:]
# Retire the Cartesian-only regular-domain entry point.
a=s.index('    pub(crate) fn point_incidence_in_regular_domain(');b=s.index('    fn point_incidence_with_tangent_field(',a);s=s[:a]+s[b:]
# All existing general callers keep their raw/pointwise source semantics.
# Insert the new argument using balanced parentheses for each call site.
name='visit_point_incidence_evidence'
pos=0;insertions=[]
while True:
 i=s.find('.'+name+'(',pos)
 if i<0:break
 start=i+len(name)+2;depth=1;j=start;commas=[]
 while depth:
  c=s[j]
  if c=='(':depth+=1
  elif c==')':depth-=1
  elif c==','and depth==1:commas.append(j)
  j+=1
 assert len(commas)>=3,(i,commas)
 insertions.append(commas[2]+1);pos=j
for i in reversed(insertions):s=s[:i]+' false,'+s[i:]
a=s.index('    pub(crate) fn visit_point_incidence_evidence(');b=s.index('    fn source_circle_polynomial(',a);visitor=s[a:b]
visitor=visitor.replace('        policy: &CurveContext,\n', '        regular_domain: bool,\n        policy: &CurveContext,\n',1)
visitor=visitor.replace('&point, range, incident, false, policy, visitor,','&point, range, incident, regular_domain, policy, visitor,')
needle='        let CurvePoint2(CurvePointData2::Algebraic(point)) = point else {'
assert visitor.count(needle)==1
visitor=visitor.replace(needle,'''        let expanded = match incident
            .map(|incident| incident.expanded_range(range, policy))
            .transpose()?
        {
            Some(Classification::Decided(range)) => Some(range),
            Some(Classification::Uncertain(reason)) => return Ok(Classification::Uncertain(reason)),
            None => None,
        };
        let domain = CurveParameterDomain2::new(
            expanded.as_ref().unwrap_or(range),
            incident.map(BezierParallelIncidentDomain2::parameter_ray),
        );
        let frame = || {
            if regular_domain {
                self.source_tangent_field_in_regular_domain(domain, policy)
            } else {
                Ok(Classification::Decided(None))
            }
        };
'''+needle)
i=visitor.index('                    let expanded = match incident');j=visitor.index('                    .map(|result| {',i)
visitor=visitor[:i]+'''                    let frame = match frame()? {
                        Classification::Decided(frame) => frame,
                        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                    };
                    self.point_incidence_with_tangent_field(point, domain, frame.as_deref(), policy)
'''+visitor[j:]
visitor=visitor.replace('point.visit_incidence_on_parallel(self, range, incident, policy, visitor)', 'point.visit_incidence_on_parallel(self, range, incident, regular_domain, policy, visitor)')
i=visitor.index('        // Membership does not evaluate endpoints.');j=visitor.index('\n    }\n',i)
visitor=visitor[:i]+'''        let frame = match frame()? {
            Classification::Decided(frame) => frame,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if let Classification::Uncertain(reason) = self.certify_source_frame_in_domain(
            SelectedThirdAxisDomain2::Finite(domain.finite),
            frame.as_deref(),
            policy,
        )? {
            return Ok(Classification::Uncertain(reason));
        }
        BezierParallelAlgebraicPointQuery2 {
            parallel: self,
            range,
            frame: frame.as_deref(),
        }.visit_point_parameters(&point, incident, policy, visitor)'''+visitor[j:]
s=s[:a]+visitor+s[b:]
needle='''    /// uncertainty leaves certified witnesses but does not prove exhaustiveness.
    pub(crate) fn visit_point_incidence_evidence('''
assert s.count(needle)==1
s=s.replace(needle,'''    /// uncertainty leaves certified witnesses but does not prove exhaustiveness.
    /// `regular_domain` selects the normal of an owned regular source cell,
    /// including its one-sided endpoint limits. Incident extensions must still
    /// certify that they stay on this cell's source-normal sheet.
    pub(crate) fn visit_point_incidence_evidence(''')
# Preserve the same source-normal authority for correlated chord point evidence.
a=s.index('    fn visit_incidence_on_parallel(');b=s.index('    fn translated(',a);part=s[a:b]
part=part.replace('        policy: &CurveContext,\n','        regular_domain: bool,\n        policy: &CurveContext,\n',1)
part=part.replace('incident, false,','incident, regular_domain,')
needle='        let finite = expanded.as_ref().unwrap_or(range);'
assert part.count(needle)==1
part=part.replace(needle,needle+'''
        let frame = if regular_domain {
            match parallel.source_tangent_field_in_regular_domain(
                CurveParameterDomain2::new(finite, incident.map(BezierParallelIncidentDomain2::parameter_ray)),
                policy,
            )? {
                Classification::Decided(frame) => frame,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        } else {
            None
        };''')
part=part.replace('recursive_projective_parallel_system_with_frame(parallel, None, false, policy)', 'recursive_projective_parallel_system_with_frame(parallel, frame.as_deref(), false, policy)')
part=part.replace('parallel, &system, None, None, domain, None, false, policy,','parallel, &system, frame.as_ref(), None, domain, None, false, policy,')
s=s[:a]+part+s[b:]
# Remove representation-dependent constraint dispatch and use the same visitor.
p2=W/paths[1];f=p2.read_text()
f=f.replace('''    incident: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    family: CurveFamily2,''','''    incident: Option<&crate::bezier_offset::BezierParallelIncidentDomain2>,
    regular_domain: bool,
    family: CurveFamily2,''',1)
f=f.replace('.visit_point_incidence_evidence(point, range, incident, policy,', '.visit_point_incidence_evidence(point, range, incident, regular_domain, policy,')
f=f.replace('''                    incident.as_ref(),
                    family,
                    policy,
                )?''','''                    incident.as_ref(),
                    false,
                    family,
                    policy,
                )?''',1)
f=f.replace('''                                incident.as_ref(),
                                families[axis],''','''                                incident.as_ref(),
                                false,
                                families[axis],''',1)
a=f.index('    fn point_parameters(\n        &self,\n        component:');b=f.index('    fn select(',a);part=f[a:b]
i=part.index('        if let Some(point) = point.coordinates()');j=part.index('\n    }\n',i)
part=part[:i]+'''        fillet_point_parameters(support, point, &range, incident.as_ref(), true, family, policy)'''+part[j:]
f=f[:a]+part+f[b:]
f+=(A/'stationary-retained-point-constraint-v565.rs').read_text()
# Commit both in-memory transforms only after all source guards and anchors pass.
p.write_text(s);p2.write_text(f)
print('updated shared point query, winding callers, regular constraint dispatch and regression')
