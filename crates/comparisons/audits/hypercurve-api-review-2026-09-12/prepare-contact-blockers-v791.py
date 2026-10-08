from pathlib import Path
import hashlib,json
A=Path(__file__).resolve().parent;W=A.parent;C=A/'contact-blockers-candidate-v791';assert not C.exists();base={}
for f in ['rational_bezier_general','bezier_offset','bezier_region','curve_region_boolean','curve','curve_fillet']:
 rel=f'hypercurve/src/{f}.rs';data=(W/rel).read_bytes();base[rel]=hashlib.sha256(data).hexdigest();p=C/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
(A/'contact-blockers-base-v791.json').write_text(json.dumps(base,indent=2)+'\n')
def sub(s,a,b):
 assert s.count(a)==1,(a[:90],s.count(a));return s.replace(a,b)
p=C/'hypercurve/src/rational_bezier_general.rs';s=p.read_text();a=s.index('pub(crate) fn exact_contact_point_evidence(');b=s.index('pub(crate) fn rational_parameter_image_matches(',a)
s=s[:a]+'''pub(crate) fn exact_contact_point_evidence(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match parameter {
        BezierParameter2::Exact(parameter) => Ok(curve
            .point_at_affine_classified(parameter, policy)
            .map(CurvePoint2::from)),
        BezierParameter2::Algebraic(parameter) => Ok(curve
            .point_at_algebraic_parameter(parameter, policy)?
            .map(CurvePoint2::from)),
    }
}

'''+s[b:]
# Optional solvers can still try a second authority, but a proved pole is not
# a coordinate projection failure. Keep their existing root/candidate replay.
old='''                            match exact_contact_point_evidence(other, parameter, policy)? {
                                Some(point) => point,
                                None => {
                                    let Some(point) = exact_contact_point_evidence(
                                        self,
                                        &conic_parameter,
                                        policy,
                                    )?
                                    else {
                                        return Ok(Some(Classification::Uncertain(
                                            UncertaintyReason::Predicate,
                                        )));
                                    };
                                    point
                                }
                            }'''
new='''                            match exact_contact_point_evidence(other, parameter, policy)? {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(UncertaintyReason::Boundary) => {
                                    return Ok(Some(Classification::Uncertain(UncertaintyReason::Boundary)));
                                }
                                Classification::Uncertain(_) => match exact_contact_point_evidence(self, &conic_parameter, policy)? {
                                    Classification::Decided(point) => point,
                                    Classification::Uncertain(reason) => return Ok(Some(Classification::Uncertain(reason))),
                                },
                            }''';s=sub(s,old,new)
old='''                let point = match exact_contact_point_evidence(other, &mapped, policy)? {
                    Some(point) => Some(point),
                    None => exact_contact_point_evidence(self, &parameter, policy)?,
                };
                let Some(point) = point else {
                    return Ok(None);
                };'''
new='''                let point = match exact_contact_point_evidence(other, &mapped, policy)? {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(UncertaintyReason::Boundary) => return Ok(None),
                    Classification::Uncertain(_) => match exact_contact_point_evidence(self, &parameter, policy)? {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(_) => return Ok(None),
                    },
                };''';s=sub(s,old,new)
old='''                        Some(point) => Some(point),
                        None => match certified_pair_point_evidence.as_deref_mut() {'''
new='''                        Classification::Decided(point) => Some(point),
                        Classification::Uncertain(UncertaintyReason::Boundary) => {
                            incomplete = true;
                            continue;
                        }
                        Classification::Uncertain(_) => match certified_pair_point_evidence.as_deref_mut() {''';s=sub(s,old,new)
s=sub(s,'let Some(point) = exact_contact_point_evidence(other, &mapped, policy)? else {','let Classification::Decided(point) = exact_contact_point_evidence(other, &mapped, policy)? else {');p.write_text(s)
p=C/'hypercurve/src/bezier_region.rs';s=p.read_text();a=s.index('fn exact_rational_endpoint_evidence(');b=s.index('fn exact_circular_algebraic_endpoint_tangent(',a);s=s[:a]+s[b:];s=s.replace('exact_rational_endpoint_evidence(', 'crate::rational_bezier_general::exact_contact_point_evidence(')
old='''            return crate::rational_bezier_general::exact_contact_point_evidence(
                &source, parameter, policy,
            )
            .map(Classification::Decided);''';new=old.replace('.map(Classification::Decided)', '.map(|point| point.map(Some))');s=sub(s,old,new);p.write_text(s)
p=C/'hypercurve/src/curve.rs';s=p.read_text();a=s.index('    if let Some(point) = crate::rational_bezier_general::exact_contact_point_evidence(',s.index('fn bezier_parallel_source_point_evidence('));b=s.index('\n    {\n        // A selected fiber',a);block=s[a:b]
block=block.replace('if let Some(point) =','match').replace('''    {
        return Ok(point);
    }''','''    {
        Classification::Decided(point) => return Ok(point),
        Classification::Uncertain(UncertaintyReason::Boundary) => return Err(ExactCurveError::blocked(operation, family, UncertaintyReason::Boundary)),
        Classification::Uncertain(_) => {}
    }''');s=s[:a]+block+s[b:];p.write_text(s)
p=C/'hypercurve/src/bezier_offset.rs';s=p.read_text()
# Four contact constructors retain geometry-owned finite-point fallbacks.
for arg in ['&candidate','&cusp_parameter','&candidate','&boundary.parameter']:
 marker=f'            let point = match exact_contact_point_evidence(other, {arg}, policy)? {{'
 # On the repeated candidate pattern, select the next unmigrated block.
 starts=[i for i in range(len(s)) if False]
 a=s.find(marker)
 if a>=0 and s[a:].startswith(marker+'\n                Classification::'):
  a=s.find(marker,a+len(marker))
 assert a>=0,marker;b=s.index('\n            };',a);part=s[a:b]
 part=part.replace('                Some(point) => point,','                Classification::Decided(point) => point,')
 part=part.replace('                None =>', '''                Classification::Uncertain(UncertaintyReason::Boundary) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                Classification::Uncertain(_) =>''',1)
 s=s[:a]+part+s[b:]
# The proved point parameter is retained; uncertainty may use the existing
# owned geometric source, but never overwrite a certified undefined value.
old='''            if let Some(point) = exact_contact_point_evidence(target, &parameter, policy)? {
                points.push(point);
                continue;
            }''';new='''            match exact_contact_point_evidence(target, &parameter, policy)? {
                Classification::Decided(point) => { points.push(point); continue; }
                Classification::Uncertain(UncertaintyReason::Boundary) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                Classification::Uncertain(_) => {}
            }''';s=sub(s,old,new)
old='''            if let Some(point) = exact_contact_point_evidence(&source, parameter, policy)? {
                return Ok(Classification::Decided(Some(point)));
            }''';new='''            match exact_contact_point_evidence(&source, parameter, policy)? {
                Classification::Decided(point) => return Ok(Classification::Decided(Some(point))),
                Classification::Uncertain(UncertaintyReason::Boundary) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                Classification::Uncertain(_) => {}
            }''';s=sub(s,old,new)
a=s.index('fn rational_point_evidence_at_parameter(');b=s.index('/// Evaluates a rational source at its native retained parameter carrier.',a);s=s[:a]+'''fn rational_point_evidence_at_parameter(
    source: &RationalBezier2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<CurvePoint2>> {
    match exact_contact_point_evidence(source, parameter, policy)? {
        Classification::Decided(point) => Ok(Classification::Decided(point)),
        Classification::Uncertain(UncertaintyReason::Boundary) => Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
        Classification::Uncertain(reason) => match parameter {
            BezierParameter2::Algebraic(parameter) => Ok(Classification::Decided(CurvePoint2::from(
                // These contact callers own a finite source-domain proof.
                // Coordinate projection may be unavailable, but a certified
                // zero denominator above must never be replaced by this source.
                RationalBezierAlgebraicPointImage2::from_parametric_source(source.clone(), parameter.clone(), policy),
            ))),
            BezierParameter2::Exact(_) => Ok(Classification::Uncertain(reason)),
        },
    }
}

'''+s[b:]
old='''            if let Some(point) = crate::rational_bezier_general::exact_contact_point_evidence(
                &source, parameter, policy,
            )? {
                return Ok(Classification::Decided(Some(point)));
            }''';new=old.replace('if let Some(point) =', 'match').replace('''                return Ok(Classification::Decided(Some(point)));''','''                Classification::Decided(point) => return Ok(Classification::Decided(Some(point))),
                Classification::Uncertain(UncertaintyReason::Boundary) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                Classification::Uncertain(_) => {}''');s=sub(s,old,new)
s=sub(s,'''            let Some(point) =
                exact_contact_point_evidence(&rational_conic, &canonical_parameter, policy)?''','''            let Classification::Decided(point) =
                exact_contact_point_evidence(&rational_conic, &canonical_parameter, policy)?''')
# The candidate system keeps all unresolved pairs for full algebraic replay.
# Its callback has a classified result as well, rather than losing the reason
# before the replay scheduler sees it.
s=sub(s,'mut point_evidence: impl FnMut(&BezierParameter2) -> CurveResult<Option<CurvePoint2>>,','mut point_evidence: impl FnMut(&BezierParameter2) -> CurveResult<Classification<CurvePoint2>>,')
s=sub(s,'let Some(point) = point_evidence(&pair.other_parameter)? else {','let Classification::Decided(point) = point_evidence(&pair.other_parameter)? else {')
a=s.index('let intersections = match self.replay_parallel_rational_candidate_system(');b=s.index('\n        )? {',a);part=s[a:b];part=sub(part,'Ok(Some(CurvePoint2::from(BezierAnalyticParallelPoint2::new(', 'Ok(Classification::Decided(CurvePoint2::from(BezierAnalyticParallelPoint2::new(');s=s[:a]+part+s[b:];p.write_text(s)
print('Prepared classified contact core and production migrations; Boolean adapters and test fixtures pending')
