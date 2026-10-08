from pathlib import Path
A=Path(__file__).resolve().parent;C=A/'finite-point-image-candidate-v787'
p=C/'hypercurve/src/bezier_algebraic_image.rs';s=p.read_text();assert s.count('CurveResult<RationalBezierAlgebraicPointImage2>')==4;s=s.replace('CurveResult<RationalBezierAlgebraicPointImage2>','CurveResult<Classification<RationalBezierAlgebraicPointImage2>>')
s=s.replace('resolved: OnceLock<Option<RationalBezierAlgebraicPointImage2>>,','resolved: OnceLock<RationalBezierAlgebraicPointImage2>,')
old='''        source
            .resolved
            .get_or_init(|| {
                source
                    .curve
                    .point_at_algebraic_parameter(&source.parameter, policy)
                    .ok()
            })
            .as_ref()'''
new='''        if let Some(image) = source.resolved.get() {
            return Some(image);
        }
        let Ok(Classification::Decided(image)) = source
            .curve
            .point_at_algebraic_parameter(&source.parameter, policy)
        else {
            return None;
        };
        // A failed attempt is not a point definition or a permanent cache
        // entry. Only exact successful images can satisfy later requests.
        let _ = source.resolved.set(image);
        source.resolved.get()'''
assert s.count(old)==1;s=s.replace(old,new)
old='''        let image = rational_point_image_from_power_basis(
            &expression.parameter,
            expression.x_numerator.clone(),
            expression.y_numerator.clone(),
            expression.denominator.clone(),
            policy,
        )
        .ok()?;'''
new=old.replace('let image =','let Classification::Decided(image) =').replace('.ok()?;', '.ok()? else {\n            return None;\n        };')
assert s.count(old)==1;s=s.replace(old,new)
a=s.index('impl RationalQuadraticBezier2 {');b=s.index('    /// Evaluates this rational quadratic\'s affine derivative vector',a);part=s[a:b]
part=part.replace('return Ok(image);','return Ok(Classification::Decided(image));')
part=part.replace('''        if image.status() == BezierAlgebraicImageStatus::Transformed {
            parameter.retain_rational_quadratic_point_image(self, image.clone());
        }''','''        if let Classification::Decided(image) = &image {
            parameter.retain_rational_quadratic_point_image(self, image.clone());
        }''')
part=part.replace('''    /// polynomial image construction above; see the exactness model for the exact-object''','''    /// polynomial image construction above. Only a decided finite image can
    /// become an exact curve point; a pole returns a boundary blocker and an
    /// unresolved denominator retains its predicate reason. See the exactness model for the exact-object''')
s=s[:a]+part+s[b:]
a=s.index('fn rational_point_image_with_parameter_representation(');b=s.index('pub(crate) fn rational_point_image_from_power_basis(',a)
s=s[:a]+'''fn rational_point_image_with_parameter_representation(
    parameter: &BezierAlgebraicParameter2,
    parameter_root: AlgebraicRootRepresentation,
    coefficients: RationalCoordinatePolynomials,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierAlgebraicPointImage2>> {
    match rational_coordinate_image_pair(
        parameter,
        &parameter_root,
        coefficients.x_numerator,
        coefficients.y_numerator,
        coefficients.denominator,
        policy,
    )? {
        RationalCoordinateImagePair::Transformed { first: x, second: y } => {
            Ok(Classification::Decided(RationalBezierAlgebraicPointImage2::new(
                BezierAlgebraicImageStatus::Transformed,
                parameter_root,
                Some(x), Some(y), None, None,
            )))
        }
        RationalCoordinateImagePair::Retained {
            first_numerator: x_numerator,
            second_numerator: y_numerator,
            denominator,
        } => Ok(Classification::Decided(RationalBezierAlgebraicPointImage2::new(
            BezierAlgebraicImageStatus::RetainedRationalExpression,
            parameter_root,
            None, None,
            Some(RetainedRationalPointExpression {
                parameter: parameter.clone(),
                x_numerator,
                y_numerator,
                denominator,
            }),
            Some("retained an exact non-pole Real-coefficient rational point expression".to_owned()),
        ))),
        RationalCoordinateImagePair::Failed { reason, .. } => Ok(Classification::Uncertain(reason)),
    }
}

'''+s[b:]
s=s.replace('RationalCoordinateImagePair::Failed(status) => {','RationalCoordinateImagePair::Failed { status, .. } => {')
s=s.replace('    Failed(BezierAlgebraicImageStatus),','    Failed {\n        status: BezierAlgebraicImageStatus,\n        reason: UncertaintyReason,\n    },')
old='''        Classification::Decided(RealSign::Zero) => Ok(RationalCoordinateImagePair::Failed(
            BezierAlgebraicImageStatus::XImageFailed,
        )),
        Classification::Uncertain(_) => Ok(RationalCoordinateImagePair::Failed(failed_status)),'''
new='''        Classification::Decided(RealSign::Zero) => Ok(RationalCoordinateImagePair::Failed {
            status: BezierAlgebraicImageStatus::XImageFailed,
            reason: UncertaintyReason::Boundary,
        }),
        Classification::Uncertain(reason) => Ok(RationalCoordinateImagePair::Failed {
            status: failed_status,
            reason,
        }),'''
assert s.count(old)==1;s=s.replace(old,new)
p.write_text(s)
p=C/'hypercurve/src/rational_bezier_general.rs';s=p.read_text();assert s.count('CurveResult<RationalBezierAlgebraicPointImage2>')==1;s=s.replace('CurveResult<RationalBezierAlgebraicPointImage2>','CurveResult<Classification<RationalBezierAlgebraicPointImage2>>')
a=s.index('    /// Evaluates the affine point at an isolated algebraic parameter.');b=s.index('    /// Evaluates the affine tangent at an isolated algebraic parameter.',a);part=s[a:b]
part=part.replace('return Ok(image);','return Ok(Classification::Decided(image));')
part=part.replace('''        if image.status() == crate::BezierAlgebraicImageStatus::Transformed {
            parameter.retain_rational_bezier_point_image(self, image.clone());
        }''','''        if let Classification::Decided(image) = &image {
            parameter.retain_rational_bezier_point_image(self, image.clone());
        }''')
part=part.replace('''    /// parameter interval.''','''    /// parameter interval. A proved pole returns a boundary blocker; an
    /// unresolved denominator preserves its predicate reason without creating
    /// an affine point image.''')
s=s[:a]+part+s[b:];p.write_text(s)
print('Edited isolated point factory/results and successful-only lazy cache; callers pending')
