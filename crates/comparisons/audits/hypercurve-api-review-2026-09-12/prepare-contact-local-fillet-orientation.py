from pathlib import Path
import difflib
import subprocess

a = Path(__file__).resolve().parent
w = a.parent / 'hypercurve'
files = ['src/bezier_offset.rs', 'src/curve.rs', 'src/bezier_region.rs', 'src/curve_corner_chain.rs']

def sub(s, old, new, count=1):
    assert s.count(old) == count, (old[:120], s.count(old), count)
    return s.replace(old, new)

def change(name, s):
    # All these callers already hold a native Bezier parameter. The wider
    # predicate retains it, or the selected fiber/projective evidence, intact.
    import re
    s = re.sub(r'\.parallel_derivative_scale_sign\((\&?[\w.]+), (\&?policy)\)',
               lambda m: '.parallel_derivative_scale_sign(&'+m[1].removeprefix('&')+'.clone().into(), '+m[2]+')', s)
    if name == 'src/bezier_region.rs':
        s = sub(s, '.parallel_derivative_scale_sign(&BezierParameter2::Exact(parameter), policy)',
                '.parallel_derivative_scale_sign(&parameter.into(), policy)')
    if name == 'src/bezier_offset.rs':
        s = sub(s, 'match other.parallel_derivative_scale_sign_selected_fiber(&candidate, policy)?',
                'match other.parallel_derivative_scale_sign(&CurveParameter2::from_selected_fiber(candidate.clone()), policy)?')
        start = s.index('    pub(crate) fn parallel_derivative_scale_sign(\n')
        end = s.index('    pub(crate) fn has_exact_affine_line_parameterization', start)
        block = s[start:end]
        block = sub(block, 'parameter: &BezierParameter2,', 'parameter: &CurveParameter2,')
        block = sub(block, 'if let Some(parameter) = parameter.scalar() {',
                    'if let Some(parameter) = parameter.as_bezier_parameter().and_then(BezierParameter2::scalar) {')
        s = s[:start] + '''    /// Returns the parallel/source derivative orientation at this exact
    /// contact, reusing its scalar, selected-fiber, or projective authority.
    /// A fillet support can cross cusps between contacts on one source range.
''' + block + s[end:]
        start = s.index('    fn parallel_derivative_scale_sign_selected_fiber(\n')
        end = s.index('    fn parallel_derivative_scale_sign_from_polynomials(\n', start)
        s = s[:start] + s[end:]
        end = s.index('    #[cfg(test)]\n', start)
        block = s[start:end]
        block = sub(block, 'parameter: &BezierParameter2,', 'parameter: &CurveParameter2,')
        block = re.sub(r'signed_coefficients_at_parameter\((\&\w+), parameter, policy\)',
                       r'parameter.polynomial_sign(\1, policy)', block)
        s = s[:start] + block + s[end:]
        s = sub(s, '&retained, &policy,\n                            ),',
                '&retained.clone().into(), &policy,\n                            ),')
    if name == 'src/curve.rs':
        start = s.index('    fn support_reverses_source(\n')
        end = s.index('    fn parallel_distance(', start)
        s = s[:start] + '''    fn support_reverses_source_at(
        &self,
        support: &BezierParallel2,
        parameter: &CurveParameter2,
        family: CurveFamily2,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        let derivative_scale = |parallel: &BezierParallel2| match parallel
            .parallel_derivative_scale_sign(parameter, policy)
            .map_err(|cause| ExactCurveError::invalid(CurveOperation2::Fillet, family, cause))?
        {
            Classification::Decided(sign @ (RealSign::Positive | RealSign::Negative)) => Ok(sign),
            Classification::Decided(RealSign::Zero) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                crate::UncertaintyReason::Boundary,
            )),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Fillet,
                family,
                reason,
            )),
        };
        let (source_scale, reversed) = match self {
            Self::Direct(_) => (RealSign::Positive, false),
            Self::Retained(source) => (derivative_scale(source.parallel())?, source.is_reversed()),
            Self::Selected(source) => (derivative_scale(&source.parallel_carrier())?, source.is_reversed()),
        };
        Ok((source_scale != derivative_scale(support)?) != reversed)
    }

''' + s[end:]
        start = s.index('fn retained_fillet_parallel_support_reverses_source(\n')
        end = s.index('fn selected_fiber_parallel_derivative_scale_sign(\n', start)
        s = s[:start] + s[end:]
        # This remaining range predicate applies only to an already certified
        # regular source fragment, never to its new center support.
        s = s.replace('selected_fiber_parallel_derivative_scale_sign',
                      'selected_fiber_regular_fragment_derivative_scale_sign')
        s = sub(s, '''            let previous_support_reverses_source =
                previous_source.support_reverses_source(previous, previous_family, policy)?;
            let next_support_reverses_source =
                next_source.support_reverses_source(next, next_family, policy)?;
            let reverse_tangent_relation =
                previous_support_reverses_source != next_support_reverses_source;
''', '')
        s = sub(s, '''                let orient = |sign| {
                    if reverse_tangent_relation {''', '''                let reverse_tangent_relation = previous_source.support_reverses_source_at(
                    previous, previous_parameter, previous_family, policy,
                )? != next_source.support_reverses_source_at(
                    next, next_parameter, next_family, policy,
                )?;
                let orient = |sign| {
                    if reverse_tangent_relation {''')
        s = sub(s, 'source.support_reverses_source(support, parallel_family, policy)?',
                'source.support_reverses_source_at(support, &parameter.clone().into(), parallel_family, policy)?')
        s = sub(s, '''            let analytic_support_reverses_source = parallel_source.support_reverses_source(
                analytic_support,
                analytic_family,
                policy,
            )?;
''', '', 2)
        s = sub(s, '    analytic_support_reverses_source: bool,\n', '')
        s = sub(s, '                                analytic_support_reverses_source,\n', '', 2)
        s = sub(s, '''    if analytic_support_reverses_source {
        cross = reverse_fillet_sign(cross);''', '''    let analytic_support_reverses_source = parallel_source.support_reverses_source_at(
        analytic_support, &analytic_parameter, analytic_family, policy,
    )?;
    if analytic_support_reverses_source {
        cross = reverse_fillet_sign(cross);''')
        s = sub(s, '''                            if analytic_support_reverses_source {
                                cross = reverse_fillet_sign(cross);''', '''                            let analytic_support_reverses_source = parallel_source.support_reverses_source_at(
                                analytic_support, &contact.parallel_parameter.clone().into(), analytic_family, policy,
                            )?;
                            if analytic_support_reverses_source {
                                cross = reverse_fillet_sign(cross);''')
        s = sub(s, '''                if analytic_support_reverses_source {
                    cross = reverse_fillet_sign(cross);''', '''                let analytic_support_reverses_source = parallel_source.support_reverses_source_at(
                    analytic_support, &contact.parallel_parameter().clone().into(), analytic_family, policy,
                )?;
                if analytic_support_reverses_source {
                    cross = reverse_fillet_sign(cross);''')
    return s

for name in files:
    path = w/name
    prior = path.read_text()
    base = subprocess.check_output(['git', 'show', 'HEAD:'+name], cwd=w, text=True)
    (a/('contact-local-fillet-prior-'+path.name)).write_text(prior)
    path.write_text(change(name, prior))
    (a/('contact-local-fillet-candidate-'+path.name)).write_text(change(name, base))
    (a/('contact-local-fillet-parent-'+path.name)).write_text(base)

fmt = '/home/tim/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin/rustfmt'
subprocess.run([fmt, '--edition', '2024', '--config', 'skip_children=true',
    *[str(w/name) for name in files],
    *[str(a/('contact-local-fillet-candidate-'+Path(name).name)) for name in files]], check=True)

patch = ''.join(''.join(difflib.unified_diff(
    (a/('contact-local-fillet-parent-'+Path(name).name)).read_text().splitlines(True),
    (a/('contact-local-fillet-candidate-'+Path(name).name)).read_text().splitlines(True),
    fromfile='a/'+name, tofile='b/'+name)) for name in files)
(a/'contact-local-fillet.patch').write_text(patch)
