"""Apply the sampled-profile error migration after its preserved baseline."""
from pathlib import Path
p=Path('src/curve/native.rs')
s=p.read_text()

def replace(name, following, args, doc, body):
 global s
 start=s.index('/// '+doc[0])
 end=s.index('/// '+following,start)
 names=[arg.split(':')[0] for arg in args]
 wrapper='\n'.join('    '+arg+',' for arg in args)
 call=', '.join(names)
 docs='\n'.join('/// '+line if line else '///' for line in doc)
 extra='#[allow(clippy::too_many_arguments)]\n' if len(args)>=7 else ''
 replacement=f'''{docs}
{extra}pub fn {name}(
{wrapper}
) -> Result<CurveRegion2, ValidationError> {{
    {name}_with_context({call}, &GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}}

/// Constructs the same sampled profile with explicit predicate policy and certainty.
{extra}pub fn {name}_with_context(
{wrapper}
    context: &GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {{
    let decisions = GeometryDecisions::new(context);
{body}
}}

'''
 s=s[:start]+replacement+s[end:]

replace('arrow','Filled trapezoid.',
 ['shaft_length: Real','shaft_width: Real','head_length: Real','head_width: Real'],
 ['Filled arrow.','', 'All four dimensions must be positive. Equal shaft and head widths share', 'their shoulder endpoints; only those proven duplicates are omitted.'],
 '''    for (value, name) in [
        (&shaft_length, "shaft_length"), (&shaft_width, "shaft_width"),
        (&head_length, "head_length"), (&head_width, "head_width"),
    ] {
        positive_with(value, name, &decisions)?;
    }
    let shoulders = compare_with(&shaft_width, &head_width, &decisions)? != Ordering::Equal;
    let two = Real::from(2_u8);
    let half_shaft = (shaft_width / &two)?;
    let half_head = (head_width / &two)?;
    let tip = shaft_length.clone() + head_length;
    let mut points = solid::reserved(7)?;
    points.push([Real::zero(), -half_shaft.clone()]);
    points.push([shaft_length.clone(), -half_shaft.clone()]);
    if shoulders { points.push([shaft_length.clone(), -half_head.clone()]); }
    points.push([tip, Real::zero()]);
    if shoulders { points.push([shaft_length.clone(), half_head]); }
    points.push([shaft_length, half_shaft.clone()]);
    points.push([Real::zero(), half_shaft]);
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
replace('trapezoid','Alternating-radius star polygon.',
 ['top_width: Real','bottom_width: Real','height: Real','top_offset: Real'],
 ['Filled trapezoid.','', 'Widths and height must be positive. The horizontal top offset is unrestricted.'],
 '''    positive_with(&top_width, "top_width", &decisions)?;
    positive_with(&bottom_width, "bottom_width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let region = ring_with(&[
        [Real::zero(), Real::zero()], [bottom_width, Real::zero()],
        [top_offset.clone() + top_width, height.clone()], [top_offset, height],
    ], &decisions)?;
    Ok(decisions.finish(region))''')
replace('star','Teardrop region.',
 ['num_points: usize','outer_radius: Real','inner_radius: Real'],
 ['Alternating-radius star polygon.','', 'Requires at least three points and `outer_radius > inner_radius > 0`.'],
 '''    solid::sample_minimum(num_points, 3, "num_points")?;
    let count = solid::capacity::<[Real; 2]>(num_points.checked_mul(2))?;
    positive_with(&inner_radius, "inner_radius", &decisions)?;
    if compare_with(&outer_radius, &inner_radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let denominator = Real::from(count as u64);
    let mut points = solid::reserved(count)?;
    for index in 0..count {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let radius = if index % 2 == 0 { &outer_radius } else { &inner_radius };
        points.push([radius.clone() * angle.clone().cos(), radius.clone() * angle.sin()]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
replace('teardrop','Egg region.',
 ['width: Real','length: Real','segments: usize'],
 ['Teardrop region.','', 'Requires positive dimensions, `length > width / 2`, and at least two', 'semicircle chords. The sampled boundary and its tip retain exact coordinates.'],
 '''    solid::sample_minimum(segments, 2, "segments")?;
    let capacity = solid::capacity::<[Real; 2]>(segments.checked_add(2))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&length, "length", &decisions)?;
    let radius = (width / Real::from(2_u8))?;
    if compare_with(&length, &radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let center_y = length - radius.clone();
    let mut points = solid::reserved(capacity)?;
    points.push([Real::zero(), Real::zero()]);
    for index in 0..=segments {
        let fraction = (Real::from(index as u64) / Real::from(segments as u64))?;
        let angle = Real::pi() * fraction;
        points.push([-radius.clone() * angle.clone().cos(),
                     center_y.clone() + radius.clone() * angle.sin()]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
replace('egg','Rounded rectangle represented by exact samples and straight chords.',
 ['width: Real','length: Real','segments: usize'],
 ['Egg region.','', 'Requires positive dimensions and at least three samples. Sample extrema are', 'normalized to the requested width and length; this does not certify an analytic surface.'],
 '''    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&length, "length", &decisions)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        let correction = (cosine.clone() * cosine.clone() / Real::from(5_u8))?;
        points.push([-sine, cosine + correction]);
    }
    let quarter_turns = segments / 4;
    let remainder = segments % 4;
    let min_x_index = quarter_turns + usize::from(remainder >= 2);
    let max_x_index = 3 * quarter_turns + ((3 * remainder + 2) / 4);
    let min_y_index = segments / 2;
    let min_x = points[min_x_index][0].clone();
    let max_x = points[max_x_index][0].clone();
    let min_y = points[min_y_index][1].clone();
    let max_y = points[0][1].clone();
    let x_scale = (width.clone() / (max_x.clone() - min_x.clone()))?;
    let y_scale = (length.clone() / (max_y.clone() - min_y.clone()))?;
    let x_midpoint = ((min_x + max_x) / Real::from(2_u8))?;
    let y_midpoint = ((min_y + max_y) / Real::from(2_u8))?;
    for [x, y] in &mut points {
        *x = (x.clone() - x_midpoint.clone()) * x_scale.clone();
        *y = (y.clone() - y_midpoint.clone()) * y_scale.clone();
    }
    let half_width = (width / Real::from(2_u8))?;
    let half_length = (length / Real::from(2_u8))?;
    points[min_x_index][0] = -half_width.clone();
    points[max_x_index][0] = half_width;
    if remainder == 2 {
        points[min_x_index - 1][0] = points[min_x_index][0].clone();
        points[max_x_index - 1][0] = points[max_x_index][0].clone();
    }
    points[min_y_index][1] = -half_length.clone();
    if segments % 2 == 1 { points[min_y_index + 1][1] = -half_length.clone(); }
    points[0][1] = half_length;
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
replace('squircle','Keyhole region.',
 ['width: Real','height: Real','segments: usize'],
 ['Squircle region.','', 'Requires positive dimensions and at least three samples. The signed square', 'roots of sampled sine and cosine define the represented polygon.'],
 '''    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let rx = (width / Real::from(2_u8))?;
    let ry = (height / Real::from(2_u8))?;
    let signed_root = |value: Real| -> Result<Real, ValidationError> {
        let negative = decisions.sign(&value, "squircle sample sign")? == hyperreal::RealSign::Negative;
        let root = value.abs().sqrt()?;
        Ok(if negative { -root } else { root })
    };
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        points.push([rx.clone() * signed_root(angle.clone().cos())?,
                     ry.clone() * signed_root(angle.sin())?]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
replace('pie_slice','Heart region.',
 ['radius: Real','start_angle_deg: Real','end_angle_deg: Real','segments: usize'],
 ['Circular sector.','', 'Requires positive radius, at least one chord, and a nonzero sweep of at most', '360 degrees in either direction. Degenerate sampled polygons fail explicitly.', 'A full turn preserves the existing full-circle sampling with at least three vertices.'],
 '''    solid::sample_minimum(segments, 1, "segments")?;
    let capacity = solid::capacity::<[Real; 2]>(segments.checked_add(2))?;
    positive_with(&radius, "radius", &decisions)?;
    let sweep = end_angle_deg - start_angle_deg.clone();
    let absolute_sweep = sweep.clone().abs();
    positive_with(&absolute_sweep, "sweep", &decisions)?;
    match compare_with(&absolute_sweep, &Real::from(360_u16), &decisions)? {
        Ordering::Greater => return Err(ValidationError::InvalidArguments),
        Ordering::Equal => {
            let region = ellipse_points_with(radius.clone(), radius, segments.max(3), &decisions)?;
            return Ok(decisions.finish(region));
        },
        Ordering::Less => {},
    }
    let start = (start_angle_deg * Real::pi() / Real::from(180_u16))?;
    let sweep = (sweep * Real::pi() / Real::from(180_u16))?;
    let mut points = solid::reserved(capacity)?;
    points.push([Real::zero(), Real::zero()]);
    for index in 0..=segments {
        let fraction = (Real::from(index as u64) / Real::from(segments as u64))?;
        let angle = start.clone() + sweep.clone() * fraction;
        points.push([radius.clone() * angle.clone().cos(), radius.clone() * angle.sin()]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
replace('heart','Crescent region.',
 ['width: Real','height: Real','segments: usize'],
 ['Heart region.','', 'Requires positive dimensions and at least eight samples. Sample extrema are', 'normalized to the requested dimensions; topology and analytic fidelity are distinct.'],
 '''    solid::sample_minimum(segments, 8, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    let quarter_rounding_numerator = segments.checked_add(2).ok_or(ValidationError::CapacityOverflow)?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        let cosine_squared = cosine.clone() * cosine.clone();
        points.push([
            Real::from(16_u8) * sine.clone() * sine.clone() * sine,
            Real::from(4_u8) + Real::from(19_u8) * cosine.clone()
                - Real::from(2_u8) * cosine_squared.clone()
                - Real::from(8_u8) * cosine_squared.clone() * cosine
                - Real::from(8_u8) * cosine_squared.clone() * cosine_squared,
        ]);
    }
    let max_x_index = quarter_rounding_numerator / 4;
    let min_x_index = (segments - max_x_index) % segments;
    let min_x = points[min_x_index][0].clone();
    let max_x = points[max_x_index][0].clone();
    let mut lower = 0;
    let mut upper = segments / 2;
    while lower < upper {
        let middle = lower + (upper - lower) / 2;
        match compare_with(&points[middle][1], &points[middle + 1][1], &decisions)? {
            Ordering::Less => lower = middle + 1,
            Ordering::Equal | Ordering::Greater => upper = middle,
        }
    }
    let min_y = points[segments / 2][1].clone();
    let max_y = points[lower][1].clone();
    let span_x = max_x - min_x.clone();
    let span_y = max_y - min_y.clone();
    for [x, y] in &mut points {
        *x = width.clone() * ((x.clone() - min_x.clone()) / span_x.clone())?;
        *y = height.clone() * ((y.clone() - min_y.clone()) / span_y.clone())?;
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
replace('supershape','Circular region with a rectangular keyway.',
 ['a: Real','b: Real','m: Real','n1: Real','n2: Real','n3: Real','segments: usize'],
 ['Superformula-derived filled region.','', 'Requires positive `a` and `b`, nonzero `n1`, at least three samples, and', 'defined scalar powers at every sample. Native scalar and topology failures remain typed.'],
 '''    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&a, "a", &decisions)?;
    positive_with(&b, "b", &decisions)?;
    let exponent = (Real::from(-1_i8) / n1)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let theta = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let angle = (m.clone() * theta.clone() / Real::from(4_u8))?;
        let first = (angle.clone().cos().abs() / a.clone())?.pow(n2.clone())?;
        let second = (angle.sin().abs() / b.clone())?.pow(n3.clone())?;
        let radius = (first + second).pow(exponent.clone())?;
        points.push([radius.clone() * theta.clone().cos(), radius * theta.sin()]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))''')
# The last user of the infallible ellipse sampler was the full-turn sector.
start=s.index('fn sampled_ellipse(');end=s.index('fn positive_real(',start)
assert s.count('sampled_ellipse(')==1
s=s[:start]+s[end:]
p.write_text(s)
