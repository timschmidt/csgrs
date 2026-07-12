//! 2D Shapes as `Profile`s

use crate::csg::CSG;
use crate::hyper_math::{
    IntoReal, Real, hreal_abs, hreal_affine, hreal_div, hreal_f64s_exactly_equal,
    hreal_from_f64, hreal_mul, hreal_sub, hreal_sum, hreal_try_cmp, hxy_lerp,
};
use crate::sketch::Profile;
use hypercurve::{Contour2, CurveString2, LineSeg2, Point2, Segment2};
use std::cmp::Ordering;

fn sampled_sin_cos(
    index: usize,
    count: usize,
    start: &Real,
    sweep: &Real,
) -> Option<(Real, Real)> {
    let angle = exact_sample_angle(index, count, start, sweep)?;
    Some((angle.clone().sin(), angle.cos()))
}

fn sampled_ellipse_point(
    rx: &Real,
    ry: &Real,
    index: usize,
    count: usize,
    start: &Real,
    sweep: &Real,
) -> Option<[Real; 2]> {
    let (sin, cos) = sampled_sin_cos(index, count, start, sweep)?;
    Some([hreal_mul(rx, cos)?, hreal_mul(ry, sin)?])
}

fn exact_sample_angle(index: usize, count: usize, start: &Real, sweep: &Real) -> Option<Real> {
    let fraction = (Real::from(index as u64) / Real::from(count as u64)).ok()?;
    Some(start.clone() + sweep.clone() * fraction)
}

fn exact_polar(radius: &Real, angle: Real) -> [Real; 2] {
    [
        radius.clone() * angle.clone().cos(),
        radius.clone() * angle.sin(),
    ]
}

fn exact_ratio(numerator: usize, denominator: usize) -> Option<Real> {
    (Real::from(numerator as u64) / Real::from(denominator as u64)).ok()
}

fn signed_sqrt(value: Real) -> Option<Real> {
    let sign = hreal_try_cmp(&value, 0.0)?;
    let root = hreal_abs(value)?.sqrt().ok()?;
    Some(if sign == Ordering::Less { -root } else { root })
}

/// Materialize an explicitly polygonal approximation without demoting its
/// exact-real analytic samples.
fn tessellation_profile(points: &[[Real; 2]]) -> Profile {
    Contour2::from_real_ring(points)
        .ok()
        .map(Profile::from_contour)
        .unwrap_or_else(Profile::empty)
}

fn hcircle_samples(samples: usize, radius: Real) -> Option<Vec<[Real; 2]>> {
    let mut points = Vec::with_capacity(samples);
    for index in 0..samples {
        points.push(exact_polar(
            &radius,
            exact_sample_angle(index, samples, &Real::zero(), &Real::tau())?,
        ));
    }
    Some(points)
}

fn translated_circle_profile(
    radius: Real,
    segments: usize,
    center_x: &Real,
    center_y: &Real,
) -> Option<Profile> {
    let mut points = hcircle_samples(segments, radius)?;
    for [x, y] in &mut points {
        *x = center_x + &*x;
        *y = center_y + &*y;
    }
    Some(tessellation_profile(&points))
}

fn hfinite_nonzero(value: Real) -> bool {
    matches!(
        hreal_try_cmp(&value, 0.0),
        Some(Ordering::Less | Ordering::Greater)
    )
}

/// Compare a public profile scalar through hyperreal ordering.
///
/// Shape constructors still accept primitive `Real` parameters at the API
/// boundary, but admission decisions should not use local float ordering. This
/// helper promotes both operands and compares through Hyper's refinement path,
/// following Yap's exact geometric computation boundary model
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn hprofile_scalar_gt<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    matches!(hreal_try_cmp(lhs, rhs), Some(Ordering::Greater))
}

/// Accept any finite, strictly positive profile scalar exactly.
///
/// Hyperreal-backed geometry no longer treats small nonzero dimensions as
/// degenerate. Keeping admission predicates exact follows Yap's exact
/// geometric computation model (<https://doi.org/10.1016/0925-7721(95)00040-2>)
/// and avoids reintroducing a floating tolerance into `Profile` construction.
fn hprofile_scalar_positive<T: IntoReal>(value: T) -> bool {
    hprofile_scalar_gt(value, 0.0)
}

fn hprofile_scalar_nonnegative<T: IntoReal>(value: T) -> bool {
    matches!(
        hreal_try_cmp(value, 0.0),
        Some(Ordering::Equal | Ordering::Greater)
    )
}

#[allow(clippy::too_many_arguments)]
fn sampled_involute_gear(
    module: &Real,
    teeth: usize,
    pressure_angle_deg: &Real,
    clearance: &Real,
    backlash: &Real,
    segments_per_flank: usize,
) -> Profile {
    if teeth < 4
        || segments_per_flank < 2
        || !hprofile_scalar_positive(module)
        || !hprofile_scalar_nonnegative(clearance)
        || !hprofile_scalar_nonnegative(backlash)
        || !matches!(
            hreal_try_cmp(pressure_angle_deg, 0.0),
            Some(Ordering::Greater)
        )
        || !matches!(hreal_try_cmp(pressure_angle_deg, 90.0), Some(Ordering::Less))
    {
        return Profile::empty();
    }
    let Some((module, pressure_angle, clearance, backlash)) = module
        .to_f64_lossy()
        .zip(pressure_angle_deg.to_f64_lossy())
        .zip(clearance.to_f64_lossy().zip(backlash.to_f64_lossy()))
        .map(|((module, pressure_angle), (clearance, backlash))| {
            (module, pressure_angle.to_radians(), clearance, backlash)
        })
    else {
        return Profile::empty();
    };

    let pitch_radius = 0.5 * module * teeth as f64;
    let base_radius = pitch_radius * pressure_angle.cos();
    let outer_radius = pitch_radius + module;
    let root_radius = pitch_radius - 1.25 * module - clearance;
    let Some(tau) = Real::tau().to_f64_lossy() else {
        return Profile::empty();
    };
    let angular_pitch = tau / teeth as f64;
    let half_tooth_angle = 0.5 * (0.5 * angular_pitch - backlash / pitch_radius);
    if !(base_radius > 0.0 && root_radius > 0.0 && half_tooth_angle > 0.0) {
        return Profile::empty();
    }

    let involute_angle = |radius: f64| {
        let parameter = ((radius / base_radius).powi(2) - 1.0).max(0.0).sqrt();
        parameter - parameter.atan()
    };
    let flank_start_radius = root_radius.max(base_radius);
    let pitch_involute = involute_angle(pitch_radius);
    let start_involute = involute_angle(flank_start_radius);
    let outer_involute = involute_angle(outer_radius);
    let offset = half_tooth_angle + pitch_involute;
    let right_start = start_involute - offset;
    let left_start = offset - start_involute;
    let right_tip = outer_involute - offset;
    let left_tip = offset - outer_involute;
    if !(right_start < left_start && right_tip < left_tip) {
        return Profile::empty();
    }

    let polar = |radius: f64, angle: f64| [radius * angle.cos(), radius * angle.sin()];
    let mut points = Vec::with_capacity(teeth * (4 * segments_per_flank + 3));
    for tooth in 0..teeth {
        let center = tooth as f64 * angular_pitch;
        points.push(polar(root_radius, center + right_start));
        if flank_start_radius > root_radius {
            points.push(polar(flank_start_radius, center + right_start));
        }
        for sample in 1..=segments_per_flank {
            let t = sample as f64 / segments_per_flank as f64;
            let radius = flank_start_radius + t * (outer_radius - flank_start_radius);
            points.push(polar(radius, center + involute_angle(radius) - offset));
        }
        for sample in 1..=segments_per_flank {
            let t = sample as f64 / segments_per_flank as f64;
            points.push(polar(
                outer_radius,
                center + right_tip + t * (left_tip - right_tip),
            ));
        }
        for sample in 1..=segments_per_flank {
            let t = sample as f64 / segments_per_flank as f64;
            let radius = outer_radius - t * (outer_radius - flank_start_radius);
            points.push(polar(radius, center + offset - involute_angle(radius)));
        }
        if flank_start_radius > root_radius {
            points.push(polar(root_radius, center + left_start));
        }
        let next_right = angular_pitch + right_start;
        for sample in 1..segments_per_flank {
            let t = sample as f64 / segments_per_flank as f64;
            points.push(polar(
                root_radius,
                center + left_start + t * (next_right - left_start),
            ));
        }
    }

    Contour2::from_finite_ring(&points)
        .map(Profile::from_contour)
        .unwrap_or_else(|_| Profile::empty())
}

fn sampled_cycloidal_gear(
    module: &Real,
    teeth: usize,
    generating_radius: &Real,
    clearance: &Real,
    segments_per_flank: usize,
) -> Profile {
    if teeth < 3
        || segments_per_flank < 2
        || !hprofile_scalar_positive(module)
        || !hprofile_scalar_positive(generating_radius)
        || !hprofile_scalar_nonnegative(clearance)
    {
        return Profile::empty();
    }
    let (Some(module), Some(generator), Some(clearance)) = (
        module.to_f64_lossy(),
        generating_radius.to_f64_lossy(),
        clearance.to_f64_lossy(),
    ) else {
        return Profile::empty();
    };
    let pitch_radius = 0.5 * module * teeth as f64;
    let outer_radius = pitch_radius + module;
    let root_radius = pitch_radius - 1.25 * module - clearance;
    if !(root_radius > 0.0 && 2.0 * generator < pitch_radius) {
        return Profile::empty();
    }
    let epicycloid = |parameter: f64| {
        let ratio = (pitch_radius + generator) / generator;
        [
            (pitch_radius + generator) * parameter.cos()
                - generator * (ratio * parameter).cos(),
            (pitch_radius + generator) * parameter.sin()
                - generator * (ratio * parameter).sin(),
        ]
    };
    let hypocycloid = |parameter: f64| {
        let ratio = (pitch_radius - generator) / generator;
        [
            (pitch_radius - generator) * parameter.cos()
                + generator * (ratio * parameter).cos(),
            (pitch_radius - generator) * parameter.sin()
                - generator * (ratio * parameter).sin(),
        ]
    };
    let radius = |point: [f64; 2]| point[0].hypot(point[1]);
    let Some((pi, tau)) = Real::pi().to_f64_lossy().zip(Real::tau().to_f64_lossy()) else {
        return Profile::empty();
    };
    let lobe_extremum = pi * generator / pitch_radius;
    let solve_first_lobe = |target: f64, increasing: bool, curve: &dyn Fn(f64) -> [f64; 2]| {
        let end_radius = radius(curve(lobe_extremum));
        let scale = target.abs().max(end_radius.abs()).max(1.0);
        if (target - end_radius).abs() <= 32.0 * f64::EPSILON * scale {
            return Some(lobe_extremum);
        }
        if (increasing && target > end_radius) || (!increasing && target < end_radius) {
            return None;
        }
        let mut low = 0.0;
        let mut high = lobe_extremum;
        for _ in 0..64 {
            let mid = 0.5 * (low + high);
            let mid_radius = radius(curve(mid));
            if (increasing && mid_radius < target) || (!increasing && mid_radius > target) {
                low = mid;
            } else {
                high = mid;
            }
        }
        Some(0.5 * (low + high))
    };
    let Some(tip_parameter) = solve_first_lobe(outer_radius, true, &epicycloid) else {
        return Profile::empty();
    };
    let generated_root_radius = pitch_radius - 2.0 * generator;
    let flank_root_radius = root_radius.max(generated_root_radius);
    let Some(root_parameter) = solve_first_lobe(flank_root_radius, false, &hypocycloid) else {
        return Profile::empty();
    };
    let angular_pitch = tau / teeth as f64;
    let pitch_half_thickness = 0.25 * angular_pitch;
    let rotate = |point: [f64; 2], angle: f64| {
        let (sin, cos) = angle.sin_cos();
        [
            point[0] * cos - point[1] * sin,
            point[0] * sin + point[1] * cos,
        ]
    };
    let angle = |point: [f64; 2]| point[1].atan2(point[0]);
    let tip_point = epicycloid(tip_parameter);
    let root_point = hypocycloid(root_parameter);
    let right_tip_angle = angle(rotate([tip_point[0], -tip_point[1]], -pitch_half_thickness));
    let right_root_angle =
        angle(rotate([root_point[0], -root_point[1]], -pitch_half_thickness));
    let left_tip_angle = -right_tip_angle;
    let left_root_angle = -right_root_angle;
    if !(right_tip_angle < left_tip_angle && right_root_angle < left_root_angle) {
        return Profile::empty();
    }

    let polar = |radius: f64, angle: f64| [radius * angle.cos(), radius * angle.sin()];
    let mut points = Vec::with_capacity(teeth * (6 * segments_per_flank + 2));
    for tooth in 0..teeth {
        let center = tooth as f64 * angular_pitch;
        for sample in 0..=segments_per_flank {
            let u = sample as f64 / segments_per_flank as f64;
            let point = hypocycloid(root_parameter * (1.0 - u));
            points.push(rotate([point[0], -point[1]], center - pitch_half_thickness));
        }
        for sample in 1..=segments_per_flank {
            let u = sample as f64 / segments_per_flank as f64;
            let point = epicycloid(tip_parameter * u);
            points.push(rotate([point[0], -point[1]], center - pitch_half_thickness));
        }
        for sample in 1..=segments_per_flank {
            let u = sample as f64 / segments_per_flank as f64;
            points.push(polar(
                outer_radius,
                center + right_tip_angle + u * (left_tip_angle - right_tip_angle),
            ));
        }
        for sample in 1..=segments_per_flank {
            let u = sample as f64 / segments_per_flank as f64;
            let point = epicycloid(tip_parameter * (1.0 - u));
            points.push(rotate(point, center + pitch_half_thickness));
        }
        for sample in 1..=segments_per_flank {
            let u = sample as f64 / segments_per_flank as f64;
            let point = hypocycloid(root_parameter * u);
            points.push(rotate(point, center + pitch_half_thickness));
        }
        let next_right_root = angular_pitch + right_root_angle;
        for sample in 1..segments_per_flank {
            let u = sample as f64 / segments_per_flank as f64;
            points.push(polar(
                root_radius,
                center + left_root_angle + u * (next_right_root - left_root_angle),
            ));
        }
    }
    Contour2::from_finite_ring(&points)
        .map(Profile::from_contour)
        .unwrap_or_else(|_| Profile::empty())
}

impl Profile {
    /// Build a finite boundary sketch as a native hypercurve region.
    ///
    /// API callers still pass ordinary `Real` coordinates, but the boundary is
    /// immediately promoted into `hyperreal` control points. Keeping this
    /// conversion at the API edge follows the exact-geometric-computation model
    /// described by Yap, "Towards exact geometric computation",
    /// Computational Geometry 7(1-2), 1997, DOI: 10.1016/0925-7721(95)00040-2.
    fn polygonal_region(points: Vec<[Real; 2]>) -> Self {
        let Ok(contour) = Contour2::from_real_ring(&points) else {
            return Profile::empty();
        };
        Profile::from_contour(contour)
    }

    /// Creates a 2D rectangle in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width of the rectangle
    /// - `length`: the height of the rectangle
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::profile::Profile;
    /// let sq2 = Profile::rectangle(2.0, 3.0 );
    /// ```
    pub fn rectangle(width: Real, length: Real) -> Self {
        if !hprofile_scalar_positive(&width) || !hprofile_scalar_positive(&length) {
            return Profile::empty();
        }
        let points = [
            [Real::zero(), Real::zero()],
            [width.clone(), Real::zero()],
            [width, length.clone()],
            [Real::zero(), length],
        ];
        Self::polygon(&points)
    }

    /// Creates a 2D square in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width=length of the square
    ///
    /// # Example
    /// let sq2 = Profile::square(2.0, None);
    pub fn square(width: Real) -> Self {
        Self::rectangle(width.clone(), width)
    }

    /// **Mathematical Foundation: Parametric Circle Discretization**
    ///
    /// Creates a 2D circle in the XY plane using parametric equations.
    /// This implements the standard circle parameterization with uniform angular sampling.
    ///
    /// ## **Circle Mathematics**
    ///
    /// ### **Parametric Representation**
    /// For a circle of radius r centered at origin:
    /// ```text
    /// x(θ) = r·cos(θ)
    /// y(θ) = r·sin(θ)
    /// where θ ∈ [0, 2π]
    /// ```text
    ///
    /// ### **Discretization Algorithm**
    /// For n segments, sample at angles:
    /// ```text
    /// θᵢ = 2πi/n, i ∈ {0, 1, ..., n-1}
    /// ```
    /// This produces n vertices uniformly distributed around the circle.
    ///
    /// ### **Approximation Error**
    /// The polygonal approximation has:
    /// - **Maximum radial error**: r(1 - cos(π/n)) ≈ r(π/n)²/8 for large n
    /// - **Perimeter error**: 2πr - n·r·sin(π/n) ≈ πr/3n² for large n
    /// - **Area error**: πr² - (nr²sin(2π/n))/2 ≈ πr³/6n² for large n
    ///
    /// ### **Numerical Stability**
    /// - Uses tau() (2π) constant for better floating-point precision
    /// - Explicit closure ensures geometric validity
    /// - Minimum 3 segments to avoid degenerate polygons
    ///
    /// ## **Applications**
    /// - **Geometric modeling**: Base shape for 3D extrusion
    /// - **Collision detection**: Circular boundaries
    /// - **Numerical integration**: Circular domains
    ///
    /// # Parameters
    /// - `radius`: Circle radius (must be > 0)
    /// - `segments`: Number of polygon edges (minimum 3 for valid geometry)
    pub fn circle(radius: Real, segments: usize) -> Self {
        if segments < 3 || !hprofile_scalar_positive(&radius) {
            return Profile::empty();
        }
        let Some(points) = hcircle_samples(segments, radius) else {
            return Profile::empty();
        };
        tessellation_profile(&points)
    }

    /// Right triangle from (0,0) to (width,0) to (0,height).
    pub fn right_triangle(width: Real, height: Real) -> Self {
        if !hprofile_scalar_positive(&width) || !hprofile_scalar_positive(&height) {
            return Profile::empty();
        }
        let points = [
            [Real::zero(), Real::zero()],
            [width, Real::zero()],
            [Real::zero(), height],
        ];
        Self::polygon(&points)
    }

    /// Creates a 2D polygon in the XY plane from a list of `[x, y]` points.
    ///
    /// # Parameters
    ///
    /// - `points`: a sequence of 2D points (e.g. `[[0.0,0.0], [1.0,0.0], [0.5,1.0]]`)
    ///   describing the polygon boundary in order.
    ///
    /// # Example
    /// let pts = vec![[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]];
    /// let poly2d = Profile::polygon_points(&pts);
    pub fn polygon(points: &[[Real; 2]]) -> Self {
        let Ok(contour) = Contour2::from_real_ring(points) else {
            return Profile::empty();
        };
        Profile::from_contour(contour)
    }

    /// Creates a 2D polygon from native hypercurve points.
    ///
    /// Unlike [`Profile::polygon`], this constructor does not first pass through
    /// finite `[f64; 2]` samples. The ring is converted directly to
    /// `hypercurve::LineSeg2` segments and then to a `Contour2`, so exact
    /// point topology stays with hypercurve from construction onward. This
    /// follows Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn polygon_points(points: &[Point2]) -> Self {
        if points.len() < 3 {
            return Profile::empty();
        }

        let mut segments = Vec::with_capacity(points.len());
        for adjacent in points.windows(2) {
            let Ok(segment) = LineSeg2::try_new(adjacent[0].clone(), adjacent[1].clone())
            else {
                return Profile::empty();
            };
            segments.push(Segment2::Line(segment));
        }
        let Ok(segment) =
            LineSeg2::try_new(points[points.len() - 1].clone(), points[0].clone())
        else {
            return Profile::empty();
        };
        segments.push(Segment2::Line(segment));

        let Ok(contour) = Contour2::try_new(segments) else {
            return Profile::empty();
        };
        Profile::from_contour(contour)
    }

    /// **Mathematical Foundation: Parametric Ellipse Generation**
    ///
    /// Creates an ellipse in XY plane, centered at (0,0), using parametric equations.
    /// This implements the standard ellipse parameterization with uniform parameter sampling.
    ///
    /// ## **Ellipse Mathematics**
    ///
    /// ### **Parametric Representation**
    /// For an ellipse with semi-major axis a and semi-minor axis b:
    /// ```text
    /// x(θ) = a·cos(θ)
    /// y(θ) = b·sin(θ)
    /// where θ ∈ [0, 2π]
    /// ```
    /// In our implementation: a = width/2, b = height/2
    ///
    /// ### **Geometric Properties**
    /// - **Area**: A = πab = π(width·height)/4
    /// - **Circumference** (Ramanujan's approximation):
    ///   ```text
    ///   C ≈ π[3(a+b) - √((3a+b)(a+3b))]
    ///   ```
    /// - **Eccentricity**: e = √(1 - b²/a²) for a ≥ b
    /// - **Focal distance**: c = a·e where foci are at (±c, 0)
    ///
    /// ### **Parametric vs Arc-Length Parameterization**
    /// **Note**: This uses parameter-uniform sampling (constant Δθ), not
    /// arc-length uniform sampling. For arc-length uniformity, use:
    /// ```text
    /// ds/dθ = √(a²sin²θ + b²cos²θ)
    /// ```
    /// Parameter-uniform is computationally simpler and sufficient for most applications.
    ///
    /// ### **Approximation Quality**
    /// For n segments, the polygonal approximation error behaves as O(1/n²):
    /// - **Maximum radial error**: Approximately (a-b)π²/(8n²) for a ≈ b
    /// - **Area convergence**: Exact area approached as n → ∞
    ///
    /// ## **Special Cases**
    /// - **Circle**: When width = height, reduces to parametric circle
    /// - **Degenerate**: When width = 0 or height = 0, becomes a line segment
    ///
    /// # Parameters
    /// - `width`: Full width (diameter) along x-axis
    /// - `height`: Full height (diameter) along y-axis  
    /// - `segments`: Number of polygon edges (minimum 3)
    pub fn ellipse(width: Real, height: Real, segments: usize) -> Self {
        if segments < 3
            || !hprofile_scalar_positive(&width)
            || !hprofile_scalar_positive(&height)
        {
            return Profile::empty();
        }
        let (Some(rx), Some(ry)) = (hreal_mul(0.5, &width), hreal_mul(0.5, &height)) else {
            return Profile::empty();
        };
        let Some(points) = (0..segments)
            .map(|index| {
                sampled_ellipse_point(&rx, &ry, index, segments, &Real::zero(), &Real::tau())
            })
            .collect::<Option<Vec<_>>>()
        else {
            return Profile::empty();
        };
        tessellation_profile(&points)
    }

    /// **Mathematical Foundation: Regular Polygon Construction**
    ///
    /// Creates a regular n-gon inscribed in a circle of given radius.
    /// This implements the classical construction of regular polygons using
    /// uniform angular division of the circumscribed circle.
    ///
    /// ## **Regular Polygon Mathematics**
    ///
    /// ### **Vertex Construction**
    /// For a regular n-gon inscribed in a circle of radius r:
    /// ```text
    /// Vertex_i = (r·cos(2πi/n), r·sin(2πi/n))
    /// where i ∈ {0, 1, ..., n-1}
    /// ```
    ///
    /// ### **Geometric Properties**
    /// - **Interior angle**: α = (n-2)π/n = π - 2π/n
    /// - **Central angle**: β = 2π/n
    /// - **Exterior angle**: γ = 2π/n
    /// - **Side length**: s = 2r·sin(π/n)
    /// - **Apothem** (distance from center to side): a = r·cos(π/n)
    /// - **Area**: A = (n·s·a)/2 = (n·r²·sin(2π/n))/2
    ///
    /// ### **Special Cases**
    /// - **n = 3**: Equilateral triangle (α = 60°)
    /// - **n = 4**: Square (α = 90°)
    /// - **n = 5**: Regular pentagon (α = 108°)
    /// - **n = 6**: Regular hexagon (α = 120°)
    /// - **n → ∞**: Approaches circle (lim α = 180°)
    ///
    /// ### **Constructibility Theorem**
    /// A regular n-gon is constructible with compass and straightedge if and only if:
    /// ```text
    /// n = 2^k · p₁ · p₂ · ... · pₘ
    /// ```
    /// where k ≥ 0 and pᵢ are distinct Fermat primes (3, 5, 17, 257, 65537).
    ///
    /// ### **Approximation to Circle**
    /// As n increases, the regular n-gon converges to a circle:
    /// - **Perimeter convergence**: P_n = n·s → 2πr as n → ∞
    /// - **Area convergence**: A_n → πr² as n → ∞
    /// - **Error bound**: |A_circle - A_n| ≤ πr³/(3n²) for large n
    ///
    /// ## **Numerical Considerations**
    /// - Uses tau() for precise angular calculations
    /// - Explicit closure for geometric validity
    /// - Minimum n = 3 to avoid degenerate cases
    ///
    /// # Parameters
    /// - `sides`: Number of polygon edges (≥ 3)
    /// - `radius`: Circumscribed circle radius
    pub fn regular_ngon(sides: usize, radius: Real) -> Self {
        if sides < 3 || !hprofile_scalar_positive(&radius) {
            return Profile::empty();
        }
        let Some(points) = hcircle_samples(sides, radius) else {
            return Profile::empty();
        };
        tessellation_profile(&points)
    }

    /// Creates a 2D arrow in the XY plane.
    ///
    /// The arrow points along the positive X-axis, starting at (0,0).
    /// It consists of a shaft and a triangular head.
    ///
    /// # Parameters
    ///
    /// - `shaft_length`: length of the arrow shaft
    /// - `shaft_width`: width of the arrow shaft
    /// - `head_length`: length of the arrow head (from tip to base)
    /// - `head_width`: width of the arrow head at its base
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::profile::Profile;
    /// let arrow = Profile::arrow(5.0, 0.5, 2.0, 1.5 );
    /// ```
    pub fn arrow(
        shaft_length: Real,
        shaft_width: Real,
        head_length: Real,
        head_width: Real,
    ) -> Self {
        if !hprofile_scalar_positive(&shaft_length)
            || !hprofile_scalar_positive(&shaft_width)
            || !hprofile_scalar_positive(&head_length)
            || !hprofile_scalar_positive(&head_width)
        {
            return Profile::empty();
        }

        // Define the points for the arrow polygon
        // The arrow points along the positive X-axis
        let half_shaft_width = shaft_width * 0.5;
        let half_head_width = head_width * 0.5;
        let tip_x = shaft_length.clone() + head_length;

        let points = vec![
            [Real::zero(), half_shaft_width.clone()], // Top-left of shaft
            [shaft_length.clone(), half_shaft_width.clone()], // Top-right of shaft
            [shaft_length.clone(), half_head_width.clone()], // Top-right of head base
            [tip_x, Real::zero()],                    // Tip of arrow
            [shaft_length.clone(), -half_head_width], // Bottom-right of head base
            [shaft_length, -half_shaft_width.clone()], // Bottom-right of shaft
            [Real::zero(), -half_shaft_width.clone()], // Bottom-left of shaft
            [Real::zero(), half_shaft_width],         // Back to top-left to close
        ];

        tessellation_profile(&points)
    }

    /// Trapezoid from (0,0) -> (bottom_width,0) -> (top_width+top_offset,height) -> (top_offset,height)
    /// Note: this is a simple shape that can represent many trapezoids or parallelograms.
    pub fn trapezoid(
        top_width: Real,
        bottom_width: Real,
        height: Real,
        top_offset: Real,
    ) -> Self {
        if !hprofile_scalar_positive(&top_width)
            || !hprofile_scalar_positive(&bottom_width)
            || !hprofile_scalar_positive(&height)
        {
            return Profile::empty();
        }
        let points = vec![
            [Real::zero(), Real::zero()],
            [bottom_width, Real::zero()],
            [top_width + top_offset.clone(), height.clone()],
            [top_offset, height],
        ];
        Self::polygonal_region(points)
    }

    /// Star shape (typical "spiky star") with `num_points`, outer_radius, inner_radius.
    /// The star is centered at (0,0).
    pub fn star(num_points: usize, outer_radius: Real, inner_radius: Real) -> Self {
        if num_points < 3
            || !hprofile_scalar_positive(&inner_radius)
            || !hprofile_scalar_gt(&outer_radius, &inner_radius)
        {
            return Profile::empty();
        }
        let mut points = Vec::with_capacity(num_points * 2);
        for i in 0..num_points {
            let Some(outer_point) = sampled_ellipse_point(
                &outer_radius,
                &outer_radius,
                i,
                num_points,
                &Real::zero(),
                &Real::tau(),
            ) else {
                return Profile::empty();
            };
            let Some(inner_point) = sampled_ellipse_point(
                &inner_radius,
                &inner_radius,
                (i * 2) + 1,
                num_points * 2,
                &Real::zero(),
                &Real::tau(),
            ) else {
                return Profile::empty();
            };
            points.push(outer_point);
            points.push(inner_point);
        }
        tessellation_profile(&points)
    }

    /// Teardrop shape.  A simple approach:
    /// - a circle arc for the "round" top
    /// - it tapers down to a cusp at bottom.
    ///
    /// This is just one of many possible "teardrop" definitions.
    ///
    /// The semicircular cap retains exact rational multiples of symbolic pi.
    pub fn teardrop(width: Real, length: Real, segments: usize) -> Profile {
        if segments < 2
            || !hprofile_scalar_positive(&width)
            || !hprofile_scalar_positive(&length)
        {
            return Profile::empty();
        }
        let Some(r) = hreal_mul(0.5, &width) else {
            return Profile::empty();
        };
        if !matches!(hreal_try_cmp(&length, &r), Some(Ordering::Greater)) {
            return Profile::empty();
        }
        let Some(center_y) = hreal_sub(&length, &r) else {
            return Profile::empty();
        };
        let mut points = vec![[Real::zero(), Real::zero()]]; // Start at the tip
        for i in 0..=segments {
            let Some([dx, dy]) =
                sampled_ellipse_point(&r, &r, i, segments, &Real::zero(), &Real::pi())
            else {
                return Profile::empty();
            };
            let (Some(x), Some(y)) = (hreal_sub(0.0, dx), hreal_affine(&center_y, 1.0, dy))
            else {
                return Profile::empty();
            };
            points.push([x, y]);
        }

        tessellation_profile(&points)
    }

    /// Egg outline.  Approximate an egg shape using a parametric approach.
    /// This is only a toy approximation.  It creates a closed "egg-ish" outline around the origin.
    ///
    /// Angles are constructed symbolically before the explicitly polygonal
    /// outline is projected to finite samples for normalization.
    pub fn egg(width: Real, length: Real, segments: usize) -> Profile {
        if segments < 3
            || !hprofile_scalar_positive(&width)
            || !hprofile_scalar_positive(&length)
        {
            return Profile::empty();
        }
        let mut raw = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(theta) = exact_sample_angle(i, segments, &Real::zero(), &Real::tau())
            else {
                return Profile::empty();
            };
            let Some((sin, cos)) = theta
                .clone()
                .sin()
                .to_f64_lossy()
                .zip(theta.cos().to_f64_lossy())
            else {
                return Profile::empty();
            };
            raw.push((-sin, cos * (1.0 + 0.2 * cos)));
        }
        let min_x = raw.iter().map(|point| point.0).fold(f64::INFINITY, f64::min);
        let max_x = raw
            .iter()
            .map(|point| point.0)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_y = raw.iter().map(|point| point.1).fold(f64::INFINITY, f64::min);
        let max_y = raw
            .iter()
            .map(|point| point.1)
            .fold(f64::NEG_INFINITY, f64::max);
        let points = raw
            .into_iter()
            .map(|(x, y)| -> Option<[Real; 2]> {
                Some([
                    width.clone()
                        * hreal_from_f64((x - min_x) / (max_x - min_x) - 0.5).ok()?,
                    length.clone()
                        * hreal_from_f64((y - min_y) / (max_y - min_y) - 0.5).ok()?,
                ])
            })
            .collect::<Option<Vec<_>>>();
        points.map_or_else(Profile::empty, Self::polygonal_region)
    }

    /// Rounded rectangle in XY plane, from (0,0) to (width,height) with radius for corners.
    /// `corner_segments` controls the smoothness of each rounded corner.
    ///
    /// Corner-radius admission and clamping use exact hyperreal comparisons.
    /// Arc samples retain exact rational multiples of symbolic pi.
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
    ) -> Self {
        if !hprofile_scalar_positive(&width)
            || !hprofile_scalar_positive(&height)
            || !hprofile_scalar_nonnegative(&corner_radius)
        {
            return Profile::empty();
        }
        let (Some(half_width), Some(half_height)) =
            (hreal_mul(&width, 0.5), hreal_mul(&height, 0.5))
        else {
            return Profile::empty();
        };
        let Some(radius_order) = hreal_try_cmp(&corner_radius, &half_width) else {
            return Profile::empty();
        };
        let radius = match radius_order {
            Ordering::Greater => half_width,
            Ordering::Less | Ordering::Equal => corner_radius.clone(),
        };
        let Some(radius_order) = hreal_try_cmp(&radius, &half_height) else {
            return Profile::empty();
        };
        let r = match radius_order {
            Ordering::Greater => half_height,
            Ordering::Less | Ordering::Equal => radius,
        };
        if corner_segments == 0 || !hprofile_scalar_positive(&r) {
            return Profile::rectangle(width, height);
        }
        // We'll approximate each 90° corner with `corner_segments` arcs
        let mut points = Vec::with_capacity((corner_segments + 1) * 4);
        let Some(right) = hreal_sub(&width, &r) else {
            return Profile::empty();
        };
        let Some(top) = hreal_sub(&height, &r) else {
            return Profile::empty();
        };
        let Some(half_pi) = (Real::pi() / Real::from(2_u8)).ok() else {
            return Profile::empty();
        };
        for (cx, cy, start_angle) in [
            (r.clone(), r.clone(), Real::pi()),
            (right.clone(), r.clone(), Real::pi() + half_pi.clone()),
            (right, top.clone(), Real::zero()),
            (r.clone(), top, half_pi.clone()),
        ] {
            for i in 0..=corner_segments {
                let Some([dx, dy]) =
                    sampled_ellipse_point(&r, &r, i, corner_segments, &start_angle, &half_pi)
                else {
                    return Profile::empty();
                };
                let (Some(x), Some(y)) =
                    (hreal_affine(&cx, 1.0, dx), hreal_affine(&cy, 1.0, dy))
                else {
                    return Profile::empty();
                };
                points.push([x, y]);
            }
        }

        tessellation_profile(&points)
    }

    /// Squircle (superellipse) centered at (0,0) with bounding box width×height.
    /// We use an exponent = 4.0 for "classic" squircle shape. `segments` controls the resolution.
    ///
    /// This is Lamé's superellipse specialized to exponent 4. The signed
    /// square-root form is evaluated in exact-real arithmetic. See also Lamé,
    /// *Examen des différentes méthodes employées pour résoudre les problèmes
    /// de géométrie*, 1818.
    pub fn squircle(width: Real, height: Real, segments: usize) -> Profile {
        if segments < 3
            || !hprofile_scalar_positive(&width)
            || !hprofile_scalar_positive(&height)
        {
            return Profile::empty();
        }
        let (Some(rx), Some(ry)) = (hreal_mul(0.5, &width), hreal_mul(0.5, &height)) else {
            return Profile::empty();
        };
        let mut points = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(angle) = exact_sample_angle(i, segments, &Real::zero(), &Real::tau())
            else {
                return Profile::empty();
            };
            let (Some(ct), Some(st)) =
                (signed_sqrt(angle.clone().cos()), signed_sqrt(angle.sin()))
            else {
                return Profile::empty();
            };
            let (Some(x), Some(y)) = (hreal_mul(&rx, ct), hreal_mul(&ry, st)) else {
                return Profile::empty();
            };
            points.push([x, y]);
        }

        tessellation_profile(&points)
    }

    /// Keyhole shape (simple version): a large circle + a rectangle "handle".
    /// This does *not* have a hole.  If you want a literal hole, you'd do difference ops.
    /// Here we do union of a circle and a rectangle.
    pub fn keyhole(
        circle_radius: Real,
        handle_width: Real,
        handle_height: Real,
        segments: usize,
    ) -> Profile {
        if segments < 3
            || !hprofile_scalar_positive(&circle_radius)
            || !hprofile_scalar_positive(&handle_width)
            || !hprofile_scalar_positive(&handle_height)
        {
            return Profile::empty();
        }
        // 1) Circle
        let circle = Profile::circle(circle_radius, segments);

        // 2) Rectangle (handle)
        let handle_offset = -(&handle_width * 0.5);
        let handle = Profile::rectangle(handle_width, handle_height).translate(
            handle_offset,
            Real::zero(),
            Real::zero(),
        );

        // 3) Union them
        circle.union(&handle)
    }

    /// Reuleaux polygon (constant–width curve) built as the *intersection* of
    /// `sides` equal–radius disks whose centres are the vertices of a regular
    /// n-gon.
    ///
    /// * `sides`                  ≥ 3  
    /// * `diameter`               desired constant width (the distance between opposite vertices)
    /// * `circle_segments`        how many segments to use for each disk
    ///
    /// For `sides == 3` this gives the canonical Reuleaux triangle. Higher
    /// odd side counts produce constant-width Reuleaux polygons.
    pub fn reuleaux(sides: usize, diameter: Real, circle_segments: usize) -> Profile {
        if sides < 3
            || sides.is_multiple_of(2)
            || circle_segments < 6
            || !hprofile_scalar_positive(&diameter)
        {
            return Profile::empty();
        }

        // Opposite vertices of an odd regular polygon are separated by the
        // requested width: W = 2 R cos(pi / (2 n)).
        let Some(half_angle) = (Real::pi() / Real::from((2 * sides) as u64)).ok() else {
            return Profile::empty();
        };
        let cos_half_angle = half_angle.cos();
        let Some(denom) = hreal_mul(2.0, cos_half_angle) else {
            return Profile::empty();
        };
        let Some(r_circ) = hreal_div(&diameter, denom) else {
            return Profile::empty();
        };

        // Pre-compute vertex positions of the regular n-gon
        let mut verts = Vec::with_capacity(sides);
        for i in 0..sides {
            let Some([x, y]) =
                sampled_ellipse_point(&r_circ, &r_circ, i, sides, &Real::zero(), &Real::tau())
            else {
                return Profile::empty();
            };
            verts.push((x, y));
        }

        // Build the first disk and use it as the running intersection
        let Some(base) = translated_circle_profile(
            diameter.clone(),
            circle_segments,
            &verts[0].0,
            &verts[0].1,
        ) else {
            return Profile::empty();
        };

        verts.iter().skip(1).fold(base, |acc, (x, y)| {
            translated_circle_profile(diameter.clone(), circle_segments, x, y)
                .map_or_else(Profile::empty, |disk| acc.intersection(&disk))
        })
    }

    /// Outer diameter = `id + 2*thickness`. This yields an annulus in the XY plane.
    /// `segments` controls how smooth the outer/inner circles are.
    ///
    /// Radius arithmetic is promoted before the annular profile is composed
    /// from hypercurve-backed circle regions, following Yap's exact-geometric
    /// computation boundary split (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn ring(id: Real, thickness: Real, segments: usize) -> Profile {
        if !hprofile_scalar_positive(&id)
            || !hprofile_scalar_positive(&thickness)
            || segments < 3
        {
            return Profile::empty();
        }
        let Some(inner_radius) = hreal_mul(0.5, &id) else {
            return Profile::empty();
        };
        let Some(outer_radius) = hreal_affine(&inner_radius, 1.0, thickness) else {
            return Profile::empty();
        };

        let outer_circle = Profile::circle(outer_radius, segments);
        let inner_circle = Profile::circle(inner_radius, segments);

        outer_circle.difference(&inner_circle)
    }

    /// Create a 2D "pie slice" (wedge) in the XY plane.
    /// - `radius`: outer radius of the slice.
    /// - `start_angle_deg`: starting angle in degrees (measured from X-axis).
    /// - `end_angle_deg`: ending angle in degrees.
    /// - `segments`: how many segments to use to approximate the arc.
    pub fn pie_slice(
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
        segments: usize,
    ) -> Profile {
        if segments < 1 || !hprofile_scalar_positive(&radius) {
            return Profile::empty();
        }

        let Some(sweep_degrees_real) = hreal_sub(&end_angle_deg, &start_angle_deg) else {
            return Profile::empty();
        };
        let Some(sweep_magnitude) = hreal_abs(&sweep_degrees_real) else {
            return Profile::empty();
        };
        if !hfinite_nonzero(sweep_degrees_real.clone())
            || !matches!(
                hreal_try_cmp(&sweep_magnitude, 360.0),
                Some(Ordering::Less | Ordering::Equal)
            )
        {
            return Profile::empty();
        }
        if matches!(hreal_try_cmp(&sweep_magnitude, 360.0), Some(Ordering::Equal)) {
            return Profile::circle(radius, segments);
        }

        let start_rad = (start_angle_deg * Real::pi() / Real::from(180_u16)).ok();
        let sweep = (sweep_degrees_real * Real::pi() / Real::from(180_u16)).ok();
        let (Some(start_rad), Some(sweep)) = (start_rad, sweep) else {
            return Profile::empty();
        };

        // Build a ring of coordinates starting at (0,0), going around the arc, and closing at (0,0).
        let mut points = Vec::with_capacity(segments + 2);
        points.push([Real::zero(), Real::zero()]);
        for i in 0..=segments {
            let Some(angle) = exact_sample_angle(i, segments, &start_rad, &sweep) else {
                return Profile::empty();
            };
            points.push(exact_polar(&radius, angle));
        }

        tessellation_profile(&points)
    }

    /// Create a 2D supershape in the XY plane, approximated by `segments` edges.
    /// The superformula parameters are typically:
    ///   r(θ) = [ (|cos(mθ/4)/a|^n2 + |sin(mθ/4)/b|^n3) ^ (-1/n1) ]
    /// Adjust as needed for your use-case.
    ///
    /// Radius sampling follows Gielis' superformula, introduced in "A generic
    /// geometric transformation that unifies a wide range of natural and
    /// abstract shapes," *American Journal of Botany* 90(3), 2003
    /// (<https://doi.org/10.3732/ajb.90.3.333>). The scalar path is promoted to
    /// hyperreal arithmetic before the finite ring is exported to hypercurve,
    /// following Yap's exact-geometric-computation boundary discipline
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[allow(clippy::too_many_arguments)]
    pub fn supershape(
        a: Real,
        b: Real,
        m: Real,
        n1: Real,
        n2: Real,
        n3: Real,
        segments: usize,
    ) -> Profile {
        if segments < 3
            || !hprofile_scalar_positive(&a)
            || !hprofile_scalar_positive(&b)
            || !hfinite_nonzero(n1.clone())
        {
            return Profile::empty();
        }

        let values = [&a, &b, &m, &n1, &n2, &n3].map(|value| value.to_f64_lossy());
        let [Some(a), Some(b), Some(m), Some(n1), Some(n2), Some(n3)] = values else {
            return Profile::empty();
        };
        let mut points = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(theta) = exact_sample_angle(i, segments, &Real::zero(), &Real::tau())
                .and_then(|angle| angle.to_f64_lossy())
            else {
                return Profile::empty();
            };
            let angle = m * theta * 0.25;
            let sum = (angle.cos() / a).abs().powf(n2) + (angle.sin() / b).abs().powf(n3);
            let radius = sum.powf(-1.0 / n1);
            if !radius.is_finite() {
                return Profile::empty();
            }
            points.push([radius * theta.cos(), radius * theta.sin()]);
        }

        Contour2::from_finite_ring(&points)
            .map(Profile::from_contour)
            .unwrap_or_else(|_| Profile::empty())
    }

    /// Creates a 2D circle with a rectangular keyway slot cut out on the +X side.
    ///
    /// The boolean operands are hypercurve-backed profiles; the remaining
    /// primitive offsets are evaluated through hyperreal helpers before
    /// composition (Yap, EGC, <https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn circle_with_keyway(
        radius: Real,
        segments: usize,
        key_width: Real,
        key_depth: Real,
    ) -> Profile {
        if segments < 3
            || !hprofile_scalar_positive(&radius)
            || !hprofile_scalar_positive(&key_width)
            || !hprofile_scalar_positive(&key_depth)
        {
            return Profile::empty();
        }
        let Some(diameter) = hreal_mul(2.0, &radius) else {
            return Profile::empty();
        };
        if !hprofile_scalar_gt(&diameter, &key_width)
            || !hprofile_scalar_gt(&diameter, &key_depth)
        {
            return Profile::empty();
        }
        // 1. Full circle
        let circle = Profile::circle(radius.clone(), segments);

        // 2. Construct the keyway rectangle
        let Some(key_x) = hreal_sub(&radius, &key_depth) else {
            return Profile::empty();
        };
        let Some(key_y) = hreal_mul(-0.5, &key_width) else {
            return Profile::empty();
        };
        let key_rect =
            Profile::rectangle(key_depth, key_width).translate(key_x, key_y, Real::zero());

        circle.difference(&key_rect)
    }

    /// Creates a 2D "D" shape (circle with one flat chord).
    /// `radius` is the circle radius,
    /// `flat_dist` is how far from the center the flat chord is placed.
    ///
    /// Cutter dimensions and offsets are evaluated in hyperreal arithmetic
    /// before the result is composed from hypercurve regions.
    pub fn circle_with_flat(radius: Real, segments: usize, flat_dist: Real) -> Profile {
        if segments < 3
            || !hprofile_scalar_positive(&radius)
            || !matches!(
                hreal_try_cmp(&flat_dist, 0.0),
                Some(Ordering::Greater | Ordering::Equal)
            )
            || !matches!(hreal_try_cmp(&flat_dist, &radius), Some(Ordering::Less))
        {
            return Profile::empty();
        }
        // 1. Full circle
        let circle = Profile::circle(radius.clone(), segments);

        // Cut exactly the portion of the circle below y = -flat_dist.
        let (Some(width), Some(neg_radius), Some(cutter_height)) = (
            hreal_mul(2.0, &radius),
            hreal_sub(0.0, &radius),
            hreal_sub(&radius, &flat_dist),
        ) else {
            return Profile::empty();
        };
        let rect_cutter = Profile::rectangle(width, cutter_height).translate(
            neg_radius,
            -radius,
            Real::zero(),
        );

        // 3. Subtract to produce the flat chord
        circle.difference(&rect_cutter)
    }

    /// Circle with two parallel flat chords on opposing sides (e.g., "double D" shape).
    /// `radius`   => circle radius
    /// `segments` => how many segments in the circle approximation
    /// `flat_dist` => half-distance between flats measured from the center.
    ///   - chord at y=+flat_dist  and  chord at y=-flat_dist
    pub fn circle_with_two_flats(radius: Real, segments: usize, flat_dist: Real) -> Profile {
        if segments < 3
            || !hprofile_scalar_positive(&radius)
            || !matches!(
                hreal_try_cmp(&flat_dist, 0.0),
                Some(Ordering::Greater | Ordering::Equal)
            )
            || !matches!(hreal_try_cmp(&flat_dist, &radius), Some(Ordering::Less))
        {
            return Profile::empty();
        }
        // 1. Full circle
        let circle = Profile::circle(radius.clone(), segments);

        // 2. Large rectangle to cut the TOP (above +flat_dist)
        let (Some(width), Some(neg_radius), Some(cutter_height)) = (
            hreal_mul(2.0, &radius),
            hreal_sub(0.0, &radius),
            hreal_sub(&radius, &flat_dist),
        ) else {
            return Profile::empty();
        };
        let top_rect = Profile::rectangle(width.clone(), cutter_height.clone())
            // place bottom at y=flat_dist
            .translate(neg_radius.clone(), flat_dist.clone(), Real::zero());

        // 3. Large rectangle to cut the BOTTOM (below -flat_dist)
        let bottom_rect = Profile::rectangle(width, cutter_height)
            // place top at y=-flat_dist => bottom extends downward
            .translate(neg_radius, -radius, Real::zero());

        // 4. Subtract both
        let with_top_flat = circle.difference(&top_rect);

        with_top_flat.difference(&bottom_rect)
    }

    /// Sample an arbitrary-degree Bezier curve with de Casteljau evaluation.
    /// Returns a poly-line, or a hypercurve-backed filled region when the
    /// tessellated boundary is closed.
    ///
    /// The recursive evaluation is the de Casteljau algorithm developed at
    /// Citroen in 1959; see Farin, *Curves and Surfaces for CAGD*, 5th ed.,
    /// 2002, Chapter 4. Closed output is promoted immediately to a
    /// `hypercurve::Region2` so downstream topology uses exact predicates
    /// rather than the finite tessellation cache.
    ///
    /// * `control`: list of 2-D control points
    /// * `segments`: number of straight-line segments used for the tessellation
    pub fn bezier(control: &[[Real; 2]], segments: usize) -> Self {
        if control.len() < 2 || segments < 1 {
            return Profile::empty();
        }

        /// Evaluates a Bézier curve using de Casteljau interpolation in hyperreal space.
        fn de_casteljau(control: &[[Real; 2]], t: Real) -> Option<(Real, Real)> {
            let mut points = control.to_vec();
            let n = points.len();

            for k in 1..n {
                for i in 0..(n - k) {
                    let next = hxy_lerp(
                        (points[i][0].clone(), points[i][1].clone()),
                        (points[i + 1][0].clone(), points[i + 1][1].clone()),
                        t.clone(),
                    )?;
                    points[i] = [next.0, next.1];
                }
            }
            Some((points[0][0].clone(), points[0][1].clone()))
        }

        let mut pts = Vec::with_capacity(segments + 1);
        for i in 0..=segments {
            let Some(t) = hreal_div(i, segments) else {
                return Profile::empty();
            };
            let Some((x, y)) = de_casteljau(control, t) else {
                return Profile::empty();
            };
            pts.push([x, y]);
        }

        let is_closed = {
            let first = &pts[0];
            let last = &pts[segments];
            let first = hyperlimit::Point2::new(first[0].clone(), first[1].clone());
            let last = hyperlimit::Point2::new(last[0].clone(), last[1].clone());
            matches!(hyperlimit::point2_equal(&first, &last).value(), Some(true))
        };

        if is_closed {
            return Self::polygonal_region(pts);
        }

        CurveString2::from_real_point_iter(pts)
            .map(Profile::from_wire)
            .unwrap_or_else(|_| Profile::empty())
    }

    /// Sample an open-uniform B-spline of arbitrary degree (`p`) using the
    /// Cox-de Boor recursion. Returns a poly-line, or a hypercurve-backed
    /// filled region when the sampled spline closes.
    ///
    /// The basis evaluation follows de Boor, "On calculating with B-splines",
    /// *Journal of Approximation Theory* 6(1), 1972,
    /// DOI: 10.1016/0021-9045(72)90080-9. Closed output is converted to
    /// `hypercurve::Region2` at the API boundary; open output becomes native
    /// `hypercurve::CurveString2` wires and is projected to finite line strings
    /// only for compatibility/export boundaries.
    ///
    /// * `control`: control points  
    /// * `p`:       spline degree (e.g. 3 for a cubic)  
    /// * `segments_per_span`: tessellation resolution inside every knot span
    pub fn bspline(control: &[[Real; 2]], p: usize, segments_per_span: usize) -> Self {
        if control.len() < p + 1 || segments_per_span < 1 {
            return Profile::empty();
        }

        let n = control.len() - 1;
        let m = n + p + 1; // highest knot index
        let span_count = n - p + 1;
        // open-uniform knot vector: 0,0,…,0,1,2,…,n-p-1,(n-p),…,(n-p)
        let mut knot = Vec::<Real>::with_capacity(m + 1);
        for i in 0..=m {
            if i <= p {
                knot.push(Real::zero());
            } else if i >= m - p {
                knot.push(Real::from(span_count as u64));
            } else {
                knot.push(Real::from((i - p) as u64));
            }
        }

        // Iterative de Boor evaluation avoids the exponential expression graph
        // produced by recursively evaluating every Cox-de Boor basis function.
        fn de_boor(
            control: &[[Real; 2]],
            knot: &[Real],
            degree: usize,
            span: usize,
            u: &Real,
        ) -> Option<[Real; 2]> {
            let k = degree + span;
            let mut points = control[k - degree..=k].to_vec();
            for level in 1..=degree {
                for j in (level..=degree).rev() {
                    let i = k - degree + j;
                    let denominator = hreal_sub(&knot[i + degree - level + 1], &knot[i])?;
                    if hreal_f64s_exactly_equal(&denominator, 0.0) {
                        continue;
                    }
                    let alpha = hreal_div(hreal_sub(u, &knot[i])?, denominator)?;
                    let x = hreal_affine(
                        &points[j - 1][0],
                        &alpha,
                        hreal_sub(&points[j][0], &points[j - 1][0])?,
                    )?;
                    let y = hreal_affine(
                        &points[j - 1][1],
                        &alpha,
                        hreal_sub(&points[j][1], &points[j - 1][1])?,
                    )?;
                    points[j] = [x, y];
                }
            }
            points.pop()
        }

        let Some(dt) = hreal_div(1.0, segments_per_span) else {
            return Profile::empty();
        };

        let mut pts = Vec::<[Real; 2]>::new();
        for span in 0..span_count {
            for s in 0..=segments_per_span {
                if span > 0 && s == 0 {
                    continue;
                }
                if span + 1 == span_count && s == segments_per_span {
                    // The clamped endpoint is appended exactly below.
                    continue;
                }
                let Some(u) = hreal_affine(span, s, dt.clone()) else {
                    return Profile::empty();
                };
                let Some(point) = de_boor(control, &knot, p, span, &u) else {
                    return Profile::empty();
                };
                pts.push(point);
            }
        }
        if let Some(last) = control.last() {
            pts.push(last.clone());
        }

        let first = pts.first().unwrap();
        let last = pts.last().unwrap();
        let first = hyperlimit::Point2::new(first[0].clone(), first[1].clone());
        let last = hyperlimit::Point2::new(last[0].clone(), last[1].clone());
        let closed = matches!(hyperlimit::point2_equal(&first, &last).value(), Some(true));
        if !closed {
            return CurveString2::from_real_point_iter(pts)
                .map(Profile::from_wire)
                .unwrap_or_else(|_| Profile::empty());
        }

        Self::polygonal_region(pts)
    }

    /// 2-D heart outline (closed polygon) sized to `width` × `height`.
    ///
    /// `segments` controls smoothness (≥ 8 recommended).
    ///
    /// The classic analytic heart curve is sampled and normalized through
    /// `Real` before finite coordinates are exported to hypercurve.
    /// This follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), keeping constructor
    /// algebra out of local primitive arithmetic.
    pub fn heart(width: Real, height: Real, segments: usize) -> Self {
        if segments < 8
            || !hprofile_scalar_positive(&width)
            || !hprofile_scalar_positive(&height)
        {
            return Profile::empty();
        }

        let mut raw = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(t) = exact_sample_angle(i, segments, &Real::zero(), &Real::tau()) else {
                return Profile::empty();
            };
            let values = [
                t.clone().sin(),
                t.clone().cos(),
                (Real::from(2_u8) * t.clone()).cos(),
                (Real::from(3_u8) * t.clone()).cos(),
                (Real::from(4_u8) * t).cos(),
            ]
            .map(|value| value.to_f64_lossy());
            let [Some(sin), Some(cos), Some(cos_2), Some(cos_3), Some(cos_4)] = values else {
                return Profile::empty();
            };
            raw.push((
                16.0 * sin.powi(3),
                13.0 * cos - 5.0 * cos_2 - 2.0 * cos_3 - cos_4,
            ));
        }
        let min_x = raw.iter().map(|point| point.0).fold(f64::INFINITY, f64::min);
        let max_x = raw
            .iter()
            .map(|point| point.0)
            .fold(f64::NEG_INFINITY, f64::max);
        let min_y = raw.iter().map(|point| point.1).fold(f64::INFINITY, f64::min);
        let max_y = raw
            .iter()
            .map(|point| point.1)
            .fold(f64::NEG_INFINITY, f64::max);
        let points = raw
            .into_iter()
            .map(|(x, y)| -> Option<[Real; 2]> {
                Some([
                    width.clone() * hreal_from_f64((x - min_x) / (max_x - min_x)).ok()?,
                    height.clone() * hreal_from_f64((y - min_y) / (max_y - min_y)).ok()?,
                ])
            })
            .collect::<Option<Vec<_>>>();
        points.map_or_else(Profile::empty, Self::polygonal_region)
    }

    /// 2-D crescent obtained by subtracting a displaced smaller circle
    /// from a larger one.
    /// `segments` controls circle smoothness.
    ///
    /// ```ignore
    /// use csgrs::profile::Profile;
    /// let cres = Profile::crescent(2.0, 1.4, 0.8, 64 );
    /// ```
    pub fn crescent(outer_r: Real, inner_r: Real, offset: Real, segments: usize) -> Self {
        if segments < 6
            || !hprofile_scalar_gt(&outer_r, &inner_r)
            || !hprofile_scalar_positive(&inner_r)
        {
            return Profile::empty();
        }

        let big = Self::circle(outer_r, segments);
        let small =
            Self::circle(inner_r, segments).translate(offset, Real::zero(), Real::zero());

        big.difference(&small)
    }

    /// Generate an involute gear outline
    ///
    /// # Parameters
    /// - `module_`: gear module (pitch diameter / number of teeth)
    /// - `teeth`: number of teeth (>= 4)
    /// - `pressure_angle_deg`: pressure angle in degrees (typically 20°)
    /// - `clearance`: additional clearance for dedendum
    /// - `backlash`: backlash allowance
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    pub fn involute_gear(
        module: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
    ) -> Profile {
        sampled_involute_gear(
            &module,
            teeth,
            &pressure_angle_deg,
            &clearance,
            &backlash,
            segments_per_flank,
        )
    }

    /// Generate an (epicyclic) cycloidal gear outline
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `teeth`: number of teeth (>= 3)
    /// - `generating_radius`: rolling-circle radius; must reach the addendum
    ///   (`>= module / 2`) and remain smaller than half the pitch radius
    /// - `clearance`: additional clearance for dedendum
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    ///
    /// * Each tooth is defined in its own local angular frame around a centre
    ///   angle φ_c.
    /// * The tooth profile is defined in polar coordinates r(φ), symmetric
    ///   about φ_c, so tips sit directly above their bases.
    ///
    /// Addendum flanks are epicycloids and dedendum flanks are hypocycloids,
    /// clipped at the standard addendum and dedendum circles.
    ///
    /// Parameter admission uses exact hyperreal predicates. The analytic
    /// cycloids are sampled at a finite boundary and represented by exact
    /// binary-rational line topology afterward.
    pub fn cycloidal_gear(
        module: Real,
        teeth: usize,
        generating_radius: Real,
        clearance: Real,
        segments_per_flank: usize,
    ) -> Profile {
        sampled_cycloidal_gear(
            &module,
            teeth,
            &generating_radius,
            &clearance,
            segments_per_flank,
        )
    }

    /// Generate a linear involute rack profile (lying in the XY plane, pitch‑line on Y = 0).
    /// The returned polygon is CCW and spans `num_teeth` pitches along +X.
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `num_teeth`: number of teeth along the rack
    /// - `pressure_angle_deg`: pressure angle in degrees
    /// - `clearance`: additional clearance for dedendum
    /// - `backlash`: backlash allowance
    ///
    /// Parameter admission uses exact hyperreal predicates; finite flank
    /// coordinates are promoted to exact dyadic topology. For involute rack
    /// geometry conventions see Litvin and Fuentes, *Gear Geometry and Applied
    /// Theory*, 2nd ed., Cambridge University Press, 2004.
    pub fn involute_rack(
        module_: Real,
        num_teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
    ) -> Profile {
        if num_teeth < 1
            || !hprofile_scalar_positive(&module_)
            || !hprofile_scalar_nonnegative(&clearance)
            || !hprofile_scalar_nonnegative(&backlash)
            || !matches!(
                hreal_try_cmp(&pressure_angle_deg, 0.0),
                Some(Ordering::Greater)
            )
            || !matches!(hreal_try_cmp(&pressure_angle_deg, 90.0), Some(Ordering::Less))
        {
            return Profile::empty();
        }
        let Some(pressure_angle) =
            (pressure_angle_deg * Real::pi() / Real::from(180_u16)).ok()
        else {
            return Profile::empty();
        };
        let pitch = Real::pi() * module_.clone();
        let dedendum = module_.clone()
            * (Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero")
            + clearance;
        let root_y = -dedendum.clone();
        let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
        let tooth_thickness = pitch.clone() * half.clone() - backlash;
        let half_thickness = tooth_thickness.clone() * half;
        let Some(tan_alpha) = (pressure_angle.clone().sin() / pressure_angle.cos()).ok()
        else {
            return Profile::empty();
        };
        let root_slant = dedendum.clone() * tan_alpha.clone();
        let tip_slant = module_.clone() * tan_alpha;
        let tip_width = tooth_thickness.clone() - Real::from(2_u8) * tip_slant.clone();
        let root_space =
            pitch.clone() - tooth_thickness.clone() - Real::from(2_u8) * root_slant.clone();
        if !hprofile_scalar_positive(&tooth_thickness)
            || !hprofile_scalar_positive(&tip_width)
            || !hprofile_scalar_nonnegative(&root_space)
        {
            return Profile::empty();
        }
        let first_x = -half_thickness.clone() - root_slant.clone();
        let mut outline = Vec::<[Real; 2]>::with_capacity(6 * num_teeth + 1);
        outline.push([first_x.clone(), root_y.clone()]);

        for i in 0..num_teeth {
            let tooth_center = Real::from(i as u64) * pitch.clone();
            let left_pitch = tooth_center.clone() - half_thickness.clone();
            let right_pitch = tooth_center + half_thickness.clone();
            outline.push([left_pitch.clone(), Real::zero()]);
            outline.push([left_pitch + tip_slant.clone(), module_.clone()]);
            outline.push([right_pitch.clone() - tip_slant.clone(), module_.clone()]);
            outline.push([right_pitch.clone(), Real::zero()]);
            outline.push([right_pitch + root_slant.clone(), root_y.clone()]);

            if i < num_teeth - 1 {
                let next_left_pitch =
                    Real::from((i + 1) as u64) * pitch.clone() - half_thickness.clone();
                outline.push([next_left_pitch - root_slant.clone(), root_y.clone()]);
            }
        }

        tessellation_profile(&outline)
    }

    /// Generate a linear cycloidal rack profile.
    ///
    /// A generating circle of radius `module / 2` rolls along the pitch line,
    /// so one full cycloid spans the linear pitch `pi * module`.
    ///
    /// The cycloid is sampled at the finite tessellation boundary and promoted
    /// to exact dyadic line topology in `hypercurve`. For the gear-geometry
    /// background see Litvin and Fuentes, *Gear Geometry and Applied Theory*,
    /// 2nd ed., Cambridge University Press, 2004.
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `num_teeth`: number of teeth along the rack
    /// - `clearance`: additional clearance for dedendum
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    pub fn cycloidal_rack(
        module_: Real,
        num_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
    ) -> Profile {
        if num_teeth < 1
            || segments_per_flank < 4
            || !hprofile_scalar_positive(&module_)
            || !hprofile_scalar_nonnegative(&clearance)
        {
            return Profile::empty();
        }
        let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
        let generating_radius = module_.clone() * half.clone();
        let pitch = Real::pi() * module_.clone();
        let root_y = -(module_.clone()
            * (Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero")
            + clearance);

        let left_edge = -half.clone() * pitch.clone();
        let right_edge = (Real::from(num_teeth as u64) - half) * pitch.clone();
        let mut top = Vec::<[Real; 2]>::with_capacity(num_teeth * segments_per_flank + 1);
        for tooth in 0..num_teeth {
            let tooth_left = (Real::from(tooth as u64)
                - (Real::one() / Real::from(2_u8)).expect("two is nonzero"))
                * pitch.clone();
            for sample in 0..=segments_per_flank {
                if tooth > 0 && sample == 0 {
                    continue;
                }
                let Some(fraction) = exact_ratio(sample, segments_per_flank) else {
                    return Profile::empty();
                };
                let theta = Real::tau() * fraction;
                top.push([
                    tooth_left.clone()
                        + generating_radius.clone() * (theta.clone() - theta.clone().sin()),
                    generating_radius.clone() * (Real::one() - theta.cos()),
                ]);
            }
        }

        let mut outline = Vec::<[Real; 2]>::with_capacity(top.len() + 3);
        outline.push([left_edge.clone(), root_y.clone()]);
        outline.push([right_edge, root_y.clone()]);
        for point in top.into_iter().rev() {
            outline.push(point);
        }
        tessellation_profile(&outline)
    }

    /// Generate a NACA 4-digit airfoil (e.g. "2412", "0015").
    ///
    /// ## Parameters
    /// - `max_camber`: max camber %, the first digit
    /// - `camber_position`: camber position, the second digit
    /// - `thickness`: thickness %, the last two digits
    /// - `chord`: physical chord length you want (same units as the rest of your model)
    /// - `samples`: number of points per surface (≥ 10 is required; NP total = 2 × samples + 1)
    ///
    /// The function returns a single closed polygon lying in the *XY* plane with its
    /// leading edge at the origin and the chord running along +X.
    ///
    /// The 4-digit thickness and camber equations trace back to Jacobs, Ward,
    /// and Pinkerton, "The characteristics of 78 related airfoil sections from
    /// tests in the variable-density wind tunnel", NACA Report 460, 1933.
    /// The analytic boundary is evaluated at the finite tessellation boundary
    /// and promoted to exact dyadic `hypercurve::Region2` topology.
    pub fn airfoil_naca4(
        max_camber: Real,
        camber_position: Real,
        thickness: Real,
        chord: Real,
        samples: usize,
    ) -> Profile {
        if samples < 10
            || !hprofile_scalar_nonnegative(&max_camber)
            || !matches!(hreal_try_cmp(&max_camber, 10.0), Some(Ordering::Less))
            || !hprofile_scalar_nonnegative(&camber_position)
            || !matches!(hreal_try_cmp(&camber_position, 10.0), Some(Ordering::Less))
            || !hprofile_scalar_positive(&chord)
            || !matches!(hreal_try_cmp(&thickness, 0.0), Some(Ordering::Greater))
            || !matches!(hreal_try_cmp(&thickness, 100.0), Some(Ordering::Less))
        {
            return Profile::empty();
        }
        let Some(m) = (max_camber / Real::from(100_u8)).ok() else {
            return Profile::empty();
        };
        let Some(p) = (camber_position / Real::from(10_u8)).ok() else {
            return Profile::empty();
        };
        let Some(thickness) = (thickness / Real::from(100_u8)).ok() else {
            return Profile::empty();
        };
        let cambered = !matches!(hreal_try_cmp(&m, Real::zero()), Some(Ordering::Equal));
        if cambered
            && (!matches!(hreal_try_cmp(&p, Real::zero()), Some(Ordering::Greater))
                || !matches!(hreal_try_cmp(&p, Real::one()), Some(Ordering::Less)))
        {
            return Profile::empty();
        }

        let coefficient = |value: f64| hreal_from_f64(value).expect("finite NACA coefficient");

        let sample = |x: Real| -> Option<(Real, Real, Real, Real, Real)> {
            let x2 = x.clone() * x.clone();
            let x3 = x2.clone() * x.clone();
            let x4 = x3.clone() * x.clone();
            let yt = Real::from(5_u8)
                * thickness.clone()
                * (coefficient(0.2969) * x.clone().sqrt().ok()?
                    - coefficient(0.1260) * x.clone()
                    - coefficient(0.3516) * x2.clone()
                    + coefficient(0.2843) * x3
                    - coefficient(0.1015) * x4);
            let (yc, dy) = if !cambered {
                (Real::zero(), Real::zero())
            } else if matches!(hreal_try_cmp(&x, &p), Some(Ordering::Less)) {
                let p2 = p.clone() * p.clone();
                (
                    (m.clone() / p2.clone()).ok()?
                        * (Real::from(2_u8) * p.clone() * x.clone() - x2.clone()),
                    (Real::from(2_u8) * m.clone() / p2).ok()? * (p.clone() - x.clone()),
                )
            } else {
                let one_minus_p = Real::one() - p.clone();
                let denominator = one_minus_p.clone() * one_minus_p;
                (
                    (m.clone() / denominator.clone()).ok()?
                        * (Real::one() - Real::from(2_u8) * p.clone()
                            + Real::from(2_u8) * p.clone() * x.clone()
                            - x2),
                    (Real::from(2_u8) * m.clone() / denominator).ok()?
                        * (p.clone() - x.clone()),
                )
            };
            let theta = dy.atan().ok()?;
            Some((x, yc, yt, theta.clone().sin(), theta.cos()))
        };

        let mut points = Vec::with_capacity(2 * samples);
        for i in 0..=samples {
            let Some((x, yc, yt, sin_theta, cos_theta)) =
                exact_ratio(i, samples).and_then(&sample)
            else {
                return Profile::empty();
            };
            points.push([
                chord.clone() * (x - yt.clone() * sin_theta),
                chord.clone() * (yc + yt * cos_theta),
            ]);
        }
        for i in (1..samples).rev() {
            let Some((x, yc, yt, sin_theta, cos_theta)) =
                exact_ratio(i, samples).and_then(&sample)
            else {
                return Profile::empty();
            };
            points.push([
                chord.clone() * (x + yt.clone() * sin_theta),
                chord.clone() * (yc - yt * cos_theta),
            ]);
        }
        tessellation_profile(&points)
    }

    /// Build a Hilbert-curve path that fills this sketch.
    /// - `order`: recursion order in `1..=10` (number of points = 4^order).
    /// - `padding`: optional inset from the bounding-box edges (same units as the sketch).
    ///   Returns a new `Profile` containing only the inside segments as native wires.
    ///
    /// Bounds are taken from the native hypercurve region/wire topology rather
    /// than from any finite compatibility cache. That keeps Hilbert
    /// path construction on the same exact-object side of the API boundary as
    /// the containment tests it uses; see Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn hilbert_curve(&self, order: usize, padding: Real) -> Profile {
        if order == 0 || order > 10 || !hprofile_scalar_nonnegative(&padding) {
            return Profile::empty();
        }
        let bounds = self.native_xy_bounds();
        let Some((min_x, min_y, max_x, max_y)) = bounds else {
            return Profile::empty();
        };

        // Bounding box and usable region (with padding).
        let Some(raw_w) = hreal_sub(&max_x, &min_x) else {
            return Profile::empty();
        };
        let Some(raw_h) = hreal_sub(&max_y, &min_y) else {
            return Profile::empty();
        };
        if !hprofile_scalar_positive(&raw_w) || !hprofile_scalar_positive(&raw_h) {
            return Profile::empty();
        }
        let w = raw_w;
        let h = raw_h;
        let Some(ox) = hreal_sum(&[min_x, padding.clone()]) else {
            return Profile::empty();
        };
        let Some(oy) = hreal_sum(&[min_y, padding.clone()]) else {
            return Profile::empty();
        };
        let Some(double_padding) = hreal_mul(2.0, &padding) else {
            return Profile::empty();
        };
        let Some(raw_sx) = hreal_sub(&w, &double_padding) else {
            return Profile::empty();
        };
        let Some(raw_sy) = hreal_sub(&h, &double_padding) else {
            return Profile::empty();
        };
        if !hprofile_scalar_positive(&raw_sx) || !hprofile_scalar_positive(&raw_sy) {
            return Profile::empty();
        }
        let sx = raw_sx;
        let sy = raw_sy;

        // Generate normalized Hilbert points in [0,1]^2, then scale/translate.
        let pts_norm = hilbert_points(order);
        let mut pts: Vec<(Real, Real)> = Vec::with_capacity(pts_norm.len());
        for (u, v) in pts_norm {
            let Some(x) = hreal_affine(&ox, &u, &sx) else {
                return Profile::empty();
            };
            let Some(y) = hreal_affine(&oy, &v, &sy) else {
                return Profile::empty();
            };
            pts.push((x, y));
        }

        let mut runs: Vec<Vec<(Real, Real)>> = Vec::new();
        let mut run: Vec<(Real, Real)> = Vec::new();

        for w in pts.windows(2) {
            let a = &w[0];
            let b = &w[1];
            let Some(sum_x) = hreal_sum(&[a.0.clone(), b.0.clone()]) else {
                continue;
            };
            let Some(sum_y) = hreal_sum(&[a.1.clone(), b.1.clone()]) else {
                continue;
            };
            let Some(mid_x) = hreal_mul(sum_x, 0.5) else {
                continue;
            };
            let Some(mid_y) = hreal_mul(sum_y, 0.5) else {
                continue;
            };
            let keep = self.contains_xy(mid_x, mid_y).unwrap_or(false);

            if keep {
                if run.is_empty() {
                    run.push(a.clone());
                }
                run.push(b.clone());
            } else {
                if run.len() >= 2 {
                    runs.push(std::mem::take(&mut run));
                }
                run.clear();
            }
        }
        if run.len() >= 2 {
            runs.push(run);
        }

        // Emit as native hypercurve wires only. Hilbert's space-filling curve
        // is a classical locality-preserving infill path; see Hilbert, "Ueber
        // die stetige Abbildung einer Linie auf ein Flächenstück," *Math.
        // Ann.* 38, 1891 (<https://doi.org/10.1007/BF01199431>).
        let wires = runs
            .into_iter()
            .filter_map(|run| {
                CurveString2::from_real_point_iter(run.into_iter().map(|(x, y)| [x, y])).ok()
            })
            .collect::<Vec<_>>();
        Profile::from_wires(wires)
    }
}

/// Generate Hilbert-curve points normalized to the unit square.
/// Order `n` yields 4^n points, ordered along the path.
fn hilbert_points(order: usize) -> Vec<(Real, Real)> {
    #[allow(
        clippy::too_many_arguments,
        reason = "This should be refactored in the future, but it's blocking CI at the moment."
    )]
    fn recur(
        out: &mut Vec<(Real, Real)>,
        x0: Real,
        y0: Real,
        xi: Real,
        xj: Real,
        yi: Real,
        yj: Real,
        n: usize,
    ) {
        if n == 0 {
            out.push((&x0 + &((&xi + &yi) * 0.5), &y0 + &((&xj + &yj) * 0.5)));
        } else {
            let (xi2, xj2) = (&xi * 0.5, &xj * 0.5);
            let (yi2, yj2) = (&yi * 0.5, &yj * 0.5);
            recur(
                out,
                x0.clone(),
                y0.clone(),
                yi2.clone(),
                yj2.clone(),
                xi2.clone(),
                xj2.clone(),
                n - 1,
            );
            recur(
                out,
                &x0 + &xi2,
                &y0 + &xj2,
                xi2.clone(),
                xj2.clone(),
                yi2.clone(),
                yj2.clone(),
                n - 1,
            );
            recur(
                out,
                &(&x0 + &xi2) + &yi2,
                &(&y0 + &xj2) + &yj2,
                xi2.clone(),
                xj2.clone(),
                yi2.clone(),
                yj2.clone(),
                n - 1,
            );
            recur(
                out,
                &(&x0 + &xi2) + &yi,
                &(&y0 + &xj2) + &yj,
                -yi2,
                -yj2,
                -xi2,
                -xj2,
                n - 1,
            );
        }
    }
    let shift: u32 = ((2 * order) as u32).min(usize::BITS - 1);
    let mut pts = Vec::with_capacity(1usize << shift);
    recur(
        &mut pts,
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        order,
    );
    pts
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::{Real, hreal_from_f64, tolerance};
    use hyperreal::SymbolicDependencyMask;

    fn r(value: f64) -> Real {
        hreal_from_f64(value).expect("test values must be finite")
    }

    #[test]
    fn profile_scalar_admission_uses_exact_hyperreal_positivity() {
        assert!(hprofile_scalar_positive(tolerance() * r(0.25)));
        assert!(hprofile_scalar_positive(tolerance()));
        assert!(hprofile_scalar_positive(tolerance() * r(2.0)));
        assert!(!hprofile_scalar_positive(Real::zero()));
        assert!(!hprofile_scalar_positive(-tolerance()));
    }

    #[test]
    fn circular_profile_samples_retain_symbolic_pi_before_projection() {
        let points = hcircle_samples(7, Real::one()).unwrap();
        assert!(points.iter().flatten().any(|coordinate| {
            coordinate
                .detailed_facts()
                .symbolic
                .dependencies
                .contains(SymbolicDependencyMask::PI)
        }));
    }

    #[test]
    fn symbolic_ring_difference_remains_decided() {
        assert!(!Profile::ring(r(2.0), r(0.5), 24).is_empty());
    }

    #[test]
    fn symbolic_circle_keyway_difference_remains_decided() {
        assert!(!Profile::circle_with_keyway(r(3.0), 24, r(1.0), r(1.0)).is_empty());
    }

    #[test]
    fn symbolic_circle_flat_difference_remains_decided() {
        assert!(!Profile::circle_with_flat(r(3.0), 24, r(1.0)).is_empty());
    }

    #[test]
    fn symbolic_circle_two_flats_difference_remains_decided() {
        assert!(!Profile::circle_with_two_flats(r(3.0), 24, r(1.0)).is_empty());
    }

    #[test]
    fn symbolic_crescent_difference_remains_decided() {
        assert!(!Profile::crescent(r(3.0), r(2.0), r(1.5), 24).is_empty());
    }

    #[test]
    fn bezier_closure_uses_exact_hyperreal_endpoint_identity() {
        let nearly_closed = Profile::bezier(
            &[
                [r(0.0), r(0.0)],
                [r(0.25), r(1.0)],
                [r(0.75), r(1.0)],
                [tolerance() * r(0.25), r(0.0)],
            ],
            8,
        );
        assert_eq!(nearly_closed.material_contour_count(), 0);
        assert_eq!(nearly_closed.wires().len(), 1);

        let exactly_closed = Profile::bezier(
            &[
                [r(0.0), r(0.0)],
                [r(0.25), r(1.0)],
                [r(0.75), r(1.0)],
                [r(0.0), r(0.0)],
            ],
            8,
        );
        assert_eq!(exactly_closed.material_contour_count(), 1);
        assert!(exactly_closed.wires().is_empty());
    }

    #[test]
    fn cubic_bspline_with_minimum_control_count_interpolates_endpoints() {
        let spline = Profile::bspline(
            &[
                [r(0.0), r(0.0)],
                [r(1.0), r(1.0)],
                [r(2.0), r(1.0)],
                [r(3.0), r(0.0)],
            ],
            3,
            8,
        );

        assert_eq!(spline.wires().len(), 1);
        let polylines = spline.wire_polylines();
        let points = &polylines[0];
        assert_eq!(points.first(), Some(&[r(0.0), r(0.0)]));
        assert_eq!(points.last(), Some(&[r(3.0), r(0.0)]));
    }

    #[test]
    fn naca_airfoil_scales_uniformly_with_chord() {
        let unit = Profile::airfoil_naca4(r(2.0), r(4.0), r(12.0), r(1.0), 24);
        let scaled = Profile::airfoil_naca4(r(2.0), r(4.0), r(12.0), r(10.0), 24);
        let unit_box = unit.bounding_box();
        let scaled_box = scaled.bounding_box();
        let unit_bounds = (
            unit_box.mins.x,
            unit_box.mins.y,
            unit_box.maxs.x,
            unit_box.maxs.y,
        );
        let scaled_bounds = (
            scaled_box.mins.x,
            scaled_box.mins.y,
            scaled_box.maxs.x,
            scaled_box.maxs.y,
        );

        for (unit, scaled) in [
            (unit_bounds.0, scaled_bounds.0),
            (unit_bounds.1, scaled_bounds.1),
            (unit_bounds.2, scaled_bounds.2),
            (unit_bounds.3, scaled_bounds.3),
        ] {
            let unit = unit.to_f64_lossy().expect("unit bound must be approximable");
            let scaled = scaled
                .to_f64_lossy()
                .expect("scaled bound must be approximable");
            assert!(
                (scaled - 10.0 * unit).abs() < 1.0e-8,
                "unit bound {unit}, scaled bound {scaled}"
            );
        }
    }

    #[test]
    fn egg_honors_requested_centered_dimensions() {
        let egg = Profile::egg(r(3.0), r(5.0), 24);
        let (min_x, min_y, max_x, max_y) = egg.native_xy_bounds().expect("egg bounds");
        let values = [min_x, min_y, max_x, max_y]
            .map(|value| value.to_f64_lossy().expect("finite egg bound"));
        assert!((values[0] + 1.5).abs() < 1.0e-12);
        assert!((values[1] + 2.5).abs() < 1.0e-12);
        assert!((values[2] - 1.5).abs() < 1.0e-12);
        assert!((values[3] - 2.5).abs() < 1.0e-12);
    }

    #[test]
    fn involute_gear_clearance_controls_root_radius() {
        let minimum_radius = |profile: Profile| {
            profile
                .region_profiles()
                .into_iter()
                .flat_map(|profile| profile.material().points().to_vec())
                .map(|[x, y]| x.hypot(y))
                .fold(f64::INFINITY, f64::min)
        };

        let nominal = Profile::involute_gear(r(2.0), 12, r(20.0), r(0.0), r(0.0), 4);
        let cleared = Profile::involute_gear(r(2.0), 12, r(20.0), r(0.5), r(0.0), 4);
        assert!(!nominal.is_empty());
        assert!(!cleared.is_empty());
        assert!((minimum_radius(nominal) - 9.5).abs() < 1.0e-8);
        assert!((minimum_radius(cleared) - 9.0).abs() < 1.0e-8);
    }

    #[test]
    fn cycloidal_gear_uses_first_lobe_and_clearance_root() {
        let radii = |profile: Profile| {
            profile
                .region_profiles()
                .into_iter()
                .flat_map(|profile| profile.material().points().to_vec())
                .map(|[x, y]| x.hypot(y))
                .collect::<Vec<_>>()
        };

        let nominal = radii(Profile::cycloidal_gear(r(2.0), 12, r(1.0), r(0.0), 8));
        let cleared = radii(Profile::cycloidal_gear(r(2.0), 12, r(1.0), r(0.5), 8));
        assert!((nominal.iter().copied().fold(f64::INFINITY, f64::min) - 9.5).abs() < 1.0e-8);
        assert!((cleared.iter().copied().fold(f64::INFINITY, f64::min) - 9.0).abs() < 1.0e-8);
        assert!(
            (nominal.iter().copied().fold(f64::NEG_INFINITY, f64::max) - 14.0).abs() < 1.0e-8
        );
    }

    #[test]
    fn reuleaux_pentagon_uses_opposite_vertex_width() {
        let profile = Profile::reuleaux(5, r(3.0), 32);
        assert!(!profile.is_empty());
        let bounds = profile.bounding_box();
        let width = (&bounds.maxs.x - &bounds.mins.x)
            .to_f64_lossy()
            .expect("finite Reuleaux width");
        assert!(width > 2.8 && width <= 3.0);
    }

    #[test]
    fn cycloidal_rack_uses_rolling_circle_motion() {
        let profile = Profile::cycloidal_rack(r(2.0), 1, r(0.0), 8);
        let profiles = profile.region_profiles();
        let points = profiles[0].material().points();
        let expected_x = -std::f64::consts::PI + std::f64::consts::FRAC_PI_2 - 1.0;
        assert!(points.iter().any(|point| {
            (point[0] - expected_x).abs() < 1.0e-12 && (point[1] - 1.0).abs() < 1.0e-12
        }));
    }

    #[test]
    fn specialized_profiles_reject_geometrically_invalid_parameters() {
        assert!(Profile::star(2, r(2.0), r(1.0)).is_empty());
        assert!(Profile::teardrop(r(4.0), r(2.0), 16).is_empty());
        assert!(Profile::reuleaux(4, r(3.0), 32).is_empty());
        assert!(Profile::pie_slice(r(2.0), r(0.0), r(361.0), 32).is_empty());
        assert!(Profile::circle_with_keyway(r(2.0), 32, r(4.0), r(1.0)).is_empty());
        assert!(Profile::circle_with_keyway(r(2.0), 32, r(1.0), r(4.0)).is_empty());
        assert!(Profile::involute_rack(r(2.0), 3, r(0.0), r(0.0), r(0.0)).is_empty());
        assert!(Profile::involute_rack(r(2.0), 3, r(20.0), r(-0.1), r(0.0)).is_empty());
        assert!(Profile::involute_rack(r(2.0), 3, r(20.0), r(0.0), r(-0.1)).is_empty());
    }

    #[test]
    fn representative_profile_shape_constructors_produce_topology() {
        let profiles = [
            ("rectangle", Profile::rectangle(r(4.0), r(3.0))),
            ("circle", Profile::circle(r(2.0), 24)),
            ("right_triangle", Profile::right_triangle(r(3.0), r(2.0))),
            ("ellipse", Profile::ellipse(r(4.0), r(2.0), 24)),
            ("regular_ngon", Profile::regular_ngon(7, r(2.0))),
            ("arrow", Profile::arrow(r(3.0), r(1.0), r(1.5), r(2.0))),
            (
                "trapezoid",
                Profile::trapezoid(r(2.0), r(4.0), r(2.0), r(1.0)),
            ),
            ("star", Profile::star(5, r(3.0), r(1.5))),
            ("teardrop", Profile::teardrop(r(3.0), r(5.0), 24)),
            ("egg", Profile::egg(r(3.0), r(5.0), 24)),
            (
                "rounded_rectangle",
                Profile::rounded_rectangle(r(4.0), r(3.0), r(0.5), 4),
            ),
            ("squircle", Profile::squircle(r(4.0), r(3.0), 24)),
            ("keyhole", Profile::keyhole(r(2.0), r(1.0), r(3.0), 24)),
            ("reuleaux", Profile::reuleaux(3, r(3.0), 24)),
            ("reuleaux_pentagon", Profile::reuleaux(5, r(3.0), 24)),
            ("ring", Profile::ring(r(2.0), r(0.5), 24)),
            ("pie_slice", Profile::pie_slice(r(2.0), r(10.0), r(100.0), 12)),
            (
                "supershape",
                Profile::supershape(r(1.0), r(1.0), r(5.0), r(2.0), r(2.0), r(2.0), 32),
            ),
            (
                "circle_with_keyway",
                Profile::circle_with_keyway(r(3.0), 24, r(1.0), r(1.0)),
            ),
            (
                "circle_with_flat",
                Profile::circle_with_flat(r(3.0), 24, r(1.0)),
            ),
            (
                "circle_with_two_flats",
                Profile::circle_with_two_flats(r(3.0), 24, r(1.0)),
            ),
            ("heart", Profile::heart(r(4.0), r(4.0), 32)),
            ("crescent", Profile::crescent(r(3.0), r(2.0), r(1.5), 24)),
            (
                "cycloidal_gear",
                Profile::cycloidal_gear(r(2.0), 12, r(1.0), r(0.0), 4),
            ),
            (
                "involute_rack",
                Profile::involute_rack(r(2.0), 4, r(20.0), r(0.0), r(0.0)),
            ),
            (
                "cycloidal_rack",
                Profile::cycloidal_rack(r(2.0), 4, r(0.0), 8),
            ),
        ];

        for (name, profile) in profiles {
            assert!(!profile.is_empty(), "{name} returned empty topology");
        }
    }
}
