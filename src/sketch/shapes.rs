//! 2D Shapes as `Profile`s

use crate::csg::CSG;
use crate::hyper_math::{
    IntoReal, Real, frac_pi_2, hangle_sin_cos, hdegrees_to_radians, hreal_abs, hreal_affine,
    hreal_atan, hreal_clamp_f64, hreal_cmp_f64, hreal_div, hreal_f64s_exactly_equal,
    hreal_from_f64, hreal_mul, hreal_pow, hreal_sqrt, hreal_sub, hreal_sum, hreal_tan,
    hxy_lerp, pi, tau,
};
use crate::sketch::Profile;
use hypercurve::{Contour2, CurveString2, LineSeg2, Point2, Segment2};
use std::borrow::Borrow;
use std::cmp::Ordering;
use std::fmt::Debug;

fn finite_profile_scalar(value: &Real) -> bool {
    hreal_from_f64(value).is_ok()
}

fn finite_profile_scalars<I, V>(values: I) -> bool
where
    I: IntoIterator<Item = V>,
    V: Borrow<Real>,
{
    values
        .into_iter()
        .all(|value| finite_profile_scalar(value.borrow()))
}

/// Sample `start + sweep * index / count` through hyperreal arithmetic.
///
/// Profile constructors still expose `f64` boundary coordinates, but all
/// parametric sampling is promoted before the final finite points are handed to
/// `hypercurve`. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>), keeping predicate-adjacent
/// construction algebra out of local primitive arithmetic.
fn hsample_angle<S: IntoReal, W: IntoReal>(
    index: usize,
    count: usize,
    start: S,
    sweep: W,
) -> Option<Real> {
    let t = hreal_div(index, count)?;
    hreal_affine(start, t, sweep)
}

/// Return a finite point on `x = rx*cos(theta), y = ry*sin(theta)`.
fn hellipse_point<RX: IntoReal, RY: IntoReal, T: IntoReal>(
    rx: RX,
    ry: RY,
    theta: T,
) -> Option<[Real; 2]> {
    let rx = rx.into_real().ok()?;
    let ry = ry.into_real().ok()?;
    let (sin_theta, cos_theta) = hangle_sin_cos(theta)?;
    Some([hreal_mul(&rx, cos_theta)?, hreal_mul(&ry, sin_theta)?])
}

fn hpolar_point<R: IntoReal, T: IntoReal>(radius: R, theta: T) -> Option<[Real; 2]> {
    let radius = radius.into_real().ok()?;
    hellipse_point(&radius, &radius, theta)
}

fn hrotate_xy<X: IntoReal, Y: IntoReal, A: IntoReal>(
    x: X,
    y: Y,
    angle: A,
) -> Option<(Real, Real)> {
    let x = x.into_real().ok()?;
    let y = y.into_real().ok()?;
    let (sin_angle, cos_angle) = hangle_sin_cos(angle)?;
    Some((
        hreal_sub(hreal_mul(&x, &cos_angle)?, hreal_mul(&y, &sin_angle)?)?,
        hreal_sum(&[hreal_mul(&x, sin_angle)?, hreal_mul(&y, cos_angle)?])?,
    ))
}

fn hellipse_samples(
    samples: usize,
    rx: Real,
    ry: Real,
    start: Real,
    sweep: Real,
) -> Option<Vec<[Real; 2]>> {
    let mut points = Vec::with_capacity(samples);
    for i in 0..samples {
        points.push(hellipse_point(
            rx.clone(),
            ry.clone(),
            hsample_angle(i, samples, start.clone(), sweep.clone())?,
        )?);
    }
    Some(points)
}

fn hcircle_samples(samples: usize, radius: Real) -> Option<Vec<[Real; 2]>> {
    hellipse_samples(samples, radius.clone(), radius, Real::zero(), tau())
}

fn hfinite_min_max(values: impl IntoIterator<Item = Real>) -> Option<(Real, Real)> {
    let mut iter = values.into_iter();
    let first = iter.next()?;
    hreal_from_f64(&first).ok()?;
    let mut min = first.clone();
    let mut max = first;
    for value in iter {
        hreal_from_f64(&value).ok()?;
        if matches!(hreal_cmp_f64(&value, &min), Ordering::Less) {
            min = value.clone();
        }
        if matches!(hreal_cmp_f64(&value, &max), Ordering::Greater) {
            max = value;
        }
    }
    Some((min, max))
}

fn hsigned_sqrt_abs(value: Real) -> Option<Real> {
    let magnitude = hreal_sqrt(hreal_abs(value.clone())?)?;
    match hreal_cmp_f64(&value, 0.0) {
        Ordering::Less => hreal_sub(0.0, magnitude),
        Ordering::Equal | Ordering::Greater => Some(magnitude),
    }
}

fn hfinite_nonzero(value: Real) -> bool {
    hreal_from_f64(&value).is_ok() && !matches!(hreal_cmp_f64(&value, 0.0), Ordering::Equal)
}

/// Compare a public profile scalar through hyperreal ordering.
///
/// Shape constructors still accept primitive `Real` parameters at the API
/// boundary, but admission decisions should not use local float ordering. This
/// helper promotes both operands and compares through Hyper's refinement path,
/// following Yap's exact geometric computation boundary model
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
fn hprofile_scalar_gt<L: IntoReal, R: IntoReal>(lhs: L, rhs: R) -> bool {
    matches!(hreal_cmp_f64(lhs, rhs), Ordering::Greater)
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

impl<M: Clone + Debug + Send + Sync> Profile<M> {
    /// Build a finite boundary sketch as a native hypercurve region.
    ///
    /// API callers still pass ordinary `Real` coordinates, but the boundary is
    /// immediately promoted into `hyperreal` control points. Keeping this
    /// conversion at the API edge follows the exact-geometric-computation model
    /// described by Yap, "Towards exact geometric computation",
    /// Computational Geometry 7(1-2), 1997, DOI: 10.1016/0925-7721(95)00040-2.
    fn polygonal_region(points: Vec<[Real; 2]>, metadata: M) -> Self {
        let Ok(contour) = Contour2::from_real_ring(&points) else {
            return Profile::empty(metadata);
        };
        Profile::from_contour(contour, metadata)
    }

    /// Creates a 2D rectangle in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width of the rectangle
    /// - `length`: the height of the rectangle
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::sketch::Profile;
    /// let sq2 = Profile::<()>::rectangle(2.0, 3.0, ());
    /// ```
    pub fn rectangle(width: Real, length: Real, metadata: M) -> Self {
        let points = [
            [Real::zero(), Real::zero()],
            [width.clone(), Real::zero()],
            [width, length.clone()],
            [Real::zero(), length],
        ];
        Self::polygon(&points, metadata)
    }

    /// Creates a 2D square in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width=length of the square
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// let sq2 = Profile::square(2.0, None);
    pub fn square(width: Real, metadata: M) -> Self {
        Self::rectangle(width.clone(), width, metadata)
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
    /// - `metadata`: Optional metadata attached to the shape
    pub fn circle(radius: Real, segments: usize, metadata: M) -> Self {
        if segments < 3 || !finite_profile_scalar(&radius) {
            return Profile::empty(metadata);
        }
        let Some(points) = hcircle_samples(segments, radius) else {
            return Profile::empty(metadata);
        };
        Self::polygonal_region(points, metadata)
    }

    /// Right triangle from (0,0) to (width,0) to (0,height).
    pub fn right_triangle(width: Real, height: Real, metadata: M) -> Self {
        let points = [
            [Real::zero(), Real::zero()],
            [width, Real::zero()],
            [Real::zero(), height],
        ];
        Self::polygon(&points, metadata)
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
    /// let poly2d = Profile::polygon_points(&pts, metadata);
    pub fn polygon(points: &[[Real; 2]], metadata: M) -> Self {
        let Ok(contour) = Contour2::from_real_ring(points) else {
            return Profile::empty(metadata);
        };
        Profile::from_contour(contour, metadata)
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
    pub fn polygon_points(points: &[Point2], metadata: M) -> Self {
        if points.len() < 3 {
            return Profile::empty(metadata);
        }

        let mut segments = Vec::with_capacity(points.len());
        for adjacent in points.windows(2) {
            let Ok(segment) = LineSeg2::try_new(adjacent[0].clone(), adjacent[1].clone())
            else {
                return Profile::empty(metadata);
            };
            segments.push(Segment2::Line(segment));
        }
        let Ok(segment) =
            LineSeg2::try_new(points[points.len() - 1].clone(), points[0].clone())
        else {
            return Profile::empty(metadata);
        };
        segments.push(Segment2::Line(segment));

        let Ok(contour) = Contour2::try_new(segments) else {
            return Profile::empty(metadata);
        };
        Profile::from_contour(contour, metadata)
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
    /// - `metadata`: Optional metadata
    pub fn ellipse(width: Real, height: Real, segments: usize, metadata: M) -> Self {
        if segments < 3 || !finite_profile_scalars([&width, &height]) {
            return Profile::empty(metadata);
        }
        let (Some(rx), Some(ry)) = (hreal_mul(0.5, &width), hreal_mul(0.5, &height)) else {
            return Profile::empty(metadata);
        };
        let Some(points) = hellipse_samples(segments, rx, ry, Real::zero(), tau()) else {
            return Profile::empty(metadata);
        };
        Self::polygonal_region(points, metadata)
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
    /// - `metadata`: Optional metadata
    pub fn regular_ngon(sides: usize, radius: Real, metadata: M) -> Self {
        if sides < 3 || !finite_profile_scalar(&radius) {
            return Profile::empty(metadata);
        }
        let Some(points) = hcircle_samples(sides, radius) else {
            return Profile::empty(metadata);
        };
        Self::polygonal_region(points, metadata)
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
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// ```ignore
    /// use csgrs::sketch::Profile;
    /// let arrow = Profile::<()>::arrow(5.0, 0.5, 2.0, 1.5, ());
    /// ```
    pub fn arrow(
        shaft_length: Real,
        shaft_width: Real,
        head_length: Real,
        head_width: Real,
        metadata: M,
    ) -> Self {
        if shaft_length <= 0.0 || shaft_width <= 0.0 || head_length <= 0.0 || head_width <= 0.0
        {
            return Profile::empty(metadata);
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

        Self::polygonal_region(points, metadata)
    }

    /// Trapezoid from (0,0) -> (bottom_width,0) -> (top_width+top_offset,height) -> (top_offset,height)
    /// Note: this is a simple shape that can represent many trapezoids or parallelograms.
    pub fn trapezoid(
        top_width: Real,
        bottom_width: Real,
        height: Real,
        top_offset: Real,
        metadata: M,
    ) -> Self {
        let points = vec![
            [Real::zero(), Real::zero()],
            [bottom_width, Real::zero()],
            [top_width + top_offset.clone(), height.clone()],
            [top_offset, height],
        ];
        Self::polygonal_region(points, metadata)
    }

    /// Star shape (typical "spiky star") with `num_points`, outer_radius, inner_radius.
    /// The star is centered at (0,0).
    pub fn star(
        num_points: usize,
        outer_radius: Real,
        inner_radius: Real,
        metadata: M,
    ) -> Self {
        if num_points < 2 || !finite_profile_scalars([&outer_radius, &inner_radius]) {
            return Profile::empty(metadata);
        }
        let mut points = Vec::with_capacity(num_points * 2);
        for i in 0..num_points {
            let Some(theta_out) = hsample_angle(i, num_points, Real::zero(), tau()) else {
                return Profile::empty(metadata);
            };
            let Some(theta_in) =
                hsample_angle((i * 2) + 1, num_points * 2, Real::zero(), tau())
            else {
                return Profile::empty(metadata);
            };
            let Some(outer_point) =
                hellipse_point(outer_radius.clone(), outer_radius.clone(), theta_out)
            else {
                return Profile::empty(metadata);
            };
            let Some(inner_point) =
                hellipse_point(inner_radius.clone(), inner_radius.clone(), theta_in)
            else {
                return Profile::empty(metadata);
            };
            points.push(outer_point);
            points.push(inner_point);
        }
        Self::polygonal_region(points, metadata)
    }

    /// Teardrop shape.  A simple approach:
    /// - a circle arc for the "round" top
    /// - it tapers down to a cusp at bottom.
    ///
    /// This is just one of many possible "teardrop" definitions.
    ///
    /// The semicircular cap is sampled through hyperreal trigonometry before
    /// the finite ring is handed to `hypercurve`, following Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    // todo: center on focus of the arc
    pub fn teardrop(width: Real, length: Real, segments: usize, metadata: M) -> Profile<M> {
        if segments < 2
            || !hprofile_scalar_positive(&width)
            || !hprofile_scalar_positive(&length)
            || !finite_profile_scalars([&width, &length])
        {
            return Profile::empty(metadata);
        }
        let Some(r) = hreal_mul(0.5, &width) else {
            return Profile::empty(metadata);
        };
        let Some(center_y) = hreal_sub(&length, &r) else {
            return Profile::empty(metadata);
        };
        let half_seg = segments / 2;

        let mut points = vec![[Real::zero(), Real::zero()]]; // Start at the tip
        for i in 0..=half_seg {
            let Some(t) = hsample_angle(i, half_seg, Real::zero(), pi()) else {
                return Profile::empty(metadata);
            };
            let Some([dx, dy]) = hellipse_point(r.clone(), r.clone(), t) else {
                return Profile::empty(metadata);
            };
            let (Some(x), Some(y)) = (hreal_sub(0.0, dx), hreal_affine(&center_y, 1.0, dy))
            else {
                return Profile::empty(metadata);
            };
            points.push([x, y]);
        }

        Self::polygonal_region(points, metadata)
    }

    /// Egg outline.  Approximate an egg shape using a parametric approach.
    /// This is only a toy approximation.  It creates a closed "egg-ish" outline around the origin.
    ///
    /// The trigonometric sampling is evaluated on `Real` before
    /// exporting the finite polygonal boundary. That keeps this parametric
    /// constructor aligned with Yap's exact-geometric-computation boundary
    /// model (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn egg(width: Real, length: Real, segments: usize, metadata: M) -> Profile<M> {
        if segments < 3 || !finite_profile_scalars([&width, &length]) {
            return Profile::empty(metadata);
        }
        let (Some(rx), Some(ry)) = (hreal_mul(0.5, &width), hreal_mul(0.5, &length)) else {
            return Profile::empty(metadata);
        };
        let mut points = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(theta) = hsample_angle(i, segments, Real::zero(), tau()) else {
                return Profile::empty(metadata);
            };
            let Some((sin_theta, cos_theta)) = hangle_sin_cos(&theta) else {
                return Profile::empty(metadata);
            };
            let Some(distort) = hreal_affine(1.0, 0.2, &cos_theta) else {
                return Profile::empty(metadata);
            };
            let Some(x) = hreal_mul(&rx, &sin_theta).and_then(|x| hreal_sub(0.0, x)) else {
                return Profile::empty(metadata);
            };
            let Some(y) = hreal_mul(&ry, &cos_theta)
                .and_then(|y| hreal_mul(y, distort))
                .and_then(|y| hreal_mul(y, 0.8))
            else {
                return Profile::empty(metadata);
            };
            points.push([x, y]);
        }

        Self::polygonal_region(points, metadata)
    }

    /// Rounded rectangle in XY plane, from (0,0) to (width,height) with radius for corners.
    /// `corner_segments` controls the smoothness of each rounded corner.
    ///
    /// Corner-radius clamping and arc sampling are evaluated through hyperreal
    /// helpers before exporting the finite boundary to hypercurve, following
    /// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
        metadata: M,
    ) -> Self {
        if !finite_profile_scalars([&width, &height, &corner_radius]) {
            return Profile::empty(metadata);
        }
        let (Some(half_width), Some(half_height)) =
            (hreal_mul(&width, 0.5), hreal_mul(&height, 0.5))
        else {
            return Profile::empty(metadata);
        };
        let radius = match hreal_cmp_f64(&corner_radius, &half_width) {
            Ordering::Greater => half_width,
            Ordering::Less | Ordering::Equal => corner_radius.clone(),
        };
        let r = match hreal_cmp_f64(&radius, &half_height) {
            Ordering::Greater => half_height,
            Ordering::Less | Ordering::Equal => radius,
        };
        if corner_segments == 0 || !hprofile_scalar_positive(&r) {
            return Profile::rectangle(width, height, metadata);
        }
        // We'll approximate each 90° corner with `corner_segments` arcs
        let mut points = Vec::with_capacity((corner_segments + 1) * 4);
        let Some(right) = hreal_sub(&width, &r) else {
            return Profile::empty(metadata);
        };
        let Some(top) = hreal_sub(&height, &r) else {
            return Profile::empty(metadata);
        };
        for (cx, cy, start_angle) in [
            (r.clone(), r.clone(), pi()),
            (right.clone(), r.clone(), 1.5 * pi()),
            (right, top.clone(), Real::zero()),
            (r.clone(), top, 0.5 * pi()),
        ] {
            for i in 0..=corner_segments {
                let Some(angle) =
                    hsample_angle(i, corner_segments, start_angle.clone(), frac_pi_2())
                else {
                    return Profile::empty(metadata);
                };
                let Some([dx, dy]) = hellipse_point(r.clone(), r.clone(), angle) else {
                    return Profile::empty(metadata);
                };
                let (Some(x), Some(y)) =
                    (hreal_affine(&cx, 1.0, dx), hreal_affine(&cy, 1.0, dy))
                else {
                    return Profile::empty(metadata);
                };
                points.push([x, y]);
            }
        }

        Self::polygonal_region(points, metadata)
    }

    /// Squircle (superellipse) centered at (0,0) with bounding box width×height.
    /// We use an exponent = 4.0 for "classic" squircle shape. `segments` controls the resolution.
    ///
    /// This is Lamé's superellipse specialized to exponent 4. The sampled
    /// signed square-root form is evaluated through hyperreal helpers before
    /// the finite ring is exported to hypercurve, following Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). See also Lamé,
    /// *Examen des différentes méthodes employées pour résoudre les problèmes
    /// de géométrie*, 1818.
    pub fn squircle(width: Real, height: Real, segments: usize, metadata: M) -> Profile<M> {
        if segments < 3 || !finite_profile_scalars([&width, &height]) {
            return Profile::empty(metadata);
        }
        let (Some(rx), Some(ry)) = (hreal_mul(0.5, &width), hreal_mul(0.5, &height)) else {
            return Profile::empty(metadata);
        };
        let mut points = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(t) = hsample_angle(i, segments, Real::zero(), tau()) else {
                return Profile::empty(metadata);
            };
            let Some((sin_t, cos_t)) = hangle_sin_cos(t) else {
                return Profile::empty(metadata);
            };
            let (Some(ct), Some(st)) = (hsigned_sqrt_abs(cos_t), hsigned_sqrt_abs(sin_t))
            else {
                return Profile::empty(metadata);
            };
            let (Some(x), Some(y)) = (hreal_mul(&rx, ct), hreal_mul(&ry, st)) else {
                return Profile::empty(metadata);
            };
            points.push([x, y]);
        }

        Self::polygonal_region(points, metadata)
    }

    /// Keyhole shape (simple version): a large circle + a rectangle "handle".
    /// This does *not* have a hole.  If you want a literal hole, you'd do difference ops.
    /// Here we do union of a circle and a rectangle.
    pub fn keyhole(
        circle_radius: Real,
        handle_width: Real,
        handle_height: Real,
        segments: usize,
        metadata: M,
    ) -> Profile<M> {
        if segments < 3 {
            return Profile::empty(metadata);
        }
        // 1) Circle
        let circle = Profile::circle(circle_radius, segments, metadata.clone());

        // 2) Rectangle (handle)
        let handle_offset = -(&handle_width * 0.5);
        let handle = Profile::rectangle(handle_width, handle_height, metadata).translate(
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
    /// * `diameter`               desired constant width (equals the distance between adjacent vertices, i.e. the polygon’s edge length)
    /// * `circle_segments`        how many segments to use for each disk
    ///
    /// For `sides == 3` this gives the canonical Reuleaux triangle; for any
    /// larger `sides` it yields the natural generalisation (odd-sided shapes
    /// retain constant width, even-sided ones do not but are still smooth).
    /// Vertex-center sampling uses hyperreal sine/cosine before composing the
    /// shape from `Profile::circle` disks, following Yap's EGC boundary split
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn reuleaux(
        sides: usize,
        diameter: Real,
        circle_segments: usize,
        metadata: M,
    ) -> Profile<M> {
        if sides < 3
            || circle_segments < 6
            || !hprofile_scalar_positive(&diameter)
            || !finite_profile_scalar(&diameter)
        {
            return Profile::empty(metadata);
        }

        // Circumradius that gives the requested *diameter* for the regular n-gon
        //            s
        //   R = -------------
        //        2 sin(π/n)
        let Some(angle) = hreal_div(pi(), sides) else {
            return Profile::empty(metadata);
        };
        let Some((sin_angle, _)) = hangle_sin_cos(angle) else {
            return Profile::empty(metadata);
        };
        let Some(denom) = hreal_mul(2.0, sin_angle) else {
            return Profile::empty(metadata);
        };
        let Some(r_circ) = hreal_div(&diameter, denom) else {
            return Profile::empty(metadata);
        };

        // Pre-compute vertex positions of the regular n-gon
        let mut verts = Vec::with_capacity(sides);
        for i in 0..sides {
            let Some(theta) = hsample_angle(i, sides, Real::zero(), tau()) else {
                return Profile::empty(metadata);
            };
            let Some([x, y]) = hellipse_point(r_circ.clone(), r_circ.clone(), theta) else {
                return Profile::empty(metadata);
            };
            verts.push((x, y));
        }

        // Build the first disk and use it as the running intersection
        let base = Profile::circle(diameter.clone(), circle_segments, metadata.clone())
            .translate(verts[0].0.clone(), verts[0].1.clone(), Real::zero());

        let shape = verts.iter().skip(1).fold(base, |acc, (x, y)| {
            let disk = Profile::circle(diameter.clone(), circle_segments, metadata.clone())
                .translate(x.clone(), y.clone(), Real::zero());
            acc.intersection(&disk)
        });

        shape.with_metadata(metadata)
    }

    /// Outer diameter = `id + 2*thickness`. This yields an annulus in the XY plane.
    /// `segments` controls how smooth the outer/inner circles are.
    ///
    /// Radius arithmetic is promoted before the annular profile is composed
    /// from hypercurve-backed circle regions, following Yap's exact-geometric
    /// computation boundary split (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn ring(id: Real, thickness: Real, segments: usize, metadata: M) -> Profile<M> {
        if id <= 0.0
            || thickness <= 0.0
            || segments < 3
            || !finite_profile_scalars([&id, &thickness])
        {
            return Profile::empty(metadata);
        }
        let Some(inner_radius) = hreal_mul(0.5, &id) else {
            return Profile::empty(metadata);
        };
        let Some(outer_radius) = hreal_affine(&inner_radius, 1.0, thickness) else {
            return Profile::empty(metadata);
        };

        let outer_circle = Profile::circle(outer_radius, segments, metadata.clone());
        let inner_circle = Profile::circle(inner_radius, segments, metadata);

        outer_circle.difference(&inner_circle)
    }

    /// Create a 2D "pie slice" (wedge) in the XY plane.
    /// - `radius`: outer radius of the slice.
    /// - `start_angle_deg`: starting angle in degrees (measured from X-axis).
    /// - `end_angle_deg`: ending angle in degrees.
    /// - `segments`: how many segments to use to approximate the arc.
    /// - `metadata`: optional user metadata for this polygon.
    pub fn pie_slice(
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
        segments: usize,
        metadata: M,
    ) -> Profile<M> {
        if segments < 1 || !finite_profile_scalars([&radius, &start_angle_deg, &end_angle_deg])
        {
            return Profile::empty(metadata);
        }

        let (Some(start_rad), Some(end_rad)) = (
            hdegrees_to_radians(start_angle_deg.clone()),
            hdegrees_to_radians(end_angle_deg),
        ) else {
            return Profile::empty(metadata);
        };
        let Some(sweep) = hreal_sub(&end_rad, &start_rad) else {
            return Profile::empty(metadata);
        };

        // Build a ring of coordinates starting at (0,0), going around the arc, and closing at (0,0).
        let mut points = Vec::with_capacity(segments + 2);
        points.push([Real::zero(), Real::zero()]);
        for i in 0..=segments {
            let Some(angle) = hsample_angle(i, segments, start_rad.clone(), sweep.clone())
            else {
                return Profile::empty(metadata);
            };
            let Some(point) = hellipse_point(radius.clone(), radius.clone(), angle) else {
                return Profile::empty(metadata);
            };
            points.push(point);
        }

        Self::polygonal_region(points, metadata)
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
        metadata: M,
    ) -> Profile<M> {
        if segments < 3
            || !finite_profile_scalars([&a, &b, &m, &n1, &n2, &n3])
            || !hfinite_nonzero(a.clone())
            || !hfinite_nonzero(b.clone())
            || !hfinite_nonzero(n1.clone())
        {
            return Profile::empty(metadata);
        }

        fn supershape_r(
            theta: Real,
            a: Real,
            b: Real,
            m: Real,
            n1: Real,
            n2: Real,
            n3: Real,
        ) -> Option<Real> {
            // r(θ) = [ |cos(mθ/4)/a|^n2 + |sin(mθ/4)/b|^n3 ]^(-1/n1)
            let t = hreal_mul(hreal_mul(m, theta)?, 0.25)?;
            let (sin_t, cos_t) = hangle_sin_cos(t)?;
            let cos_term = hreal_abs(hreal_div(cos_t, a)?)?;
            let sin_term = hreal_abs(hreal_div(sin_t, b)?)?;
            let term1 = hreal_pow(cos_term, n2)?;
            let term2 = hreal_pow(sin_term, n3)?;
            let sum = hreal_sum(&[term1, term2])?;
            let exponent = hreal_div(-1.0, n1)?;
            hreal_pow(sum, exponent)
        }

        let mut points = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(theta) = hsample_angle(i, segments, Real::zero(), tau()) else {
                return Profile::empty(metadata);
            };
            let Some(r) = supershape_r(
                theta.clone(),
                a.clone(),
                b.clone(),
                m.clone(),
                n1.clone(),
                n2.clone(),
                n3.clone(),
            ) else {
                return Profile::empty(metadata);
            };

            let Some((sin_theta, cos_theta)) = hangle_sin_cos(theta) else {
                return Profile::empty(metadata);
            };
            let (Some(x), Some(y)) = (hreal_mul(&r, cos_theta), hreal_mul(r, sin_theta))
            else {
                return Profile::empty(metadata);
            };
            points.push([x, y]);
        }

        Self::polygonal_region(points, metadata)
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
        metadata: M,
    ) -> Profile<M> {
        if segments < 3 || !finite_profile_scalars([&radius, &key_width, &key_depth]) {
            return Profile::empty(metadata);
        }
        // 1. Full circle
        let circle = Profile::circle(radius.clone(), segments, metadata.clone());

        // 2. Construct the keyway rectangle
        let Some(key_x) = hreal_sub(&radius, &key_depth) else {
            return Profile::empty(metadata);
        };
        let Some(key_y) = hreal_mul(-0.5, &key_width) else {
            return Profile::empty(metadata);
        };
        let key_rect = Profile::rectangle(key_depth, key_width, metadata.clone()).translate(
            key_x,
            key_y,
            Real::zero(),
        );

        circle.difference(&key_rect)
    }

    /// Creates a 2D "D" shape (circle with one flat chord).
    /// `radius` is the circle radius,
    /// `flat_dist` is how far from the center the flat chord is placed.
    ///
    /// Cutter dimensions and offsets are evaluated in hyperreal arithmetic
    /// before the result is composed from hypercurve regions.
    pub fn circle_with_flat(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: M,
    ) -> Profile<M> {
        if segments < 3 || !finite_profile_scalars([&radius, &flat_dist]) {
            return Profile::empty(metadata);
        }
        // 1. Full circle
        let circle = Profile::circle(radius.clone(), segments, metadata.clone());

        // 2. Build a large rectangle that cuts off everything below y = -flat_dist
        let cutter_height = hreal_from_f64(9999.0).expect("finite cutter height");
        let (Some(width), Some(neg_radius), Some(neg_height), Some(neg_flat)) = (
            hreal_mul(2.0, &radius),
            hreal_sub(0.0, &radius),
            hreal_sub(0.0, &cutter_height),
            hreal_sub(0.0, &flat_dist),
        ) else {
            return Profile::empty(metadata);
        };
        let rect_cutter = Profile::rectangle(width, cutter_height, metadata.clone())
            .translate(neg_radius, neg_height, Real::zero()) // put its bottom near "negative infinity"
            .translate(Real::zero(), neg_flat, Real::zero()); // now top edge is at y = -flat_dist

        // 3. Subtract to produce the flat chord
        circle.difference(&rect_cutter)
    }

    /// Circle with two parallel flat chords on opposing sides (e.g., "double D" shape).
    /// `radius`   => circle radius
    /// `segments` => how many segments in the circle approximation
    /// `flat_dist` => half-distance between flats measured from the center.
    ///   - chord at y=+flat_dist  and  chord at y=-flat_dist
    pub fn circle_with_two_flats(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: M,
    ) -> Profile<M> {
        if segments < 3 || !finite_profile_scalars([&radius, &flat_dist]) {
            return Profile::empty(metadata);
        }
        // 1. Full circle
        let circle = Profile::circle(radius.clone(), segments, metadata.clone());

        // 2. Large rectangle to cut the TOP (above +flat_dist)
        let cutter_height = hreal_from_f64(9999.0).expect("finite cutter height");
        let (Some(width), Some(neg_radius), Some(neg_height)) = (
            hreal_mul(2.0, &radius),
            hreal_sub(0.0, &radius),
            hreal_sub(0.0, &cutter_height),
        ) else {
            return Profile::empty(metadata);
        };
        let top_rect =
            Profile::rectangle(width.clone(), cutter_height.clone(), metadata.clone())
                // place bottom at y=flat_dist
                .translate(neg_radius.clone(), flat_dist.clone(), Real::zero());

        // 3. Large rectangle to cut the BOTTOM (below -flat_dist)
        let Some(bottom_y) = hreal_sub(&neg_height, &flat_dist) else {
            return Profile::empty(metadata);
        };
        let bottom_rect = Profile::rectangle(width, cutter_height, metadata.clone())
            // place top at y=-flat_dist => bottom extends downward
            .translate(neg_radius, bottom_y, Real::zero());

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
    pub fn bezier(control: &[[Real; 2]], segments: usize, metadata: M) -> Self {
        if control.len() < 2
            || segments < 1
            || control.iter().any(|point| !finite_profile_scalars(point))
        {
            return Profile::empty(metadata);
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
                return Profile::empty(metadata);
            };
            let Some((x, y)) = de_casteljau(control, t) else {
                return Profile::empty(metadata);
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
            return Self::polygonal_region(pts, metadata);
        }

        CurveString2::from_real_point_iter(pts)
            .map(|wire| Profile::from_wire(wire, metadata.clone()))
            .unwrap_or_else(|_| Profile::empty(metadata))
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
    pub fn bspline(
        control: &[[Real; 2]],
        p: usize,
        segments_per_span: usize,
        metadata: M,
    ) -> Self {
        if control.len() < p + 1
            || segments_per_span < 1
            || control.iter().any(|point| !finite_profile_scalars(point))
        {
            return Profile::empty(metadata);
        }

        let n = control.len() - 1;
        let m = n + p + 1; // knot count
        // open-uniform knot vector: 0,0,…,0,1,2,…,n-p-1,(n-p),…,(n-p)
        let mut knot = Vec::<Real>::with_capacity(m + 1);
        for i in 0..=m {
            if i <= p {
                knot.push(Real::zero());
            } else if i >= m - p {
                knot.push(Real::from((n - p) as u64));
            } else {
                knot.push(Real::from((i - p) as u64));
            }
        }

        // Cox-de Boor basis evaluation with hyperreal scalar operations.
        fn basis(i: usize, p: usize, u: Real, knot: &[Real]) -> Option<Real> {
            if p == 0 {
                return Some(if u >= knot[i] && u < knot[i + 1] {
                    Real::from(1_u8)
                } else {
                    Real::zero()
                });
            }
            let denom1 = hreal_sub(&knot[i + p], &knot[i])?;
            let denom2 = hreal_sub(&knot[i + p + 1], &knot[i + 1])?;
            let term1 = if hreal_f64s_exactly_equal(&denom1, 0.0) {
                Real::zero()
            } else {
                let numerator = hreal_sub(&u, &knot[i])?;
                hreal_mul(
                    hreal_div(numerator, denom1)?,
                    basis(i, p - 1, u.clone(), knot)?,
                )?
            };
            let term2 = if hreal_f64s_exactly_equal(&denom2, 0.0) {
                Real::zero()
            } else {
                let numerator = hreal_sub(&knot[i + p + 1], &u)?;
                hreal_mul(hreal_div(numerator, denom2)?, basis(i + 1, p - 1, u, knot)?)?
            };
            hreal_affine(term1, 1.0, term2)
        }

        let span_count = n - p; // #inner knot spans
        let _max_u = Real::from(span_count as u64); // parametric upper bound
        let Some(dt) = hreal_div(1.0, segments_per_span) else {
            return Profile::empty(metadata);
        };

        let mut pts = Vec::<[Real; 2]>::new();
        for span in 0..span_count {
            for s in 0..=segments_per_span {
                if span + 1 == span_count && s == segments_per_span {
                    // avoid duplicating final knot value
                    continue;
                }
                let Some(u) = hreal_affine(span, s, dt.clone()) else {
                    return Profile::empty(metadata);
                };
                let mut xs = Vec::with_capacity(control.len());
                let mut ys = Vec::with_capacity(control.len());
                for (idx, point) in control.iter().enumerate() {
                    let Some(b) = basis(idx, p, u.clone(), &knot) else {
                        return Profile::empty(metadata);
                    };
                    let (Some(x), Some(y)) =
                        (hreal_mul(&b, &point[0]), hreal_mul(b, &point[1]))
                    else {
                        return Profile::empty(metadata);
                    };
                    xs.push(x);
                    ys.push(y);
                }
                let (Some(x), Some(y)) = (hreal_sum(&xs), hreal_sum(&ys)) else {
                    return Profile::empty(metadata);
                };
                pts.push([x, y]);
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
                .map(|wire| Profile::from_wire(wire, metadata.clone()))
                .unwrap_or_else(|_| Profile::empty(metadata));
        }

        Self::polygonal_region(pts, metadata)
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
    pub fn heart(width: Real, height: Real, segments: usize, metadata: M) -> Self {
        if segments < 8 || !finite_profile_scalars([&width, &height]) {
            return Profile::empty(metadata);
        }

        // classic analytic “cardioid-style” heart
        let mut pts = Vec::with_capacity(segments);
        for i in 0..segments {
            let Some(t) = hsample_angle(i, segments, Real::zero(), tau()) else {
                return Profile::empty(metadata);
            };
            let Some((sin_t, cos_t)) = hangle_sin_cos(&t) else {
                return Profile::empty(metadata);
            };
            let Some(sin2) = hreal_mul(&sin_t, &sin_t) else {
                return Profile::empty(metadata);
            };
            let Some(sin3) = hreal_mul(sin2, &sin_t) else {
                return Profile::empty(metadata);
            };
            let Some(x) = hreal_mul(16.0, sin3) else {
                return Profile::empty(metadata);
            };

            let Some(t2) = hreal_mul(2.0, &t) else {
                return Profile::empty(metadata);
            };
            let Some(t3) = hreal_mul(3.0, &t) else {
                return Profile::empty(metadata);
            };
            let Some(t4) = hreal_mul(4.0, &t) else {
                return Profile::empty(metadata);
            };
            let Some((_, cos_2t)) = hangle_sin_cos(&t2) else {
                return Profile::empty(metadata);
            };
            let Some((_, cos_3t)) = hangle_sin_cos(&t3) else {
                return Profile::empty(metadata);
            };
            let Some((_, cos_4t)) = hangle_sin_cos(&t4) else {
                return Profile::empty(metadata);
            };
            let Some(mut y) = hreal_mul(13.0, &cos_t) else {
                return Profile::empty(metadata);
            };
            let Some(term) = hreal_mul(5.0, cos_2t) else {
                return Profile::empty(metadata);
            };
            let Some(next_y) = hreal_sub(y, term) else {
                return Profile::empty(metadata);
            };
            y = next_y;
            let Some(term) = hreal_mul(2.0, cos_3t) else {
                return Profile::empty(metadata);
            };
            let Some(next_y) = hreal_sub(y, term) else {
                return Profile::empty(metadata);
            };
            y = next_y;
            let Some(next_y) = hreal_sub(y, cos_4t) else {
                return Profile::empty(metadata);
            };
            pts.push((x, next_y));
        }

        // normalise & scale to desired bounding box ---------------------
        let Some((min_x, max_x)) = hfinite_min_max(pts.iter().map(|(x, _)| x.clone())) else {
            return Profile::empty(metadata);
        };
        let Some((min_y, max_y)) = hfinite_min_max(pts.iter().map(|(_, y)| y.clone())) else {
            return Profile::empty(metadata);
        };
        let Some(w) = hreal_sub(&max_x, &min_x) else {
            return Profile::empty(metadata);
        };
        let Some(h) = hreal_sub(&max_y, &min_y) else {
            return Profile::empty(metadata);
        };
        let Some(s_x) = hreal_div(&width, w) else {
            return Profile::empty(metadata);
        };
        let Some(s_y) = hreal_div(&height, h) else {
            return Profile::empty(metadata);
        };

        let mut points = Vec::with_capacity(pts.len());
        for (x, y) in pts {
            let Some(dx) = hreal_sub(x, &min_x) else {
                return Profile::empty(metadata);
            };
            let Some(dy) = hreal_sub(y, &min_y) else {
                return Profile::empty(metadata);
            };
            let (Some(x), Some(y)) = (hreal_mul(dx, &s_x), hreal_mul(dy, &s_y)) else {
                return Profile::empty(metadata);
            };
            points.push([x, y]);
        }

        Self::polygonal_region(points, metadata)
    }

    /// 2-D crescent obtained by subtracting a displaced smaller circle
    /// from a larger one.  
    /// `segments` controls circle smoothness.
    ///
    /// ```ignore
    /// use csgrs::sketch::Profile;
    /// let cres = Profile::<()>::crescent(2.0, 1.4, 0.8, 64, ());
    /// ```
    pub fn crescent(
        outer_r: Real,
        inner_r: Real,
        offset: Real,
        segments: usize,
        metadata: M,
    ) -> Self {
        if segments < 6
            || !finite_profile_scalars([&outer_r, &inner_r, &offset])
            || !hprofile_scalar_gt(&outer_r, &inner_r)
            || !hprofile_scalar_positive(&inner_r)
        {
            return Profile::empty(metadata);
        }

        let big = Self::circle(outer_r, segments, metadata.clone());
        let small = Self::circle(inner_r, segments, metadata.clone()).translate(
            offset,
            Real::zero(),
            Real::zero(),
        );

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
    /// - `metadata`: optional metadata
    pub fn involute_gear(
        module: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        metadata: M,
    ) -> Profile<M> {
        if teeth < 4
            || segments_per_flank < 2
            || !hprofile_scalar_positive(&module)
            || !finite_profile_scalars([&module, &pressure_angle_deg, &clearance, &backlash])
        {
            return Profile::empty(metadata);
        }

        let m = &module;
        let z = Real::from(teeth as u64);
        let Some(pressure_angle) = hdegrees_to_radians(&pressure_angle_deg) else {
            return Profile::empty(metadata);
        };

        // Standard gear dimensions. Involute sampling follows Litvin and Fuentes,
        // Gear Geometry and Applied Theory, 2nd ed., Cambridge University Press,
        // 2004, while scalar construction stays in hyperreal helpers.
        let Some(pitch_radius) = hreal_mul(0.5, m).and_then(|half_m| hreal_mul(&half_m, &z))
        else {
            return Profile::empty(metadata);
        };
        let addendum = module.clone();
        let Some(dedendum) =
            hreal_mul(1.25, m).and_then(|scaled| hreal_sum(&[scaled, clearance.clone()]))
        else {
            return Profile::empty(metadata);
        };
        let Some(outer_radius) = hreal_sum(&[pitch_radius.clone(), addendum]) else {
            return Profile::empty(metadata);
        };
        let Some((_, pressure_cos)) = hangle_sin_cos(&pressure_angle) else {
            return Profile::empty(metadata);
        };
        let Some(base_radius) = hreal_mul(&pitch_radius, pressure_cos) else {
            return Profile::empty(metadata);
        };
        let Some(raw_root_radius) = hreal_sub(&pitch_radius, &dedendum) else {
            return Profile::empty(metadata);
        };
        let Some(base_root_floor) = hreal_mul(&base_radius, 0.9) else {
            return Profile::empty(metadata);
        };
        let _root_radius = match hreal_cmp_f64(&raw_root_radius, &base_root_floor) {
            Ordering::Greater => raw_root_radius.clone(),
            _ => base_root_floor.clone(),
        };

        let Some(angular_pitch) = hreal_div(tau(), &z) else {
            return Profile::empty(metadata);
        };
        let Some(backlash_angle) = hreal_div(&backlash, &pitch_radius) else {
            return Profile::empty(metadata);
        };
        let Some(tooth_thickness_at_pitch) = hreal_div(&angular_pitch, 2.0)
            .and_then(|half_pitch| hreal_sub(&half_pitch, &backlash_angle))
        else {
            return Profile::empty(metadata);
        };
        let Some(half_tooth_angle) = hreal_div(&tooth_thickness_at_pitch, 2.0) else {
            return Profile::empty(metadata);
        };

        if !finite_profile_scalars([
            &pitch_radius,
            &outer_radius,
            &base_radius,
            &tooth_thickness_at_pitch,
            &half_tooth_angle,
        ]) || !hprofile_scalar_positive(&pitch_radius)
            || !hprofile_scalar_positive(&base_radius)
            || !hprofile_scalar_gt(&outer_radius, &base_radius)
            || !hprofile_scalar_positive(&tooth_thickness_at_pitch)
        {
            return Profile::empty(metadata);
        }

        // Helper: generate one involute flank from r1 to r2
        let generate_flank =
            |r_start: &Real, r_end: &Real, reverse: bool| -> Option<Vec<(Real, Real)>> {
                let mut pts = Vec::with_capacity(segments_per_flank + 1);
                for i in 0..=segments_per_flank {
                    let t = hreal_div(i, segments_per_flank)?;
                    let delta_r = hreal_sub(r_end, r_start)?;
                    let r = hreal_affine(r_start, &t, &delta_r)?;
                    let radius_ratio = hreal_div(&r, &base_radius)?;
                    let ratio2 = hreal_mul(&radius_ratio, &radius_ratio)?;
                    let phi2 = match hreal_cmp_f64(&ratio2, 1.0) {
                        Ordering::Less => Real::zero(),
                        _ => hreal_sub(&ratio2, 1.0)?,
                    };
                    let phi = hreal_sqrt(phi2)?;
                    let (sin_phi, cos_phi) = hangle_sin_cos(&phi)?;
                    let x_term = hreal_sum(&[cos_phi.clone(), hreal_mul(&phi, &sin_phi)?])?;
                    let y_term = hreal_sub(&sin_phi, hreal_mul(&phi, &cos_phi)?)?;
                    pts.push((
                        hreal_mul(&base_radius, x_term)?,
                        hreal_mul(&base_radius, y_term)?,
                    ));
                }
                if reverse {
                    pts.reverse();
                }
                Some(pts)
            };

        // Build one full tooth (right flank + arc at tip + left flank + root arc)
        let mut tooth_profile = Vec::new();

        // Right flank: from base to outer
        let Some(right_flank) = generate_flank(&base_radius, &outer_radius, false) else {
            return Profile::empty(metadata);
        };
        // Left flank: mirror and reverse
        let left_flank: Vec<_> = right_flank
            .iter()
            .map(|(x, y)| (x.clone(), -y.clone()))
            .rev()
            .collect();

        // Angular offset from tooth center to flank start at base circle
        let Some(phi_base) = hreal_div(&pitch_radius, &base_radius)
            .and_then(|ratio| hreal_mul(&ratio, &ratio))
            .and_then(|ratio2| hreal_sub(&ratio2, 1.0))
            .and_then(hreal_sqrt)
        else {
            return Profile::empty(metadata);
        };
        let Some(inv_phi_base) = hreal_sub(&phi_base, &pressure_angle) else {
            return Profile::empty(metadata);
        };
        let Some(offset_angle) = hreal_sum(&[inv_phi_base, half_tooth_angle.clone()]) else {
            return Profile::empty(metadata);
        };

        // Apply rotation to flanks
        for (x, y) in &right_flank {
            let Some(angle) = hreal_sub(0.0, &offset_angle) else {
                return Profile::empty(metadata);
            };
            let Some(rotated) = hrotate_xy(x, y, &angle) else {
                return Profile::empty(metadata);
            };
            tooth_profile.push(rotated);
        }
        for (x, y) in &left_flank {
            let Some(rotated) = hrotate_xy(x, y, &offset_angle) else {
                return Profile::empty(metadata);
            };
            tooth_profile.push(rotated);
        }

        // Close the tooth at the root with a small arc (optional but improves validity)
        // For simplicity, we'll just connect to root circle with straight lines or small arc.
        // But for now, connect last point to first via root radius approximation.
        // Better: add root fillet, but we'll skip for brevity.

        // Now replicate around the gear
        let mut outline = Vec::with_capacity(tooth_profile.len() * teeth + 1);
        for i in 0..teeth {
            let Some(rot) = hreal_mul(i, &angular_pitch) else {
                return Profile::empty(metadata);
            };
            for (x, y) in &tooth_profile {
                let Some((x, y)) = hrotate_xy(x, y, &rot) else {
                    return Profile::empty(metadata);
                };
                outline.push([x, y]);
            }
        }
        outline.push(outline[0].clone()); // close

        Self::polygonal_region(outline, metadata)
    }

    /// Generate an (epicyclic) cycloidal gear outline
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `teeth`: number of teeth (>= 3)
    /// - `pin_teeth`: number of teeth in the pin wheel for pairing
    /// - `clearance`: additional clearance for dedendum
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    /// - `metadata`: optional metadata
    ///
    /// * Each tooth is defined in its own local angular frame around a centre
    ///   angle φ_c.
    /// * The tooth profile is defined in polar coordinates r(φ), symmetric
    ///   about φ_c, so tips sit directly above their bases.
    ///
    /// It is not a mathematically exact epicycloid/hypocycloid construction
    /// (like Sparks/Daniels), but produces a clean, non-self-intersecting,
    /// cycloidal-looking gear that meshes reasonably with the matching
    /// cycloidal rack.
    ///
    /// Scalar construction is routed through hyperreal arithmetic before the
    /// finite outline is exported to hypercurve, following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). Gear-shape semantics
    /// remain the existing approximate cycloidal profile rather than an exact
    /// epicycloid/hypocycloid solver.
    pub fn cycloidal_gear(
        module: Real,
        teeth: usize,
        _pin_teeth: usize, // kept for API symmetry, not used in this approx
        clearance: Real,
        segments_per_flank: usize,
        metadata: M,
    ) -> Profile<M> {
        if teeth < 3
            || segments_per_flank < 2
            || !finite_profile_scalars([&module, &clearance])
            || !hprofile_scalar_positive(&module)
        {
            return Profile::empty(metadata);
        }

        let z = Real::from(teeth as u64);
        let m = &module;

        // Basic radii (same conventions as involute_gear).
        let Some(half_module) = hreal_mul(0.5, m) else {
            return Profile::empty(metadata);
        };
        let Some(pitch_radius) = hreal_mul(&half_module, &z) else {
            return Profile::empty(metadata);
        };
        let addendum = module.clone();
        let Some(scaled_module) = hreal_mul(1.25, m) else {
            return Profile::empty(metadata);
        };
        let Some(dedendum) = hreal_sum(&[scaled_module, clearance.clone()]) else {
            return Profile::empty(metadata);
        };
        let Some(outer_radius) = hreal_sum(&[pitch_radius.clone(), addendum]) else {
            return Profile::empty(metadata);
        };
        let Some(raw_root_radius) = hreal_sub(&pitch_radius, &dedendum) else {
            return Profile::empty(metadata);
        };
        if !hprofile_scalar_positive(&raw_root_radius) {
            return Profile::empty(metadata);
        }
        let root_radius = raw_root_radius;

        // Angular pitch between tooth centres.
        let Some(ang_pitch) = hreal_div(tau(), &z) else {
            return Profile::empty(metadata);
        };

        // We give each tooth half the pitch for material, half for space.
        // So tooth half-angle at the pitch circle is:
        let Some(half_tooth_angle) = hreal_mul(&ang_pitch, 0.25) else {
            return Profile::empty(metadata);
        };

        if !finite_profile_scalars([
            &pitch_radius,
            &outer_radius,
            &root_radius,
            &ang_pitch,
            &half_tooth_angle,
        ]) || !hprofile_scalar_positive(&pitch_radius)
            || !hprofile_scalar_gt(&outer_radius, &pitch_radius)
            || !hprofile_scalar_positive(&half_tooth_angle)
        {
            return Profile::empty(metadata);
        }

        // Total angular span per tooth profile (from left gap to right gap):
        let Some(_span_per_tooth) = hreal_mul(2.0, &half_tooth_angle) else {
            return Profile::empty(metadata);
        };

        // Helper: "cycloidal-ish" bump shape for the addendum.
        //
        // φ_offset ∈ [-half_tooth_angle, +half_tooth_angle] (angle from tooth centre).
        // We map that to u ∈ [-1, +1] and use a smooth bump with zero slope at
        // the edges and tip.
        fn addendum_profile(
            pitch_radius: &Real,
            outer_radius: &Real,
            half_tooth_angle: &Real,
            phi_offset: &Real,
        ) -> Option<Real> {
            // Normalised offset from tooth centre: u ∈ [-1, 1]
            let u = hreal_clamp_f64(hreal_div(phi_offset, half_tooth_angle)?, -1.0, 1.0)?;

            // Strictly convex bump: 0 at |u| = 1, 1 at u = 0
            // p controls how “fat” the tip is; p = 2 is a good starting point.
            let p = 2.0;
            let bump = hreal_sub(1.0, hreal_pow(hreal_abs(u)?, p)?)?;

            hreal_sum(&[
                pitch_radius.clone(),
                hreal_mul(bump, hreal_sub(outer_radius, pitch_radius)?)?,
            ])
        }

        // Precompute how many angular samples per tooth we want.
        // Two flanks per tooth, so total samples per tooth:
        let samples_per_tooth = 2 * segments_per_flank + 2; // +2 for including endpoints

        let mut outline: Vec<[Real; 2]> = Vec::with_capacity(samples_per_tooth * teeth + 1);

        for i in 0..teeth {
            let Some(tooth_center_angle) = hreal_mul(i, &ang_pitch) else {
                return Profile::empty(metadata);
            };

            // 1. ADDENDUM (tip region) – go CCW from left flank to right flank.
            //
            // We sample φ over the tooth-material region:
            //   φ ∈ [φ_c - half_tooth_angle, φ_c + half_tooth_angle]
            for j in 0..=segments_per_flank {
                let Some(left_tooth_angle) = hreal_sub(&tooth_center_angle, &half_tooth_angle)
                else {
                    return Profile::empty(metadata);
                };
                let Some(addendum_sweep) = hreal_mul(2.0, &half_tooth_angle) else {
                    return Profile::empty(metadata);
                };
                let Some(phi) =
                    hsample_angle(j, segments_per_flank, &left_tooth_angle, &addendum_sweep)
                else {
                    return Profile::empty(metadata);
                };
                let Some(phi_offset) = hreal_sub(&phi, &tooth_center_angle) else {
                    return Profile::empty(metadata);
                };

                let Some(r) = addendum_profile(
                    &pitch_radius,
                    &outer_radius,
                    &half_tooth_angle,
                    &phi_offset,
                ) else {
                    return Profile::empty(metadata);
                };

                let Some(point) = hpolar_point(&r, &phi) else {
                    return Profile::empty(metadata);
                };
                outline.push(point);
            }

            // 2. ROOT REGION – simple circular arc on root_radius between
            //    this tooth's right gap and the next tooth's left gap.
            //
            // Right gap angle for this tooth:
            let Some(right_gap_angle) =
                hreal_sum(&[tooth_center_angle.clone(), half_tooth_angle.clone()])
            else {
                return Profile::empty(metadata);
            };
            // Left gap angle for next tooth (wrap around at 2π):
            let Some(next_center_angle) =
                hreal_sum(&[tooth_center_angle.clone(), ang_pitch.clone()])
            else {
                return Profile::empty(metadata);
            };
            let Some(next_left_gap_angle) = hreal_sub(&next_center_angle, &half_tooth_angle)
            else {
                return Profile::empty(metadata);
            };

            for j in 0..=segments_per_flank {
                let Some(root_sweep) = hreal_sub(&next_left_gap_angle, &right_gap_angle)
                else {
                    return Profile::empty(metadata);
                };
                let Some(phi) =
                    hsample_angle(j, segments_per_flank, &right_gap_angle, &root_sweep)
                else {
                    return Profile::empty(metadata);
                };
                let Some(point) = hpolar_point(&root_radius, &phi) else {
                    return Profile::empty(metadata);
                };
                outline.push(point);
            }
        }

        // Close the polygon
        outline.push(outline[0].clone());

        Self::polygonal_region(outline, metadata)
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
    /// - `metadata`: optional metadata
    ///
    /// Rack dimensions and flank intersections are evaluated through
    /// hyperreal helpers before the finite outline is exported to hypercurve,
    /// following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). For involute rack
    /// geometry conventions see Litvin and Fuentes, *Gear Geometry and Applied
    /// Theory*, 2nd ed., Cambridge University Press, 2004.
    pub fn involute_rack(
        module_: Real,
        num_teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        metadata: M,
    ) -> Profile<M> {
        if num_teeth < 1
            || !finite_profile_scalars([&module_, &pressure_angle_deg, &clearance, &backlash])
            || !hprofile_scalar_positive(&module_)
        {
            return Profile::empty(metadata);
        }
        let m = &module_;
        let Some(p) = hreal_mul(pi(), m) else {
            return Profile::empty(metadata);
        }; // linear pitch
        let addendum = module_.clone();
        let Some(scaled_module) = hreal_mul(1.25, m) else {
            return Profile::empty(metadata);
        };
        let Some(dedendum) = hreal_sum(&[scaled_module, clearance.clone()]) else {
            return Profile::empty(metadata);
        };
        let tip_y = addendum.clone();
        let Some(root_y) = hreal_sub(0.0, &dedendum) else {
            return Profile::empty(metadata);
        };
        // Tooth thickness at pitch‑line (centre) minus backlash.
        let Some(half_pitch) = hreal_div(&p, 2.0) else {
            return Profile::empty(metadata);
        };
        let Some(t) = hreal_sub(&half_pitch, &backlash) else {
            return Profile::empty(metadata);
        };
        let Some(half_t) = hreal_div(&t, 2.0) else {
            return Profile::empty(metadata);
        };
        // For a rack, the involute flank is a straight line at pressure angle
        let Some(alpha) = hdegrees_to_radians(&pressure_angle_deg) else {
            return Profile::empty(metadata);
        };
        let Some(tan_alpha) = hreal_tan(&alpha) else {
            return Profile::empty(metadata);
        };

        if !finite_profile_scalars([
            &p, &addendum, &dedendum, &tip_y, &root_y, &t, &half_t, &tan_alpha,
        ]) || !hreal_abs(&tan_alpha).is_some_and(hfinite_nonzero)
            || !hprofile_scalar_positive(&t)
        {
            return Profile::empty(metadata);
        }

        // Build the complete rack profile as a single closed polygon
        let mut outline = Vec::<[Real; 2]>::new();

        // Start at the bottom left of the first tooth
        let Some(tooth_height) = hreal_sub(&tip_y, &root_y) else {
            return Profile::empty(metadata);
        };
        let Some(root_slant) = hreal_div(&tooth_height, &tan_alpha) else {
            return Profile::empty(metadata);
        };
        let Some(negative_half_t) = hreal_sub(0.0, &half_t) else {
            return Profile::empty(metadata);
        };
        let Some(first_x) = hreal_sub(&negative_half_t, &root_slant) else {
            return Profile::empty(metadata);
        };
        outline.push([first_x.clone(), root_y.clone()]);

        // Build each tooth
        for i in 0..num_teeth {
            let Some(tooth_center) = hreal_mul(i, &p) else {
                return Profile::empty(metadata);
            };
            let Some(left_pitch) = hreal_sub(&tooth_center, &half_t) else {
                return Profile::empty(metadata);
            };
            let Some(right_pitch) = hreal_sum(&[tooth_center.clone(), half_t.clone()]) else {
                return Profile::empty(metadata);
            };
            let Some(tip_slant) = hreal_div(&tip_y, &tan_alpha) else {
                return Profile::empty(metadata);
            };
            let Some(left_tip) = hreal_sub(&left_pitch, &tip_slant) else {
                return Profile::empty(metadata);
            };
            let Some(right_tip) = hreal_sum(&[right_pitch.clone(), tip_slant.clone()]) else {
                return Profile::empty(metadata);
            };

            // Left flank (from root to tip)
            outline.push([left_pitch.clone(), Real::zero()]);
            outline.push([left_tip, tip_y.clone()]);

            // Top of tooth
            outline.push([right_tip, tip_y.clone()]);

            // Right flank (from tip to root)
            outline.push([right_pitch, Real::zero()]);

            // Bottom right (root)
            if i < num_teeth - 1 {
                let Some(next_center) = hreal_mul(i + 1, &p) else {
                    return Profile::empty(metadata);
                };
                let Some(next_left_pitch) = hreal_sub(&next_center, &half_t) else {
                    return Profile::empty(metadata);
                };
                let Some(next_root_left) = hreal_sub(&next_left_pitch, &root_slant) else {
                    return Profile::empty(metadata);
                };
                outline.push([next_root_left, root_y.clone()]);
            }
        }

        // Close the polygon by connecting back to the start
        // Add the bottom right corner
        let Some(last_tooth_center) = hreal_mul(num_teeth - 1, &p) else {
            return Profile::empty(metadata);
        };
        let Some(last_right_pitch) = hreal_sum(&[last_tooth_center, half_t.clone()]) else {
            return Profile::empty(metadata);
        };
        let Some(last_root_right) = hreal_sum(&[last_right_pitch, root_slant]) else {
            return Profile::empty(metadata);
        };
        outline.push([last_root_right, root_y.clone()]);

        // Now close the polygon by going back to the start
        outline.push([first_x, root_y]);

        Self::polygonal_region(outline, metadata)
    }

    /// Generate a linear cycloidal rack profile.
    /// The cycloidal rack is generated by rolling a circle of radius `r_p` along the
    /// rack's pitch‑line. The flanks become a trochoid; for practical purposes we
    /// approximate with the classic curtate cycloid equations.
    ///
    /// The cycloidal cap samples are evaluated through `Real`
    /// before the finite outline is composed as a `hypercurve` region. This
    /// keeps the tooth-pitch and trigonometric construction on the
    /// exact-aware side of the boundary, following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). For the gear-geometry
    /// background see Litvin and Fuentes, *Gear Geometry and Applied Theory*,
    /// 2nd ed., Cambridge University Press, 2004.
    ///
    /// # Parameters
    /// - `module_`: gear module
    /// - `num_teeth`: number of teeth along the rack
    /// - `generating_radius`: radius of the generating circle (usually = module_/2)
    /// - `clearance`: additional clearance for dedendum
    /// - `segments_per_flank`: tessellation resolution per tooth flank
    /// - `metadata`: optional metadata
    pub fn cycloidal_rack(
        module_: Real,
        num_teeth: usize,
        generating_radius: Real, // usually = module_/2
        clearance: Real,
        segments_per_flank: usize,
        metadata: M,
    ) -> Profile<M> {
        if num_teeth < 1
            || segments_per_flank < 4
            || !hprofile_scalar_positive(&module_)
            || !hprofile_scalar_positive(&generating_radius)
            || !finite_profile_scalars([&module_, &generating_radius, &clearance])
        {
            return Profile::empty(metadata);
        }
        let Some(p) = hreal_mul(pi(), &module_) else {
            return Profile::empty(metadata);
        };
        let addendum = module_.clone();
        let Some(dedendum) = hreal_affine(&clearance, 1.25, &module_) else {
            return Profile::empty(metadata);
        };
        let _tip_y = addendum.clone();
        let Some(root_y) = hreal_sub(0.0, &dedendum) else {
            return Profile::empty(metadata);
        };

        if !finite_profile_scalars([&p, &addendum, &dedendum, &root_y]) {
            return Profile::empty(metadata);
        }

        // Build one monotone rack boundary rather than repeating a closed
        // tooth island. The sampled cycloidal cap is intentionally composed
        // into a single hypercurve region, avoiding self-intersections at
        // tooth joins while preserving Profile as the 2-D shape carrier.
        let Some(left_edge) = hreal_mul(-0.5, &p) else {
            return Profile::empty(metadata);
        };
        let Some(right_edge) = hreal_mul(Real::from(num_teeth as u64) - 0.5, &p) else {
            return Profile::empty(metadata);
        };
        let Some(half_addendum) = hreal_mul(&addendum, 0.5) else {
            return Profile::empty(metadata);
        };
        let mut top = Vec::<[Real; 2]>::with_capacity(num_teeth * segments_per_flank + 1);
        for k in 0..num_teeth {
            let Some(tooth_left) = hreal_mul(Real::from(k as u64) - 0.5, &p) else {
                return Profile::empty(metadata);
            };
            for j in 0..=segments_per_flank {
                if k > 0 && j == 0 {
                    continue;
                }
                let Some(u) = hreal_div(j, segments_per_flank) else {
                    return Profile::empty(metadata);
                };
                let Some(theta) = hreal_mul(tau(), &u) else {
                    return Profile::empty(metadata);
                };
                let Some(x) = hreal_affine(&tooth_left, &u, &p) else {
                    return Profile::empty(metadata);
                };
                let Some((_, cos_theta)) = hangle_sin_cos(&theta) else {
                    return Profile::empty(metadata);
                };
                let Some(one_minus_cos) = hreal_sub(1.0, &cos_theta) else {
                    return Profile::empty(metadata);
                };
                let Some(y) = hreal_mul(&half_addendum, one_minus_cos) else {
                    return Profile::empty(metadata);
                };
                top.push([x, y]);
            }
        }

        let mut outline = Vec::<[Real; 2]>::with_capacity(top.len() + 3);
        outline.push([left_edge.clone(), root_y.clone()]);
        outline.push([right_edge, root_y.clone()]);
        for point in top.into_iter().rev() {
            outline.push(point);
        }
        outline.push([left_edge, root_y]);

        Self::polygonal_region(outline, metadata)
    }

    /// Generate a NACA 4-digit airfoil (e.g. "2412", "0015").
    ///
    /// ## Parameters
    /// - `max_camber`: max camber %, the first digit
    /// - `camber_position`: camber position, the second digit
    /// - `thickness`: thickness %, the last two digits
    /// - `chord`: physical chord length you want (same units as the rest of your model)
    /// - `samples`: number of points per surface (≥ 10 is required; NP total = 2 × samples + 1)
    /// - `metadata`: optional metadata
    ///
    /// The function returns a single closed polygon lying in the *XY* plane with its
    /// leading edge at the origin and the chord running along +X.
    ///
    /// The 4-digit thickness and camber equations trace back to Jacobs, Ward,
    /// and Pinkerton, "The characteristics of 78 related airfoil sections from
    /// tests in the variable-density wind tunnel", NACA Report 460, 1933.
    /// The sampled boundary is evaluated through hyperreal helpers before
    /// export to `hypercurve::Region2`, following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn airfoil_naca4(
        max_camber: Real,
        camber_position: Real,
        thickness: Real,
        chord: Real,
        samples: usize,
        metadata: M,
    ) -> Profile<M> {
        if samples < 10
            || !finite_profile_scalars([&max_camber, &camber_position, &thickness, &chord])
            || !hprofile_scalar_positive(&chord)
            || !matches!(hreal_cmp_f64(&thickness, 0.0), Ordering::Greater)
        {
            return Profile::empty(metadata);
        }
        let Some(max_camber_percentage) = hreal_div(&max_camber, 100.0) else {
            return Profile::empty(metadata);
        };
        let Some(camber_pos) = hreal_div(&camber_position, 10.0) else {
            return Profile::empty(metadata);
        };
        let cambered = !matches!(hreal_cmp_f64(&max_camber_percentage, 0.0), Ordering::Equal);
        if cambered
            && (!matches!(hreal_cmp_f64(&camber_pos, 0.0), Ordering::Greater)
                || !matches!(hreal_cmp_f64(&camber_pos, 1.0), Ordering::Less))
        {
            return Profile::empty(metadata);
        }

        // thickness half-profile
        let half_profile = |x: &Real| -> Option<Real> {
            let x2 = hreal_mul(x, x)?;
            let x3 = hreal_mul(&x2, x)?;
            let x4 = hreal_mul(&x3, x)?;
            let terms = [
                hreal_mul(0.2969, hreal_sqrt(x)?)?,
                hreal_mul(-0.1260, x)?,
                hreal_mul(-0.3516, &x2)?,
                hreal_mul(0.2843, &x3)?,
                hreal_mul(-0.1015, x4)?,
            ];
            let thickness_scale = hreal_div(hreal_mul(5.0, &thickness)?, 100.0)?;
            hreal_mul(thickness_scale, hreal_sum(&terms)?)
        };

        // mean-camber line & slope
        let camber = |x: &Real| -> Option<(Real, Real)> {
            if !cambered {
                return Some((Real::zero(), Real::zero()));
            }

            if matches!(hreal_cmp_f64(x, &camber_pos), Ordering::Less) {
                let camber_pos2 = hreal_mul(&camber_pos, &camber_pos)?;
                let scale = hreal_div(&max_camber_percentage, &camber_pos2)?;
                let two_p_x = hreal_mul(hreal_mul(2.0, &camber_pos)?, x)?;
                let x2 = hreal_mul(x, x)?;
                let yc = hreal_mul(&scale, hreal_sub(&two_p_x, &x2)?)?;
                let dy_scale =
                    hreal_div(hreal_mul(2.0, &max_camber_percentage)?, &camber_pos2)?;
                let dy = hreal_mul(dy_scale, hreal_sub(&camber_pos, x)?)?;
                Some((yc, dy))
            } else {
                let one_minus_p = hreal_sub(1.0, &camber_pos)?;
                let one_minus_p2 = hreal_mul(&one_minus_p, &one_minus_p)?;
                let scale = hreal_div(&max_camber_percentage, &one_minus_p2)?;
                let one_minus_2p = hreal_sub(1.0, hreal_mul(2.0, &camber_pos)?)?;
                let two_p_x = hreal_mul(hreal_mul(2.0, &camber_pos)?, x)?;
                let x2 = hreal_mul(x, x)?;
                let yc = hreal_mul(
                    &scale,
                    hreal_sum(&[one_minus_2p, two_p_x, hreal_sub(0.0, &x2)?])?,
                )?;
                let dy_scale =
                    hreal_div(hreal_mul(2.0, &max_camber_percentage)?, &one_minus_p2)?;
                let dy = hreal_mul(dy_scale, hreal_sub(&camber_pos, x)?)?;
                Some((yc, dy))
            }
        };

        // sample upper & lower surfaces
        let mut points: Vec<[Real; 2]> = Vec::with_capacity(2 * samples);

        // leading-edge → trailing-edge (upper)
        for i in 0..=samples {
            let Some(xc) = hreal_div(i, samples) else {
                return Profile::empty(metadata);
            };
            let Some(x) = hreal_mul(&xc, &chord) else {
                return Profile::empty(metadata);
            };
            let Some(t) = half_profile(&xc) else {
                return Profile::empty(metadata);
            };
            let Some((yc_val, dy)) = camber(&xc) else {
                return Profile::empty(metadata);
            };
            let Some(theta) = hreal_atan(&dy) else {
                return Profile::empty(metadata);
            };
            let Some((sin_theta, cos_theta)) = hangle_sin_cos(&theta) else {
                return Profile::empty(metadata);
            };

            let Some(t_sin) = hreal_mul(&t, &sin_theta) else {
                return Profile::empty(metadata);
            };
            let Some(t_cos) = hreal_mul(&t, &cos_theta) else {
                return Profile::empty(metadata);
            };
            let Some(xu) = hreal_sub(&x, &t_sin) else {
                return Profile::empty(metadata);
            };
            let Some(upper_yc) = hreal_sum(&[yc_val, t_cos]) else {
                return Profile::empty(metadata);
            };
            let Some(yu) = hreal_mul(&chord, upper_yc) else {
                return Profile::empty(metadata);
            };
            points.push([xu, yu]);
        }

        // trailing-edge → leading-edge (lower)
        for i in (1..samples).rev() {
            let Some(xc) = hreal_div(i, samples) else {
                return Profile::empty(metadata);
            };
            let Some(x) = hreal_mul(&xc, &chord) else {
                return Profile::empty(metadata);
            };
            let Some(t) = half_profile(&xc) else {
                return Profile::empty(metadata);
            };
            let Some((yc_val, dy)) = camber(&xc) else {
                return Profile::empty(metadata);
            };
            let Some(theta) = hreal_atan(&dy) else {
                return Profile::empty(metadata);
            };
            let Some((sin_theta, cos_theta)) = hangle_sin_cos(&theta) else {
                return Profile::empty(metadata);
            };

            let Some(t_sin) = hreal_mul(&t, &sin_theta) else {
                return Profile::empty(metadata);
            };
            let Some(t_cos) = hreal_mul(&t, &cos_theta) else {
                return Profile::empty(metadata);
            };
            let Some(xl) = hreal_sum(&[x, t_sin]) else {
                return Profile::empty(metadata);
            };
            let Some(lower_yc) = hreal_sub(&yc_val, &t_cos) else {
                return Profile::empty(metadata);
            };
            let Some(yl) = hreal_mul(&chord, lower_yc) else {
                return Profile::empty(metadata);
            };
            points.push([xl, yl]);
        }

        Self::polygonal_region(points, metadata)
    }

    /// Build a Hilbert-curve path that fills this sketch.
    /// - `order`: recursion order (number of points ≈ 4^order).
    /// - `padding`: optional inset from the bounding-box edges (same units as the sketch).
    ///   Returns a new `Profile` containing only the inside segments as native wires.
    ///
    /// Bounds are taken from the native hypercurve region/wire topology rather
    /// than from any finite compatibility cache. That keeps Hilbert
    /// path construction on the same exact-object side of the API boundary as
    /// the containment tests it uses; see Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    pub fn hilbert_curve(&self, order: usize, padding: Real) -> Profile<M> {
        if order == 0 {
            return Profile::empty(self.metadata.clone());
        }
        let bounds = self.native_xy_bounds();
        let Some((min_x, min_y, max_x, max_y)) = bounds else {
            return Profile::empty(self.metadata.clone());
        };

        // Bounding box and usable region (with padding).
        let Some(raw_w) = hreal_sub(&max_x, &min_x) else {
            return Profile::empty(self.metadata.clone());
        };
        let Some(raw_h) = hreal_sub(&max_y, &min_y) else {
            return Profile::empty(self.metadata.clone());
        };
        if !hprofile_scalar_positive(&raw_w) || !hprofile_scalar_positive(&raw_h) {
            return Profile::empty(self.metadata.clone());
        }
        let w = raw_w;
        let h = raw_h;
        let Some(ox) = hreal_sum(&[min_x, padding.clone()]) else {
            return Profile::empty(self.metadata.clone());
        };
        let Some(oy) = hreal_sum(&[min_y, padding.clone()]) else {
            return Profile::empty(self.metadata.clone());
        };
        let Some(double_padding) = hreal_mul(2.0, &padding) else {
            return Profile::empty(self.metadata.clone());
        };
        let Some(raw_sx) = hreal_sub(&w, &double_padding) else {
            return Profile::empty(self.metadata.clone());
        };
        let Some(raw_sy) = hreal_sub(&h, &double_padding) else {
            return Profile::empty(self.metadata.clone());
        };
        if !hprofile_scalar_positive(&raw_sx) || !hprofile_scalar_positive(&raw_sy) {
            return Profile::empty(self.metadata.clone());
        }
        let sx = raw_sx;
        let sy = raw_sy;

        // Generate normalized Hilbert points in [0,1]^2, then scale/translate.
        let pts_norm = hilbert_points(order);
        let mut pts: Vec<(Real, Real)> = Vec::with_capacity(pts_norm.len());
        for (u, v) in pts_norm {
            let Some(x) = hreal_affine(&ox, &u, &sx) else {
                return Profile::empty(self.metadata.clone());
            };
            let Some(y) = hreal_affine(&oy, &v, &sy) else {
                return Profile::empty(self.metadata.clone());
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
            let keep = self.contains_xy(mid_x, mid_y).unwrap_or(true);

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
        Profile::from_wires(wires, self.metadata.clone())
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
    fn bezier_closure_uses_exact_hyperreal_endpoint_identity() {
        let nearly_closed = Profile::<()>::bezier(
            &[
                [r(0.0), r(0.0)],
                [r(0.25), r(1.0)],
                [r(0.75), r(1.0)],
                [tolerance() * r(0.25), r(0.0)],
            ],
            8,
            (),
        );
        assert_eq!(nearly_closed.material_contour_count(), 0);
        assert_eq!(nearly_closed.wires().len(), 1);

        let exactly_closed = Profile::<()>::bezier(
            &[
                [r(0.0), r(0.0)],
                [r(0.25), r(1.0)],
                [r(0.75), r(1.0)],
                [r(0.0), r(0.0)],
            ],
            8,
            (),
        );
        assert_eq!(exactly_closed.material_contour_count(), 1);
        assert!(exactly_closed.wires().is_empty());
    }
}
