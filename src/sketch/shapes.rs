//! 2D Shapes as `Profile`s

use crate::csg::CSG;
use crate::float_types::{
    FRAC_PI_2, PI, Real, TAU, hangle_sin_cos, hdegrees_to_radians, hreal_affine, hreal_div,
    hreal_from_f64, hreal_mul, hreal_sub, tolerance,
};
use crate::sketch::Profile;
use hypercurve::{Contour2, CurveString2, LineSeg2, Point2, Segment2};
use std::fmt::Debug;

fn finite_profile_scalar(value: Real) -> bool {
    hreal_from_f64(value).is_ok()
}

fn finite_profile_scalars(values: &[Real]) -> bool {
    values.iter().all(|&value| finite_profile_scalar(value))
}

/// Sample `start + sweep * index / count` through hyperreal arithmetic.
///
/// Profile constructors still expose `f64` boundary coordinates, but all
/// parametric sampling is promoted before the final finite points are handed to
/// `hypercurve`. This follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7(1-2), 1997
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>), keeping predicate-adjacent
/// construction algebra out of local primitive arithmetic.
fn hsample_angle(index: usize, count: usize, start: Real, sweep: Real) -> Option<Real> {
    let t = hreal_div(index as Real, count as Real)?;
    hreal_affine(start, t, sweep)
}

/// Return a finite point on `x = rx*cos(theta), y = ry*sin(theta)`.
fn hellipse_point(rx: Real, ry: Real, theta: Real) -> Option<[Real; 2]> {
    let (sin_theta, cos_theta) = hangle_sin_cos(theta)?;
    Some([hreal_mul(rx, cos_theta)?, hreal_mul(ry, sin_theta)?])
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
            rx,
            ry,
            hsample_angle(i, samples, start, sweep)?,
        )?);
    }
    Some(points)
}

fn hcircle_samples(samples: usize, radius: Real) -> Option<Vec<[Real; 2]>> {
    hellipse_samples(samples, radius, radius, 0.0, TAU)
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
        let Ok(contour) = Contour2::from_finite_ring(&points) else {
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
    /// ```
    /// use csgrs::sketch::Profile;
    /// let sq2 = Profile::<()>::rectangle(2.0, 3.0, ());
    /// ```
    pub fn rectangle(width: Real, length: Real, metadata: M) -> Self {
        let points = [[0.0, 0.0], [width, 0.0], [width, length], [0.0, length]];
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
        Self::rectangle(width, width, metadata)
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
    /// ```
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
    /// - Uses TAU (2π) constant for better floating-point precision
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
        if segments < 3 || !finite_profile_scalar(radius) {
            return Profile::empty(metadata);
        }
        let Some(points) = hcircle_samples(segments, radius) else {
            return Profile::empty(metadata);
        };
        Self::polygonal_region(points, metadata)
    }

    /// Right triangle from (0,0) to (width,0) to (0,height).
    pub fn right_triangle(width: Real, height: Real, metadata: M) -> Self {
        let points = [[0.0, 0.0], [width, 0.0], [0.0, height]];
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
        let Ok(contour) = Contour2::from_finite_ring(points) else {
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
        if segments < 3 || !finite_profile_scalars(&[width, height]) {
            return Profile::empty(metadata);
        }
        let (Some(rx), Some(ry)) = (hreal_mul(0.5, width), hreal_mul(0.5, height)) else {
            return Profile::empty(metadata);
        };
        let Some(points) = hellipse_samples(segments, rx, ry, 0.0, TAU) else {
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
    /// - Uses TAU for precise angular calculations
    /// - Explicit closure for geometric validity
    /// - Minimum n = 3 to avoid degenerate cases
    ///
    /// # Parameters
    /// - `sides`: Number of polygon edges (≥ 3)
    /// - `radius`: Circumscribed circle radius
    /// - `metadata`: Optional metadata
    pub fn regular_ngon(sides: usize, radius: Real, metadata: M) -> Self {
        if sides < 3 || !finite_profile_scalar(radius) {
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
    /// ```
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

        let points = vec![
            [0.0, half_shaft_width],           // Top-left of shaft
            [shaft_length, half_shaft_width],  // Top-right of shaft
            [shaft_length, half_head_width],   // Top-right of head base
            [shaft_length + head_length, 0.0], // Tip of arrow
            [shaft_length, -half_head_width],  // Bottom-right of head base
            [shaft_length, -half_shaft_width], // Bottom-right of shaft
            [0.0, -half_shaft_width],          // Bottom-left of shaft
            [0.0, half_shaft_width],           // Back to top-left to close
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
            [0.0, 0.0],
            [bottom_width, 0.0],
            [top_width + top_offset, height],
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
        if num_points < 2 || !finite_profile_scalars(&[outer_radius, inner_radius]) {
            return Profile::empty(metadata);
        }
        let mut points = Vec::with_capacity(num_points * 2);
        for i in 0..num_points {
            let Some(theta_out) = hsample_angle(i, num_points, 0.0, TAU) else {
                return Profile::empty(metadata);
            };
            let Some(theta_in) = hsample_angle((i * 2) + 1, num_points * 2, 0.0, TAU) else {
                return Profile::empty(metadata);
            };
            let Some(outer_point) = hellipse_point(outer_radius, outer_radius, theta_out)
            else {
                return Profile::empty(metadata);
            };
            let Some(inner_point) = hellipse_point(inner_radius, inner_radius, theta_in)
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
    // todo: center on focus of the arc
    pub fn teardrop(width: Real, length: Real, segments: usize, metadata: M) -> Profile<M> {
        if segments < 2 || width < tolerance() || length < tolerance() {
            return Profile::empty(metadata);
        }
        let r = 0.5 * width;
        let center_y = length - r;
        let half_seg = segments / 2;

        let mut points = vec![[0.0, 0.0]]; // Start at the tip
        points.extend((0..=half_seg).map(|i| {
            let t = PI * (i as Real / half_seg as Real); // Corrected angle for semi-circle
            let x = -r * t.cos();
            let y = center_y + r * t.sin();
            [x, y]
        }));

        Self::polygonal_region(points, metadata)
    }

    /// Egg outline.  Approximate an egg shape using a parametric approach.
    /// This is only a toy approximation.  It creates a closed "egg-ish" outline around the origin.
    pub fn egg(width: Real, length: Real, segments: usize, metadata: M) -> Profile<M> {
        if segments < 3 {
            return Profile::empty(metadata);
        }
        let rx = 0.5 * width;
        let ry = 0.5 * length;
        let points = (0..segments)
            .map(|i| {
                let theta = TAU * (i as Real) / (segments as Real);
                // toy distortion approach
                let distort = 1.0 + 0.2 * theta.cos();
                let x = rx * theta.sin();
                let y = ry * theta.cos() * distort * 0.8;
                [-x, y] // mirrored
            })
            .collect();

        Self::polygonal_region(points, metadata)
    }

    /// Rounded rectangle in XY plane, from (0,0) to (width,height) with radius for corners.
    /// `corner_segments` controls the smoothness of each rounded corner.
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
        metadata: M,
    ) -> Self {
        if !finite_profile_scalars(&[width, height, corner_radius]) {
            return Profile::empty(metadata);
        }
        let (Some(half_width), Some(half_height)) =
            (hreal_mul(width, 0.5), hreal_mul(height, 0.5))
        else {
            return Profile::empty(metadata);
        };
        let r = corner_radius.min(half_width).min(half_height);
        if corner_segments == 0 || r <= tolerance() {
            return Profile::rectangle(width, height, metadata);
        }
        // We'll approximate each 90° corner with `corner_segments` arcs
        let mut points = Vec::with_capacity((corner_segments + 1) * 4);
        let Some(right) = hreal_sub(width, r) else {
            return Profile::empty(metadata);
        };
        let Some(top) = hreal_sub(height, r) else {
            return Profile::empty(metadata);
        };
        for (cx, cy, start_angle) in [
            (r, r, PI),
            (right, r, 1.5 * PI),
            (right, top, 0.0),
            (r, top, 0.5 * PI),
        ] {
            for i in 0..=corner_segments {
                let Some(angle) = hsample_angle(i, corner_segments, start_angle, FRAC_PI_2)
                else {
                    return Profile::empty(metadata);
                };
                let Some([dx, dy]) = hellipse_point(r, r, angle) else {
                    return Profile::empty(metadata);
                };
                let (Some(x), Some(y)) =
                    (hreal_affine(cx, 1.0, dx), hreal_affine(cy, 1.0, dy))
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
    pub fn squircle(width: Real, height: Real, segments: usize, metadata: M) -> Profile<M> {
        if segments < 3 {
            return Profile::empty(metadata);
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let m = 4.0;
        let points = (0..segments)
            .map(|i| {
                let t = TAU * (i as Real) / (segments as Real);
                let ct = t.cos().abs().powf(2.0 / m) * t.cos().signum();
                let st = t.sin().abs().powf(2.0 / m) * t.sin().signum();
                [rx * ct, ry * st]
            })
            .collect();

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
        let handle = Profile::rectangle(handle_width, handle_height, metadata).translate(
            -handle_width * 0.5,
            0.0,
            0.0,
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
    pub fn reuleaux(
        sides: usize,
        diameter: Real,
        circle_segments: usize,
        metadata: M,
    ) -> Profile<M> {
        if sides < 3 || circle_segments < 6 || diameter <= tolerance() {
            return Profile::empty(metadata);
        }

        // Circumradius that gives the requested *diameter* for the regular n-gon
        //            s
        //   R = -------------
        //        2 sin(π/n)
        let r_circ = diameter / (2.0 * (PI / sides as Real).sin());

        // Pre-compute vertex positions of the regular n-gon
        let verts: Vec<(Real, Real)> = (0..sides)
            .map(|i| {
                let theta = TAU * (i as Real) / (sides as Real);
                (r_circ * theta.cos(), r_circ * theta.sin())
            })
            .collect();

        // Build the first disk and use it as the running intersection
        let base = Profile::circle(diameter, circle_segments, metadata.clone())
            .translate(verts[0].0, verts[0].1, 0.0);

        let shape = verts.iter().skip(1).fold(base, |acc, &(x, y)| {
            let disk = Profile::circle(diameter, circle_segments, metadata.clone())
                .translate(x, y, 0.0);
            acc.intersection(&disk)
        });

        shape.with_metadata(metadata)
    }

    /// Outer diameter = `id + 2*thickness`. This yields an annulus in the XY plane.
    /// `segments` controls how smooth the outer/inner circles are.
    pub fn ring(id: Real, thickness: Real, segments: usize, metadata: M) -> Profile<M> {
        if id <= 0.0 || thickness <= 0.0 || segments < 3 {
            return Profile::empty(metadata);
        }
        let inner_radius = 0.5 * id;
        let outer_radius = inner_radius + thickness;

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
        if segments < 1 || !finite_profile_scalars(&[radius, start_angle_deg, end_angle_deg]) {
            return Profile::empty(metadata);
        }

        let (Some(start_rad), Some(end_rad)) = (
            hdegrees_to_radians(start_angle_deg),
            hdegrees_to_radians(end_angle_deg),
        ) else {
            return Profile::empty(metadata);
        };
        let Some(sweep) = hreal_sub(end_rad, start_rad) else {
            return Profile::empty(metadata);
        };

        // Build a ring of coordinates starting at (0,0), going around the arc, and closing at (0,0).
        let mut points = Vec::with_capacity(segments + 2);
        points.push([0.0, 0.0]);
        for i in 0..=segments {
            let Some(angle) = hsample_angle(i, segments, start_rad, sweep) else {
                return Profile::empty(metadata);
            };
            let Some(point) = hellipse_point(radius, radius, angle) else {
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
        if segments < 3 {
            return Profile::empty(metadata);
        }

        // The typical superformula radius function
        fn supershape_r(
            theta: Real,
            a: Real,
            b: Real,
            m: Real,
            n1: Real,
            n2: Real,
            n3: Real,
        ) -> Real {
            // r(θ) = [ |cos(mθ/4)/a|^n2 + |sin(mθ/4)/b|^n3 ]^(-1/n1)
            let t = m * theta * 0.25;
            let cos_t = t.cos().abs();
            let sin_t = t.sin().abs();
            let term1 = (cos_t / a).powf(n2);
            let term2 = (sin_t / b).powf(n3);
            (term1 + term2).powf(-1.0 / n1)
        }

        let mut points = Vec::with_capacity(segments);
        for i in 0..segments {
            let frac = i as Real / (segments as Real);
            let theta = TAU * frac;
            let r = supershape_r(theta, a, b, m, n1, n2, n3);

            let x = r * theta.cos();
            let y = r * theta.sin();
            points.push([x, y]);
        }

        Self::polygonal_region(points, metadata)
    }

    /// Creates a 2D circle with a rectangular keyway slot cut out on the +X side.
    pub fn circle_with_keyway(
        radius: Real,
        segments: usize,
        key_width: Real,
        key_depth: Real,
        metadata: M,
    ) -> Profile<M> {
        // 1. Full circle
        let circle = Profile::circle(radius, segments, metadata.clone());

        // 2. Construct the keyway rectangle
        let key_rect = Profile::rectangle(key_depth, key_width, metadata.clone()).translate(
            radius - key_depth,
            -key_width * 0.5,
            0.0,
        );

        circle.difference(&key_rect)
    }

    /// Creates a 2D "D" shape (circle with one flat chord).
    /// `radius` is the circle radius,
    /// `flat_dist` is how far from the center the flat chord is placed.
    pub fn circle_with_flat(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: M,
    ) -> Profile<M> {
        // 1. Full circle
        let circle = Profile::circle(radius, segments, metadata.clone());

        // 2. Build a large rectangle that cuts off everything below y = -flat_dist
        let cutter_height = 9999.0; // some large number
        let rect_cutter = Profile::rectangle(2.0 * radius, cutter_height, metadata.clone())
            .translate(-radius, -cutter_height, 0.0) // put its bottom near "negative infinity"
            .translate(0.0, -flat_dist, 0.0); // now top edge is at y = -flat_dist

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
        // 1. Full circle
        let circle = Profile::circle(radius, segments, metadata.clone());

        // 2. Large rectangle to cut the TOP (above +flat_dist)
        let cutter_height = 9999.0;
        let top_rect = Profile::rectangle(2.0 * radius, cutter_height, metadata.clone())
            // place bottom at y=flat_dist
            .translate(-radius, flat_dist, 0.0);

        // 3. Large rectangle to cut the BOTTOM (below -flat_dist)
        let bottom_rect = Profile::rectangle(2.0 * radius, cutter_height, metadata.clone())
            // place top at y=-flat_dist => bottom extends downward
            .translate(-radius, -cutter_height - flat_dist, 0.0);

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
        if control.len() < 2 || segments < 1 {
            return Profile::empty(metadata);
        }

        /// Evaluates a Bézier curve at a given parameter `t` using de Casteljau's algorithm.
        fn de_casteljau(control: &[[Real; 2]], t: Real) -> (Real, Real) {
            let mut points = control.to_vec();
            let n = points.len();

            for k in 1..n {
                for i in 0..(n - k) {
                    points[i][0] = (1.0 - t) * points[i][0] + t * points[i + 1][0];
                    points[i][1] = (1.0 - t) * points[i][1] + t * points[i + 1][1];
                }
            }
            (points[0][0], points[0][1])
        }

        let pts: Vec<[Real; 2]> = (0..=segments)
            .map(|i| {
                let t = i as Real / segments as Real;
                let (x, y) = de_casteljau(control, t);
                [x, y]
            })
            .collect();

        let is_closed = {
            let first = pts[0];
            let last = pts[segments];
            (first[0] - last[0]).abs() < tolerance()
                && (first[1] - last[1]).abs() < tolerance()
        };

        if is_closed {
            return Self::polygonal_region(pts, metadata);
        }

        CurveString2::from_finite_point_iter(pts)
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
        if control.len() < p + 1 || segments_per_span < 1 {
            return Profile::empty(metadata);
        }

        let n = control.len() - 1;
        let m = n + p + 1; // knot count
        // open-uniform knot vector: 0,0,…,0,1,2,…,n-p-1,(n-p),…,(n-p)
        let mut knot = Vec::<Real>::with_capacity(m + 1);
        for i in 0..=m {
            if i <= p {
                knot.push(0.0);
            } else if i >= m - p {
                knot.push((n - p) as Real);
            } else {
                knot.push((i - p) as Real);
            }
        }

        // Cox-de Boor basis evaluation
        fn basis(i: usize, p: usize, u: Real, knot: &[Real]) -> Real {
            if p == 0 {
                return if u >= knot[i] && u < knot[i + 1] {
                    1.0
                } else {
                    0.0
                };
            }
            let denom1 = knot[i + p] - knot[i];
            let denom2 = knot[i + p + 1] - knot[i + 1];
            let term1 = if denom1.abs() < tolerance() {
                0.0
            } else {
                (u - knot[i]) / denom1 * basis(i, p - 1, u, knot)
            };
            let term2 = if denom2.abs() < tolerance() {
                0.0
            } else {
                (knot[i + p + 1] - u) / denom2 * basis(i + 1, p - 1, u, knot)
            };
            term1 + term2
        }

        let span_count = n - p; // #inner knot spans
        let _max_u = span_count as Real; // parametric upper bound
        let dt = 1.0 / segments_per_span as Real; // step in local span coords

        let mut pts = Vec::<[Real; 2]>::new();
        for span in 0..=span_count {
            for s in 0..=segments_per_span {
                if span == span_count && s == segments_per_span {
                    // avoid duplicating final knot value
                    continue;
                }
                let u = span as Real + s as Real * dt; // global param
                let mut x = 0.0;
                let mut y = 0.0;
                for (idx, &[px, py]) in control.iter().enumerate() {
                    let b = basis(idx, p, u, &knot);
                    x += b * px;
                    y += b * py;
                }
                pts.push([x, y]);
            }
        }

        let first = *pts.first().unwrap();
        let last = *pts.last().unwrap();
        let closed = (first[0] - last[0]).abs() < tolerance()
            && (first[1] - last[1]).abs() < tolerance();
        if !closed {
            return CurveString2::from_finite_point_iter(pts)
                .map(|wire| Profile::from_wire(wire, metadata.clone()))
                .unwrap_or_else(|_| Profile::empty(metadata));
        }

        Self::polygonal_region(pts, metadata)
    }

    /// 2-D heart outline (closed polygon) sized to `width` × `height`.
    ///
    /// `segments` controls smoothness (≥ 8 recommended).
    pub fn heart(width: Real, height: Real, segments: usize, metadata: M) -> Self {
        if segments < 8 {
            return Profile::empty(metadata);
        }

        let step = TAU / segments as Real;

        // classic analytic “cardioid-style” heart
        let pts: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let t = i as Real * step;
                let x = 16.0 * (t.sin().powi(3));
                let y = 13.0 * t.cos()
                    - 5.0 * (2.0 * t).cos()
                    - 2.0 * (3.0 * t).cos()
                    - (4.0 * t).cos();
                (x, y)
            })
            .collect();

        // normalise & scale to desired bounding box ---------------------
        let (min_x, max_x) = pts.iter().fold((Real::MAX, -Real::MAX), |(lo, hi), &(x, _)| {
            (lo.min(x), hi.max(x))
        });
        let (min_y, max_y) = pts.iter().fold((Real::MAX, -Real::MAX), |(lo, hi), &(_, y)| {
            (lo.min(y), hi.max(y))
        });
        let s_x = width / (max_x - min_x);
        let s_y = height / (max_y - min_y);

        let points = pts
            .into_iter()
            .map(|(x, y)| [(x - min_x) * s_x, (y - min_y) * s_y])
            .collect();

        Self::polygonal_region(points, metadata)
    }

    /// 2-D crescent obtained by subtracting a displaced smaller circle
    /// from a larger one.  
    /// `segments` controls circle smoothness.
    ///
    /// ```
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
        if outer_r <= inner_r + tolerance() || segments < 6 {
            return Profile::empty(metadata);
        }

        let big = Self::circle(outer_r, segments, metadata.clone());
        let small =
            Self::circle(inner_r, segments, metadata.clone()).translate(offset, 0.0, 0.0);

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
            || module <= tolerance()
            || !finite_profile_scalars(&[module, pressure_angle_deg, clearance, backlash])
        {
            return Profile::empty(metadata);
        }

        let m = module;
        let z = teeth as Real;
        let pressure_angle = pressure_angle_deg.to_radians();

        // Standard gear dimensions
        let pitch_radius = 0.5 * m * z;
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let outer_radius = pitch_radius + addendum;
        let base_radius = pitch_radius * pressure_angle.cos();
        let _root_radius = (pitch_radius - dedendum).max(base_radius * 0.9); // avoid < base

        let angular_pitch = TAU / z;
        let tooth_thickness_at_pitch = angular_pitch / 2.0 - backlash / pitch_radius;
        let half_tooth_angle = tooth_thickness_at_pitch / 2.0;

        if !finite_profile_scalars(&[
            pitch_radius,
            outer_radius,
            base_radius,
            tooth_thickness_at_pitch,
            half_tooth_angle,
        ]) || pitch_radius <= tolerance()
            || base_radius <= tolerance()
            || outer_radius <= base_radius
            || tooth_thickness_at_pitch <= tolerance()
        {
            return Profile::empty(metadata);
        }

        // Helper: generate one involute flank from r1 to r2
        let generate_flank =
            |r_start: Real, r_end: Real, reverse: bool| -> Vec<(Real, Real)> {
                let mut pts = Vec::with_capacity(segments_per_flank + 1);
                for i in 0..=segments_per_flank {
                    let t = i as Real / segments_per_flank as Real;
                    let r = r_start + t * (r_end - r_start);
                    let phi = ((r / base_radius).powi(2) - 1.0).max(0.0).sqrt(); // involute angle
                    let (x, y) = (
                        base_radius * (phi.cos() + phi * phi.sin()),
                        base_radius * (phi.sin() - phi * phi.cos()),
                    );
                    pts.push((x, y));
                }
                if reverse {
                    pts.reverse();
                }
                pts
            };

        // Build one full tooth (right flank + arc at tip + left flank + root arc)
        let mut tooth_profile = Vec::new();

        // Right flank: from base to outer
        let right_flank = generate_flank(base_radius, outer_radius, false);
        // Left flank: mirror and reverse
        let left_flank: Vec<_> = right_flank.iter().map(|&(x, y)| (x, -y)).rev().collect();

        // Rotate flanks to align with tooth center
        let rotate = |x: Real, y: Real, angle: Real| -> (Real, Real) {
            let c = angle.cos();
            let s = angle.sin();
            (x * c - y * s, x * s + y * c)
        };

        // Angular offset from tooth center to flank start at base circle
        let phi_base = ((pitch_radius / base_radius).powi(2) - 1.0).sqrt();
        let inv_phi_base = phi_base - pressure_angle; // involute function value
        let offset_angle = inv_phi_base + half_tooth_angle;

        // Apply rotation to flanks
        for &(x, y) in &right_flank {
            tooth_profile.push(rotate(x, y, -offset_angle));
        }
        for &(x, y) in &left_flank {
            tooth_profile.push(rotate(x, y, offset_angle));
        }

        // Close the tooth at the root with a small arc (optional but improves validity)
        // For simplicity, we'll just connect to root circle with straight lines or small arc.
        // But for now, connect last point to first via root radius approximation.
        // Better: add root fillet, but we'll skip for brevity.

        // Now replicate around the gear
        let mut outline = Vec::with_capacity(tooth_profile.len() * teeth + 1);
        for i in 0..teeth {
            let rot = i as Real * angular_pitch;
            let c = rot.cos();
            let s = rot.sin();
            for &(x, y) in &tooth_profile {
                outline.push([x * c - y * s, x * s + y * c]);
            }
        }
        outline.push(outline[0]); // close

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
            || module <= tolerance()
            || !finite_profile_scalars(&[module, clearance])
        {
            return Profile::empty(metadata);
        }

        let z = teeth as Real;
        let m = module;

        // Basic radii (same conventions as involute_gear).
        let pitch_radius = 0.5 * m * z;
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let outer_radius = pitch_radius + addendum;
        let root_radius = (pitch_radius - dedendum).max(tolerance());

        // Angular pitch between tooth centres.
        let ang_pitch = TAU / z;

        // We give each tooth half the pitch for material, half for space.
        // So tooth half-angle at the pitch circle is:
        let half_tooth_angle = ang_pitch * 0.25;

        if !finite_profile_scalars(&[
            pitch_radius,
            outer_radius,
            root_radius,
            ang_pitch,
            half_tooth_angle,
        ]) || pitch_radius <= tolerance()
            || outer_radius <= pitch_radius
            || half_tooth_angle <= tolerance()
        {
            return Profile::empty(metadata);
        }

        // Total angular span per tooth profile (from left gap to right gap):
        let _span_per_tooth = 2.0 * half_tooth_angle;

        // Helper: "cycloidal-ish" bump shape for the addendum.
        //
        // φ_offset ∈ [-half_tooth_angle, +half_tooth_angle] (angle from tooth centre).
        // We map that to u ∈ [-1, +1] and use a smooth bump with zero slope at
        // the edges and tip.
        fn addendum_profile(
            pitch_radius: Real,
            outer_radius: Real,
            half_tooth_angle: Real,
            phi_offset: Real,
        ) -> Real {
            // Normalised offset from tooth centre: u ∈ [-1, 1]
            let u = (phi_offset / half_tooth_angle).clamp(-1.0, 1.0);

            // Strictly convex bump: 0 at |u| = 1, 1 at u = 0
            // p controls how “fat” the tip is; p = 2 is a good starting point.
            let p = 2.0;
            let bump = 1.0 - u.abs().powf(p);

            pitch_radius + bump * (outer_radius - pitch_radius)
        }

        // Precompute how many angular samples per tooth we want.
        // Two flanks per tooth, so total samples per tooth:
        let samples_per_tooth = 2 * segments_per_flank + 2; // +2 for including endpoints

        let mut outline: Vec<[Real; 2]> = Vec::with_capacity(samples_per_tooth * teeth + 1);

        for i in 0..teeth {
            let tooth_center_angle = (i as Real) * ang_pitch;

            // 1. ADDENDUM (tip region) – go CCW from left flank to right flank.
            //
            // We sample φ over the tooth-material region:
            //   φ ∈ [φ_c - half_tooth_angle, φ_c + half_tooth_angle]
            for j in 0..=segments_per_flank {
                let t = j as Real / (segments_per_flank as Real);
                let phi = tooth_center_angle - half_tooth_angle + t * (2.0 * half_tooth_angle);
                let phi_offset = phi - tooth_center_angle;

                let r =
                    addendum_profile(pitch_radius, outer_radius, half_tooth_angle, phi_offset);

                outline.push([r * phi.cos(), r * phi.sin()]);
            }

            // 2. ROOT REGION – simple circular arc on root_radius between
            //    this tooth's right gap and the next tooth's left gap.
            //
            // Right gap angle for this tooth:
            let right_gap_angle = tooth_center_angle + half_tooth_angle;
            // Left gap angle for next tooth (wrap around at 2π):
            let next_center_angle = tooth_center_angle + ang_pitch;
            let next_left_gap_angle = next_center_angle - half_tooth_angle;

            for j in 0..=segments_per_flank {
                let t = j as Real / (segments_per_flank as Real);
                let phi = right_gap_angle + t * (next_left_gap_angle - right_gap_angle);
                outline.push([root_radius * phi.cos(), root_radius * phi.sin()]);
            }
        }

        // Close the polygon
        outline.push(outline[0]);

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
    pub fn involute_rack(
        module_: Real,
        num_teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        metadata: M,
    ) -> Profile<M> {
        if num_teeth < 1
            || module_ <= tolerance()
            || !finite_profile_scalars(&[module_, pressure_angle_deg, clearance, backlash])
        {
            return Profile::empty(metadata);
        }
        let m = module_;
        let p = PI * m; // linear pitch
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let tip_y = addendum;
        let root_y = -dedendum;
        // Tooth thickness at pitch‑line (centre) minus backlash.
        let t = p / 2.0 - backlash;
        let half_t = t / 2.0;
        // For a rack, the involute flank is a straight line at pressure angle
        let alpha = pressure_angle_deg.to_radians();
        let tan_alpha = alpha.tan();

        if !finite_profile_scalars(&[
            p, addendum, dedendum, tip_y, root_y, t, half_t, tan_alpha,
        ]) || tan_alpha.abs() <= tolerance()
            || t <= tolerance()
        {
            return Profile::empty(metadata);
        }

        // Build the complete rack profile as a single closed polygon
        let mut outline = Vec::<[Real; 2]>::new();

        // Start at the bottom left of the first tooth
        let first_x = -half_t - (tip_y - root_y) / tan_alpha;
        outline.push([first_x, root_y]);

        // Build each tooth
        for i in 0..num_teeth {
            let tooth_center = (i as Real) * p;
            let left_pitch = tooth_center - half_t;
            let right_pitch = tooth_center + half_t;
            let left_tip = left_pitch - (tip_y) / tan_alpha;
            let right_tip = right_pitch + (tip_y) / tan_alpha;

            // Left flank (from root to tip)
            outline.push([left_pitch, 0.0]);
            outline.push([left_tip, tip_y]);

            // Top of tooth
            outline.push([right_tip, tip_y]);

            // Right flank (from tip to root)
            outline.push([right_pitch, 0.0]);

            // Bottom right (root)
            if i < num_teeth - 1 {
                let next_left_pitch = (i as Real + 1.0) * p - half_t;
                let next_root_left = next_left_pitch - (tip_y - root_y) / tan_alpha;
                outline.push([next_root_left, root_y]);
            }
        }

        // Close the polygon by connecting back to the start
        // Add the bottom right corner
        let last_tooth_center = ((num_teeth - 1) as Real) * p;
        let last_right_pitch = last_tooth_center + half_t;
        let last_root_right = last_right_pitch + (tip_y - root_y) / tan_alpha;
        outline.push([last_root_right, root_y]);

        // Now close the polygon by going back to the start
        outline.push([first_x, root_y]);

        Self::polygonal_region(outline, metadata)
    }

    /// Generate a linear cycloidal rack profile.
    /// The cycloidal rack is generated by rolling a circle of radius `r_p` along the
    /// rack's pitch‑line. The flanks become a trochoid; for practical purposes we
    /// approximate with the classic curtate cycloid equations.
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
            || module_ <= tolerance()
            || generating_radius <= tolerance()
            || !finite_profile_scalars(&[module_, generating_radius, clearance])
        {
            return Profile::empty(metadata);
        }
        let m = module_;
        let p = PI * m;
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let _tip_y = addendum;
        let root_y = -dedendum;

        if !finite_profile_scalars(&[p, addendum, dedendum, root_y]) {
            return Profile::empty(metadata);
        }

        // Build one monotone rack boundary rather than repeating a closed
        // tooth island. The sampled cycloidal cap is intentionally composed
        // into a single hypercurve region, avoiding self-intersections at
        // tooth joins while preserving Profile as the 2-D shape carrier.
        let left_edge = -0.5 * p;
        let right_edge = (num_teeth as Real - 0.5) * p;
        let mut top = Vec::<[Real; 2]>::with_capacity(num_teeth * segments_per_flank + 1);
        for k in 0..num_teeth {
            let tooth_left = (k as Real - 0.5) * p;
            for j in 0..=segments_per_flank {
                if k > 0 && j == 0 {
                    continue;
                }
                let u = j as Real / segments_per_flank as Real;
                let theta = TAU * u;
                let x = tooth_left + p * u;
                let y = addendum * 0.5 * (1.0 - theta.cos());
                top.push([x, y]);
            }
        }

        let mut outline = Vec::<[Real; 2]>::with_capacity(top.len() + 3);
        outline.push([left_edge, root_y]);
        outline.push([right_edge, root_y]);
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
    /// The sampled boundary is promoted directly into `hypercurve::Region2`.
    pub fn airfoil_naca4(
        max_camber: Real,
        camber_position: Real,
        thickness: Real,
        chord: Real,
        samples: usize,
        metadata: M,
    ) -> Profile<M> {
        let max_camber_percentage = max_camber / 100.0;
        let camber_pos = camber_position / 10.0;

        // thickness half-profile
        let half_profile = |x: Real| -> Real {
            5.0 * thickness / 100.0
                * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x
                    - 0.1015 * x * x * x * x)
        };

        // mean-camber line & slope
        let camber = |x: Real| -> (Real, Real) {
            if x < camber_pos {
                let yc = max_camber_percentage / (camber_pos * camber_pos)
                    * (2.0 * camber_pos * x - x * x);
                let dy =
                    2.0 * max_camber_percentage / (camber_pos * camber_pos) * (camber_pos - x);
                (yc, dy)
            } else {
                let yc = max_camber_percentage / ((1.0 - camber_pos).powi(2))
                    * ((1.0 - 2.0 * camber_pos) + 2.0 * camber_pos * x - x * x);
                let dy = 2.0 * max_camber_percentage / ((1.0 - camber_pos).powi(2))
                    * (camber_pos - x);
                (yc, dy)
            }
        };

        // sample upper & lower surfaces
        let n = samples as Real;
        let mut points: Vec<[Real; 2]> = Vec::with_capacity(2 * samples);

        // leading-edge → trailing-edge (upper)
        for i in 0..=samples {
            let xc = i as Real / n; // 0–1
            let x = xc * chord; // physical
            let t = half_profile(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();

            let xu = x - t * theta.sin();
            let yu = chord * (yc_val + t * theta.cos());
            points.push([xu, yu]);
        }

        // trailing-edge → leading-edge (lower)
        for i in (1..samples).rev() {
            let xc = i as Real / n;
            let x = xc * chord;
            let t = half_profile(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();

            let xl = x + t * theta.sin();
            let yl = chord * (yc_val - t * theta.cos());
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
        let w = (max_x - min_x).max(tolerance());
        let h = (max_y - min_y).max(tolerance());
        let ox = min_x + padding;
        let oy = min_y + padding;
        let sx = (w - 2.0 * padding).max(tolerance());
        let sy = (h - 2.0 * padding).max(tolerance());

        // Generate normalized Hilbert points in [0,1]^2, then scale/translate.
        let pts_norm = hilbert_points(order);
        let pts: Vec<(Real, Real)> = pts_norm
            .into_iter()
            .map(|(u, v)| (ox + u * sx, oy + v * sy))
            .collect();

        let mut runs: Vec<Vec<(Real, Real)>> = Vec::new();
        let mut run: Vec<(Real, Real)> = Vec::new();

        for w in pts.windows(2) {
            let a = w[0];
            let b = w[1];
            let mid_x = (a.0 + b.0) * 0.5;
            let mid_y = (a.1 + b.1) * 0.5;
            let keep = self.contains_xy(mid_x, mid_y).unwrap_or(true);

            if keep {
                if run.is_empty() {
                    run.push(a);
                }
                run.push(b);
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
                CurveString2::from_finite_point_iter(run.into_iter().map(|(x, y)| [x, y])).ok()
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
            out.push((x0 + (xi + yi) * 0.5, y0 + (xj + yj) * 0.5));
        } else {
            let (xi2, xj2) = (xi * 0.5, xj * 0.5);
            let (yi2, yj2) = (yi * 0.5, yj * 0.5);
            recur(out, x0, y0, yi2, yj2, xi2, xj2, n - 1);
            recur(out, x0 + xi2, y0 + xj2, xi2, xj2, yi2, yj2, n - 1);
            recur(out, x0 + xi2 + yi2, y0 + xj2 + yj2, xi2, xj2, yi2, yj2, n - 1);
            recur(
                out,
                x0 + xi2 + yi,
                y0 + xj2 + yj,
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
    recur(&mut pts, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, order);
    pts
}
