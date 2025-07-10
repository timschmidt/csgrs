//! 2D Shapes as `Sketch`s

use crate::float_types::{EPSILON, FRAC_PI_2, PI, Real, TAU};
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{
    Geometry, GeometryCollection, LineString, Orient, Polygon as GeoPolygon, coord,
    line_string, orient::Direction,
};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
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
    /// let sq2 = Sketch::rectangle(2.0, 3.0, None);
    /// ```
    pub fn rectangle(width: Real, length: Real, metadata: Option<S>) -> Self {
        // In geo, a Polygon is basically (outer: LineString, Vec<LineString> for holes).
        let outer = line_string![
            (x: 0.0,     y: 0.0),
            (x: width,   y: 0.0),
            (x: width,   y: length),
            (x: 0.0,     y: length),
            (x: 0.0,     y: 0.0),  // close explicitly
        ];
        let polygon_2d = GeoPolygon::new(outer, vec![]);

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Creates a 2D square in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width=length of the square
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// let sq2 = Sketch::square(2.0, None);
    pub fn square(width: Real, metadata: Option<S>) -> Self {
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
    pub fn circle(radius: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return Sketch::new();
        }
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = 2.0 * PI * (i as Real) / (segments as Real);
                (radius * theta.cos(), radius * theta.sin())
            })
            .collect();
        // close it
        coords.push((coords[0].0, coords[0].1));
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Right triangle from (0,0) to (width,0) to (0,height).
    pub fn right_triangle(width: Real, height: Real, metadata: Option<S>) -> Self {
        let line_string = LineString::new(vec![
            coord! {x: 0.0, y: 0.0},
            coord! {x: width, y: 0.0},
            coord! {x: 0.0, y: height},
        ]);
        let polygon = GeoPolygon::new(line_string, vec![]);
        Sketch::from_geo(GeometryCollection(vec![Geometry::Polygon(polygon)]), metadata)
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
    /// let poly2d = Sketch::polygon(&pts, metadata);
    pub fn polygon(points: &[[Real; 2]], metadata: Option<S>) -> Self {
        if points.len() < 3 {
            return Sketch::new();
        }
        let mut coords: Vec<(Real, Real)> = points.iter().map(|p| (p[0], p[1])).collect();
        // close
        if coords[0] != *coords.last().unwrap() {
            coords.push(coords[0]);
        }
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
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
    pub fn ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return Sketch::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = TAU * (i as Real) / (segments as Real);
                (rx * theta.cos(), ry * theta.sin())
            })
            .collect();
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
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
    pub fn regular_ngon(sides: usize, radius: Real, metadata: Option<S>) -> Self {
        if sides < 3 {
            return Sketch::new();
        }
        let mut coords: Vec<(Real, Real)> = (0..sides)
            .map(|i| {
                let theta = TAU * (i as Real) / (sides as Real);
                (radius * theta.cos(), radius * theta.sin())
            })
            .collect();
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Trapezoid from (0,0) -> (bottom_width,0) -> (top_width+top_offset,height) -> (top_offset,height)
    /// Note: this is a simple shape that can represent many trapezoids or parallelograms.
    pub fn trapezoid(
        top_width: Real,
        bottom_width: Real,
        height: Real,
        top_offset: Real,
        metadata: Option<S>,
    ) -> Self {
        let coords = vec![
            (0.0, 0.0),
            (bottom_width, 0.0),
            (top_width + top_offset, height),
            (top_offset, height),
            (0.0, 0.0), // close
        ];
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Star shape (typical "spiky star") with `num_points`, outer_radius, inner_radius.
    /// The star is centered at (0,0).
    pub fn star(
        num_points: usize,
        outer_radius: Real,
        inner_radius: Real,
        metadata: Option<S>,
    ) -> Self {
        if num_points < 2 {
            return Sketch::new();
        }
        let step = TAU / (num_points as Real);
        let mut coords: Vec<(Real, Real)> = (0..num_points)
            .flat_map(|i| {
                let theta_out = i as Real * step;
                let outer_point =
                    (outer_radius * theta_out.cos(), outer_radius * theta_out.sin());

                let theta_in = theta_out + 0.5 * step;
                let inner_point =
                    (inner_radius * theta_in.cos(), inner_radius * theta_in.sin());

                [outer_point, inner_point]
            })
            .collect();
        // close
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Teardrop shape.  A simple approach:
    /// - a circle arc for the "round" top
    /// - it tapers down to a cusp at bottom.
    ///
    /// This is just one of many possible "teardrop" definitions.
    // todo: center on focus of the arc
    pub fn teardrop(
        width: Real,
        length: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> Sketch<S> {
        if segments < 2 || width < EPSILON || length < EPSILON {
            return Sketch::new();
        }
        let r = 0.5 * width;
        let center_y = length - r;
        let half_seg = segments / 2;

        let mut coords = vec![(0.0, 0.0)]; // Start at the tip
        coords.extend((0..=half_seg).map(|i| {
            let t = PI * (i as Real / half_seg as Real); // Corrected angle for semi-circle
            let x = -r * t.cos();
            let y = center_y + r * t.sin();
            (x, y)
        }));
        coords.push((0.0, 0.0)); // Close path to the tip

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Egg outline.  Approximate an egg shape using a parametric approach.
    /// This is only a toy approximation.  It creates a closed "egg-ish" outline around the origin.
    pub fn egg(width: Real, length: Real, segments: usize, metadata: Option<S>) -> Sketch<S> {
        if segments < 3 {
            return Sketch::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * length;
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let theta = TAU * (i as Real) / (segments as Real);
                // toy distortion approach
                let distort = 1.0 + 0.2 * theta.cos();
                let x = rx * theta.sin();
                let y = ry * theta.cos() * distort * 0.8;
                (-x, y) // mirrored
            })
            .collect();
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Rounded rectangle in XY plane, from (0,0) to (width,height) with radius for corners.
    /// `corner_segments` controls the smoothness of each rounded corner.
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
        metadata: Option<S>,
    ) -> Self {
        let r = corner_radius.min(width * 0.5).min(height * 0.5);
        if r <= EPSILON {
            return Sketch::rectangle(width, height, metadata);
        }
        // We'll approximate each 90° corner with `corner_segments` arcs
        let step = FRAC_PI_2 / corner_segments as Real;

        let corner = |cx, cy, start_angle| {
            (0..=corner_segments).map(move |i| {
                let angle: Real = start_angle + (i as Real) * step;
                (cx + r * angle.cos(), cy + r * angle.sin())
            })
        };

        let mut coords: Vec<(Real, Real)> = corner(r, r, PI) // Bottom-left
            .chain(corner(width - r, r, 1.5 * PI)) // Bottom-right
            .chain(corner(width - r, height - r, 0.0)) // Top-right
            .chain(corner(r, height - r, 0.5 * PI)) // Top-left
            .collect();

        coords.push(coords[0]); // close

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Squircle (superellipse) centered at (0,0) with bounding box width×height.
    /// We use an exponent = 4.0 for "classic" squircle shape. `segments` controls the resolution.
    pub fn squircle(
        width: Real,
        height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> Sketch<S> {
        if segments < 3 {
            return Sketch::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let m = 4.0;
        let mut coords: Vec<(Real, Real)> = (0..segments)
            .map(|i| {
                let t = TAU * (i as Real) / (segments as Real);
                let ct = t.cos().abs().powf(2.0 / m) * t.cos().signum();
                let st = t.sin().abs().powf(2.0 / m) * t.sin().signum();
                (rx * ct, ry * st)
            })
            .collect();
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Keyhole shape (simple version): a large circle + a rectangle "handle".
    /// This does *not* have a hole.  If you want a literal hole, you'd do difference ops.
    /// Here we do union of a circle and a rectangle.
    pub fn keyhole(
        circle_radius: Real,
        handle_width: Real,
        handle_height: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> Sketch<S> {
        if segments < 3 {
            return Sketch::new();
        }
        // 1) Circle
        let circle = Sketch::circle(circle_radius, segments, metadata.clone());

        // 2) Rectangle (handle)
        let handle = Sketch::rectangle(handle_width, handle_height, metadata).translate(
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
        metadata: Option<S>,
    ) -> Sketch<S> {
        if sides < 3 || circle_segments < 6 || diameter <= EPSILON {
            return Sketch::new();
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
        let base = Sketch::circle(diameter, circle_segments, metadata.clone())
            .translate(verts[0].0, verts[0].1, 0.0);

        let shape = verts.iter().skip(1).fold(base, |acc, &(x, y)| {
            let disk = Sketch::circle(diameter, circle_segments, metadata.clone())
                .translate(x, y, 0.0);
            acc.intersection(&disk)
        });

        Sketch {
            geometry: shape.geometry,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Outer diameter = `id + 2*thickness`. This yields an annulus in the XY plane.
    /// `segments` controls how smooth the outer/inner circles are.
    pub fn ring(id: Real, thickness: Real, segments: usize, metadata: Option<S>) -> Sketch<S> {
        if id <= 0.0 || thickness <= 0.0 || segments < 3 {
            return Sketch::new();
        }
        let inner_radius = 0.5 * id;
        let outer_radius = inner_radius + thickness;

        let outer_circle = Sketch::circle(outer_radius, segments, metadata.clone());
        let inner_circle = Sketch::circle(inner_radius, segments, metadata);

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
        metadata: Option<S>,
    ) -> Sketch<S> {
        if segments < 1 {
            return Sketch::new();
        }

        let start_rad = start_angle_deg.to_radians();
        let end_rad = end_angle_deg.to_radians();
        let sweep = end_rad - start_rad;

        // Build a ring of coordinates starting at (0,0), going around the arc, and closing at (0,0).
        let mut coords = Vec::with_capacity(segments + 2);
        coords.push((0.0, 0.0));
        for i in 0..=segments {
            let t = i as Real / (segments as Real);
            let angle = start_rad + t * sweep;
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            coords.push((x, y));
        }
        coords.push((0.0, 0.0)); // close explicitly

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
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
        metadata: Option<S>,
    ) -> Sketch<S> {
        if segments < 3 {
            return Sketch::new();
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

        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let frac = i as Real / (segments as Real);
            let theta = TAU * frac;
            let r = supershape_r(theta, a, b, m, n1, n2, n3);

            let x = r * theta.cos();
            let y = r * theta.sin();
            coords.push((x, y));
        }
        // close it
        coords.push(coords[0]);

        let polygon_2d = geo::Polygon::new(LineString::from(coords), vec![]);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Creates a 2D circle with a rectangular keyway slot cut out on the +X side.
    pub fn circle_with_keyway(
        radius: Real,
        segments: usize,
        key_width: Real,
        key_depth: Real,
        metadata: Option<S>,
    ) -> Sketch<S> {
        // 1. Full circle
        let circle = Sketch::circle(radius, segments, metadata.clone());

        // 2. Construct the keyway rectangle
        let key_rect = Sketch::rectangle(key_depth, key_width, metadata.clone()).translate(
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
        metadata: Option<S>,
    ) -> Sketch<S> {
        // 1. Full circle
        let circle = Sketch::circle(radius, segments, metadata.clone());

        // 2. Build a large rectangle that cuts off everything below y = -flat_dist
        let cutter_height = 9999.0; // some large number
        let rect_cutter = Sketch::rectangle(2.0 * radius, cutter_height, metadata.clone())
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
        metadata: Option<S>,
    ) -> Sketch<S> {
        // 1. Full circle
        let circle = Sketch::circle(radius, segments, metadata.clone());

        // 2. Large rectangle to cut the TOP (above +flat_dist)
        let cutter_height = 9999.0;
        let top_rect = Sketch::rectangle(2.0 * radius, cutter_height, metadata.clone())
            // place bottom at y=flat_dist
            .translate(-radius, flat_dist, 0.0);

        // 3. Large rectangle to cut the BOTTOM (below -flat_dist)
        let bottom_rect = Sketch::rectangle(2.0 * radius, cutter_height, metadata.clone())
            // place top at y=-flat_dist => bottom extends downward
            .translate(-radius, -cutter_height - flat_dist, 0.0);

        // 4. Subtract both
        let with_top_flat = circle.difference(&top_rect);

        with_top_flat.difference(&bottom_rect)
    }

    /// Sample an arbitrary-degree Bézier curve (de Casteljau).
    /// Returns a poly-line (closed if the first = last point).
    ///
    /// * `control`: list of 2-D control points
    /// * `segments`: number of straight-line segments used for the tessellation
    pub fn bezier(control: &[[Real; 2]], segments: usize, metadata: Option<S>) -> Self {
        if control.len() < 2 || segments < 1 {
            return Sketch::new();
        }

        // de Casteljau evaluator
        fn de_casteljau(
            ctrl: &[[Real; 2]],
            t: Real,
            tmp: &mut Vec<(Real, Real)>,
        ) -> (Real, Real) {
            tmp.clear();
            tmp.extend(ctrl.iter().map(|&[x, y]| (x, y)));
            let n = tmp.len();
            for k in 1..n {
                for i in 0..(n - k) {
                    tmp[i].0 = (1.0 - t) * tmp[i].0 + t * tmp[i + 1].0;
                    tmp[i].1 = (1.0 - t) * tmp[i].1 + t * tmp[i + 1].1;
                }
            }
            tmp[0]
        }

        let mut pts = Vec::<(Real, Real)>::with_capacity(segments + 1);
        let mut tmp = Vec::with_capacity(control.len());
        for i in 0..=segments {
            let t = i as Real / segments as Real;
            pts.push(de_casteljau(control, t, &mut tmp));
        }

        // If the curve happens to be closed, make sure the polygon ring closes.
        let closed = (pts.first().unwrap().0 - pts.last().unwrap().0).abs() < EPSILON
            && (pts.first().unwrap().1 - pts.last().unwrap().1).abs() < EPSILON;
        if !closed {
            // open curve → produce a LineString geometry, *not* a filled polygon
            let ls: LineString<Real> = pts.into();
            let mut gc = GeometryCollection::default();
            gc.0.push(Geometry::LineString(ls));
            return Sketch::from_geo(gc, metadata);
        }

        // closed curve → create a filled polygon
        let poly_2d = GeoPolygon::new(LineString::from(pts), vec![]);
        Sketch::from_geo(GeometryCollection(vec![Geometry::Polygon(poly_2d)]), metadata)
    }

    /// Sample an open-uniform B-spline of arbitrary degree (`p`) using the
    /// Cox-de Boor recursion. Returns a poly-line (or a filled region if closed).
    ///
    /// * `control`: control points  
    /// * `p`:       spline degree (e.g. 3 for a cubic)  
    /// * `segments_per_span`: tessellation resolution inside every knot span
    pub fn bspline(
        control: &[[Real; 2]],
        p: usize,
        segments_per_span: usize,
        metadata: Option<S>,
    ) -> Self {
        if control.len() < p + 1 || segments_per_span < 1 {
            return Sketch::new();
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
            let term1 = if denom1.abs() < EPSILON {
                0.0
            } else {
                (u - knot[i]) / denom1 * basis(i, p - 1, u, knot)
            };
            let term2 = if denom2.abs() < EPSILON {
                0.0
            } else {
                (knot[i + p + 1] - u) / denom2 * basis(i + 1, p - 1, u, knot)
            };
            term1 + term2
        }

        let span_count = n - p; // #inner knot spans
        let _max_u = span_count as Real; // parametric upper bound
        let dt = 1.0 / segments_per_span as Real; // step in local span coords

        let mut pts = Vec::<(Real, Real)>::new();
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
                pts.push((x, y));
            }
        }

        let closed = (pts.first().unwrap().0 - pts.last().unwrap().0).abs() < EPSILON
            && (pts.first().unwrap().1 - pts.last().unwrap().1).abs() < EPSILON;
        if !closed {
            let ls: LineString<Real> = pts.into();
            let mut gc = GeometryCollection::default();
            gc.0.push(Geometry::LineString(ls));
            return Sketch::from_geo(gc, metadata);
        }

        let poly_2d = GeoPolygon::new(LineString::from(pts), vec![]);
        Sketch::from_geo(GeometryCollection(vec![Geometry::Polygon(poly_2d)]), metadata)
    }

    /// 2-D heart outline (closed polygon) sized to `width` × `height`.
    ///
    /// `segments` controls smoothness (≥ 8 recommended).
    pub fn heart(width: Real, height: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 8 {
            return Sketch::new();
        }

        let step = TAU / segments as Real;

        // classic analytic “cardioid-style” heart
        let mut pts: Vec<(Real, Real)> = (0..segments)
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
        pts.push(pts[0]); // close

        // normalise & scale to desired bounding box ---------------------
        let (min_x, max_x) = pts.iter().fold((Real::MAX, -Real::MAX), |(lo, hi), &(x, _)| {
            (lo.min(x), hi.max(x))
        });
        let (min_y, max_y) = pts.iter().fold((Real::MAX, -Real::MAX), |(lo, hi), &(_, y)| {
            (lo.min(y), hi.max(y))
        });
        let s_x = width / (max_x - min_x);
        let s_y = height / (max_y - min_y);

        let coords: Vec<(Real, Real)> = pts
            .into_iter()
            .map(|(x, y)| ((x - min_x) * s_x, (y - min_y) * s_y))
            .collect();

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        Self::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// 2-D crescent obtained by subtracting a displaced smaller circle
    /// from a larger one.  
    /// `segments` controls circle smoothness.
    ///
    /// ```
    /// use csgrs::sketch::Sketch;
    /// let cres = Sketch::<()>::crescent(2.0, 1.4, 0.8, 64, None);
    /// ```
    pub fn crescent(
        outer_r: Real,
        inner_r: Real,
        offset: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> Self {
        if outer_r <= inner_r + EPSILON || segments < 6 {
            return Sketch::new();
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
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> Sketch<S> {
        assert!(teeth >= 4, "Need at least 4 teeth for a valid gear");
        assert!(segments_per_flank >= 3);

        // Standard proportions (ISO 21771)
        let m = module_;
        let z = teeth as Real;
        let pitch_radius = 0.5 * m * z;
        let addendum = m;
        let dedendum = 1.25 * m + clearance;

        let rb = pitch_radius * (1.0_f64.to_radians() as Real * pressure_angle_deg).cos();
        let ra = pitch_radius + addendum;
        let rf = (pitch_radius - dedendum).max(0.0);

        // Angular pitch and base offsets
        let ang_pitch = TAU / z;
        let tooth_thick_ang = ang_pitch / 2.0 - backlash / pitch_radius;

        // φ at pitch and addendum circles
        let phi_p = involute_angle_at_radius(pitch_radius, rb);
        let phi_a = involute_angle_at_radius(ra, rb);

        // Helper to build a single half‑flank (left‑hand)
        let mut half_flank = Vec::<(Real, Real)>::with_capacity(segments_per_flank + 1);
        for i in 0..=segments_per_flank {
            let phi = phi_p + (phi_a - phi_p) * (i as Real) / (segments_per_flank as Real);
            let (ix, iy) = involute_xy(rb, phi);
            let theta = (iy).atan2(ix); // polar angle of involute point
            let global_theta = -tooth_thick_ang + theta; // left side offset
            let r = (ix * ix + iy * iy).sqrt();
            half_flank.push((r * global_theta.cos(), r * global_theta.sin()));
        }

        // Mirror to get right‑hand flank (reverse order so outline is CCW)
        let mut full_tooth = half_flank.clone();
        for &(x, y) in half_flank.iter().rev() {
            // mirror across X axis and shift right
            let theta = (-y).atan2(x);
            let r = (x * x + y * y).sqrt();
            let global_theta = tooth_thick_ang - theta;
            full_tooth.push((r * global_theta.cos(), r * global_theta.sin()));
        }

        // Root circle arc between successive teeth
        let root_arc_steps = 4;
        let arc_step = (ang_pitch - 2.0 * tooth_thick_ang) / (root_arc_steps as Real);
        for i in 1..=root_arc_steps {
            let ang = tooth_thick_ang + (i as Real) * arc_step;
            full_tooth.push((rf * (ang).cos(), rf * (ang).sin()));
        }

        // Replicate the tooth profile around the gear
        let mut outline = Vec::<[Real; 2]>::with_capacity(full_tooth.len() * teeth + 1);
        for tooth_idx in 0..teeth {
            let rot = (tooth_idx as Real) * ang_pitch;
            let (c, s) = (rot.cos(), rot.sin());
            for &(x, y) in &full_tooth {
                outline.push([x * c - y * s, x * s + y * c]);
            }
        }
        // Close path
        outline.push(outline[0]);

        Sketch::polygon(&outline, metadata)
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
    pub fn cycloidal_gear(
        module_: Real,
        teeth: usize,
        pin_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> Sketch<S> {
        assert!(teeth >= 3 && pin_teeth >= 3);
        let m = module_;
        let z = teeth as Real;
        let z_p = pin_teeth as Real; // for pin‑wheel pairing

        // Pitch and derived radii
        let r_p = 0.5 * m * z; // gear pitch radius
        let r_g = 0.5 * m * z_p; // (made‑up) mating wheel for hypocycloid – gives correct root
        let r_pin = r_p / z; // generating circle radius (standard assumes z_p = z ± 1)

        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let _ra = r_p + addendum;
        let rf = (r_p - dedendum).max(0.0);

        let ang_pitch = TAU / z;
        let flank_steps = segments_per_flank.max(4);

        let mut tooth_points = Vec::<(Real, Real)>::new();

        // 1. addendum epicycloid (tip)
        for i in 0..=flank_steps {
            let t = (i as Real) / (flank_steps as Real);
            let theta = t * ang_pitch / 2.0;
            let (x, y) = epicycloid_xy(r_p, r_pin, theta);
            tooth_points.push((x, y));
        }
        // 2. hypocycloid root (reverse order to keep CCW)
        for i in (0..=flank_steps).rev() {
            let t = (i as Real) / (flank_steps as Real);
            let theta = t * ang_pitch / 2.0;
            let (x, y) = hypocycloid_xy(r_g, r_pin, theta);
            let r = (x * x + y * y).sqrt();
            if r < rf - EPSILON {
                tooth_points.push((rf * theta.cos(), rf * theta.sin()));
            } else {
                tooth_points.push((x, y));
            }
        }

        // Replicate
        let mut outline = Vec::<[Real; 2]>::with_capacity(tooth_points.len() * teeth + 1);
        for k in 0..teeth {
            let rot = (k as Real) * ang_pitch;
            let (c, s) = (rot.cos(), rot.sin());
            for &(x, y) in &tooth_points {
                outline.push([x * c - y * s, x * s + y * c]);
            }
        }
        outline.push(outline[0]);

        Sketch::polygon(&outline, metadata)
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
        metadata: Option<S>,
    ) -> Sketch<S> {
        assert!(num_teeth >= 1);
        let m = module_;
        let p = PI * m; // linear pitch
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let tip_y = addendum;
        let root_y = -dedendum;

        // Tooth thickness at pitch‑line (centre) minus backlash.
        let t = p / 2.0 - backlash;
        let half_t = t / 2.0;

        // Flank rises with slope = tan(pressure_angle)
        let alpha = pressure_angle_deg.to_radians();
        let rise = tip_y; // from pitch‑line (0) up to tip
        let run = rise / alpha.tan();

        // Build one tooth (start at pitch centre) – CCW
        // Points: Root‑left → Flank‑left → Tip‑left → Tip‑right → Flank‑right → Root‑right
        let tooth: Vec<[Real; 2]> = vec![
            [-half_t - run, root_y], // root left beneath flank
            [-half_t, 0.0],          // pitch left
            [-half_t + run, tip_y],  // tip left
            [half_t - run, tip_y],   // tip right
            [half_t, 0.0],           // pitch right
            [half_t + run, root_y],  // root right
        ];

        // Repeat teeth
        let mut outline = Vec::<[Real; 2]>::with_capacity(tooth.len() * num_teeth + 4);
        for i in 0..num_teeth {
            let dx = (i as Real) * p;
            for &[x, y] in &tooth {
                outline.push([x + dx, y]);
            }
        }

        // Close rectangle ends (simple straight ends)
        // add right root extension then back to first point
        outline.push([outline.last().unwrap()[0], 0.0]);
        outline.push([outline[0][0], 0.0]);
        outline.push(outline[0]);

        Sketch::polygon(&outline, metadata)
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
        metadata: Option<S>,
    ) -> Sketch<S> {
        assert!(num_teeth >= 1 && segments_per_flank >= 4);
        let m = module_;
        let p = PI * m;
        let addendum = m;
        let dedendum = 1.25 * m + clearance;
        let _tip_y = addendum;
        let root_y = -dedendum;

        let r = generating_radius;

        // Curtate cycloid y(t) spans 0..2πr giving height 2r.
        // We scale t so that y range equals addendum (= m)
        let scale = addendum / (2.0 * r);

        let mut flank: Vec<[Real; 2]> = Vec::with_capacity(segments_per_flank);
        for i in 0..=segments_per_flank {
            let t = PI * (i as Real) / (segments_per_flank as Real); // 0..π gives half‑trochoid
            let x = r * (t - t.sin());
            let y = r * (1.0 - t.cos());
            flank.push([x * scale, y * scale]);
        }

        // Build one tooth (CCW): left flank, mirrored right flank, root bridge
        let mut tooth: Vec<[Real; 2]> = Vec::with_capacity(flank.len() * 2 + 2);
        // Left side (reverse so CCW)
        for &[x, y] in flank.iter().rev() {
            tooth.push([-x, y]);
        }
        // Right side
        for &[x, y] in &flank {
            tooth.push([x, y]);
        }
        // Root bridge
        let bridge = tooth.last().unwrap()[0] + 2.0 * (r * scale - flank.last().unwrap()[0]);
        tooth.push([bridge, root_y]);
        tooth.push([-bridge, root_y]);

        // Repeat
        let mut outline = Vec::<[Real; 2]>::with_capacity(tooth.len() * num_teeth + 1);
        for k in 0..num_teeth {
            let dx = (k as Real) * p;
            for &[x, y] in &tooth {
                outline.push([x + dx, y]);
            }
        }
        outline.push(outline[0]);

        Sketch::polygon(&outline, metadata)
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
    pub fn airfoil_naca4(
        max_camber: Real,
        camber_position: Real,
        thickness: Real,
        chord: Real,
        samples: usize,
        metadata: Option<S>,
    ) -> Sketch<S> {
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
        let mut coords: Vec<(Real, Real)> = Vec::with_capacity(2 * samples + 1);

        // leading-edge → trailing-edge (upper)
        for i in 0..=samples {
            let xc = i as Real / n; // 0–1
            let x = xc * chord; // physical
            let t = half_profile(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();

            let xu = x - t * theta.sin();
            let yu = chord * (yc_val + t * theta.cos());
            coords.push((xu, yu));
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
            coords.push((xl, yl));
        }

        coords.push(coords[0]); // close

        let polygon_2d =
            GeoPolygon::new(LineString::from(coords), vec![]).orient(Direction::Default);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }
}

// -------------------------------------------------------------------------------------------------
// Involute helper                                                                               //
// -------------------------------------------------------------------------------------------------

/// Classic parametric involute of a circle calculation.
///
/// # Parameters
/// - `rb`: base-circle radius
/// - `phi`: involute parameter
///
/// x = rb( cosφ + φ·sinφ )
/// y = rb( sinφ – φ·cosφ )
///
/// # Returns
/// Cartesian coordinates (x, y) of the involute point
#[inline]
fn involute_xy(rb: Real, phi: Real) -> (Real, Real) {
    (
        rb * (phi.cos() + phi * phi.sin()),
        rb * (phi.sin() - phi * phi.cos()),
    )
}

/// Calculate the involute angle at a given radius.
///
/// # Parameters
/// - `r`: radius at which to calculate the angle
/// - `rb`: base circle radius
///
/// # Returns
/// The involute angle φ = sqrt((r/rb)² - 1)
#[inline]
fn involute_angle_at_radius(r: Real, rb: Real) -> Real {
    ((r / rb).powi(2) - 1.0).max(0.0).sqrt()
}

// -------------------------------------------------------------------------------------------------
// Cycloid helpers                                                                               //
// -------------------------------------------------------------------------------------------------

/// Generate epicycloid coordinates for gear tooth profiles.
///
/// # Parameters
/// - `r_g`: pitch-circle radius
/// - `r_p`: pin circle (generating circle) radius
/// - `theta`: parameter angle
///
/// # Returns
/// Cartesian coordinates (x, y) of the epicycloid point
#[inline]
fn epicycloid_xy(r_g: Real, r_p: Real, theta: Real) -> (Real, Real) {
    // r_g : pitch‑circle radius, r_p : pin circle (generating circle) radius
    // x = (r_g + r_p) (cos θ) – r_p cos((r_g + r_p)/r_p · θ)
    // y = (r_g + r_p) (sin θ) – r_p sin((r_g + r_p)/r_p · θ)
    let k = (r_g + r_p) / r_p;
    (
        (r_g + r_p) * theta.cos() - r_p * (k * theta).cos(),
        (r_g + r_p) * theta.sin() - r_p * (k * theta).sin(),
    )
}

/// Generate hypocycloid coordinates for gear root flanks.
///
/// # Parameters
/// - `r_g`: pitch-circle radius
/// - `r_p`: pin circle (generating circle) radius
/// - `theta`: parameter angle
///
/// # Returns
/// Cartesian coordinates (x, y) of the hypocycloid point
#[inline]
fn hypocycloid_xy(r_g: Real, r_p: Real, theta: Real) -> (Real, Real) {
    // For root flank of a cycloidal tooth
    let k = (r_g - r_p) / r_p;
    (
        (r_g - r_p) * theta.cos() + r_p * (k * theta).cos(),
        (r_g - r_p) * theta.sin() - r_p * (k * theta).sin(),
    )
}
