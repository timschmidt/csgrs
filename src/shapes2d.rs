use crate::csg::CSG;
use crate::float_types::{Real, PI, EPSILON, FRAC_PI_2, TAU};
use geo::{line_string, GeometryCollection, Geometry, LineString, MultiPolygon, Polygon as GeoPolygon, BooleanOps};
use std::fmt::Debug;

impl<S: Clone + Debug> CSG<S>
where S: Clone + Send + Sync {
    /// Creates a 2D square in the XY plane.
    ///
    /// # Parameters
    ///
    /// - `width`: the width of the square
    /// - `length`: the height of the square
    /// - `metadata`: optional metadata
    ///
    /// # Example
    /// let sq2 = CSG::square(2.0, 3.0, None);
    pub fn square(width: Real, length: Real, metadata: Option<S>) -> Self {
        // In geo, a Polygon is basically (outer: LineString, Vec<LineString> for holes).
        let outer = line_string![
            (x: 0.0,     y: 0.0),
            (x: width,   y: 0.0),
            (x: width,   y: length),
            (x: 0.0,     y: length),
            (x: 0.0,     y: 0.0),  // close explicitly
        ];
        let polygon_2d = GeoPolygon::new(outer, vec![]);

        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Creates a 2D circle in the XY plane.
    pub fn circle(radius: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let theta = 2.0 * PI * (i as Real) / (segments as Real);
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            coords.push((x, y));
        }
        // close it
        coords.push((coords[0].0, coords[0].1));
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Right triangle from (0,0) to (width,0) to (0,height).
    pub fn right_triangle(width: Real, height: Real, metadata: Option<S>) -> Self {
        let line_string: LineString = vec![[0.0, 0.0], [width, 0.0], [0.0, height]].into();
        let polygon = GeoPolygon::new(line_string, vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon)]),
            metadata,
        )
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
    /// let poly2d = CSG::polygon(&pts, metadata);
    pub fn polygon(points: &[[Real; 2]], metadata: Option<S>) -> Self {
        if points.len() < 3 {
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(points.len() + 1);
        for p in points {
            coords.push((p[0], p[1]));
        }
        // close
        if coords[0] != *coords.last().unwrap() {
            coords.push(coords[0]);
        }
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);

        CSG::from_geo(
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
            return Self::square(width, height, metadata);
        }
        let mut coords = Vec::new();
        // We'll approximate each 90° corner with `corner_segments` arcs
        let step = FRAC_PI_2 / corner_segments as Real;

        // Top-left corner arc, center (r, height-r), (π → 3π/2) angles 180 -> 270
        let cx_tl = r;
        let cy_tl = height - r;
        for i in 0..=corner_segments {
            let angle = FRAC_PI_2 + (i as Real) * step;
            let x = cx_tl + r * angle.cos();
            let y = cy_tl + r * angle.sin();
            coords.push((x, y));
        }

        // Bottom-left corner arc, center (r, r), (π/2 → π) angles 90 -> 180
        let cx_bl = r;
        let cy_bl = r;
        for i in 0..=corner_segments {
            let angle = PI + (i as Real) * step;
            let x = cx_bl + r * angle.cos();
            let y = cy_bl + r * angle.sin();
            coords.push((x, y));
        }

        // Bottom-right corner arc, center (width-r, r), (0 → π/2) angles 0 -> 90
        let cx_br = width - r;
        let cy_br = r;
        for i in 0..=corner_segments {
            let angle = 1.5 * PI + (i as Real) * step;
            let x = cx_br + r * angle.cos();
            let y = cy_br + r * angle.sin();
            coords.push((x, y));
        }

        // Top-right corner arc, center (width-r, height-r), (3π/2 → 2π) angles 270 -> 360
        let cx_tr = width - r;
        let cy_tr = height - r;
        for i in 0..=corner_segments {
            let angle = 0.0 + (i as Real) * step;
            let x = cx_tr + r * angle.cos();
            let y = cy_tr + r * angle.sin();
            coords.push((x, y));
        }

        // close
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Ellipse in XY plane, centered at (0,0), with full width `width`, full height `height`.
    /// `segments` is the number of polygon edges approximating the ellipse.
    pub fn ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>) -> Self {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let theta = TAU * (i as Real) / (segments as Real);
            let x = rx * theta.cos();
            let y = ry * theta.sin();
            coords.push((x, y));
        }
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Regular N-gon in XY plane, centered at (0,0), with circumscribed radius `radius`.
    /// `sides` is how many edges (>=3).
    pub fn regular_ngon(sides: usize, radius: Real, metadata: Option<S>) -> Self {
        if sides < 3 {
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(sides + 1);
        for i in 0..sides {
            let theta = TAU * (i as Real) / (sides as Real);
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            coords.push((x, y));
        }
        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
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
        CSG::from_geo(
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
            return CSG::new();
        }
        let mut coords = Vec::with_capacity(2 * num_points + 1);
        let step = TAU / (num_points as Real);
        for i in 0..num_points {
            // Outer point
            let theta_out = i as Real * step;
            let x_out = outer_radius * theta_out.cos();
            let y_out = outer_radius * theta_out.sin();
            coords.push((x_out, y_out));

            // Inner point
            let theta_in = theta_out + 0.5 * step;
            let x_in = inner_radius * theta_in.cos();
            let y_in = inner_radius * theta_in.sin();
            coords.push((x_in, y_in));
        }
        // close
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Teardrop shape.  A simple approach:
    /// - a circle arc for the "round" top
    /// - it tapers down to a cusp at bottom.
    /// This is just one of many possible "teardrop" definitions.
    // todo: center on focus of the arc
    pub fn teardrop_outline(
        width: Real,
        length: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 2 || width < EPSILON || length < EPSILON {
            return CSG::new();
        }
        let r = 0.5 * width;
        let center_y = length - r;
        let half_seg = segments / 2;

        // We’ll store points, starting from the bottom tip at (0,0).
        let mut coords = Vec::with_capacity(segments + 2);
        coords.push((0.0, 0.0));

        // Arc around
        for i in 0..=half_seg {
            let t = PI * (i as Real / half_seg as Real);
            let x = -r * t.cos(); // left
            let y = center_y + r * t.sin();
            coords.push((x, y));
        }

        coords.push(coords[0]);
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Egg outline.  Approximate an egg shape using a parametric approach.
    /// This is only a toy approximation.  It creates a closed "egg-ish" outline around the origin.
    pub fn egg_outline(width: Real, length: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * length;
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let theta = TAU * (i as Real) / (segments as Real);
            // toy distortion approach
            let distort = 1.0 + 0.2 * theta.cos();
            let x = rx * theta.sin();
            let y = ry * theta.cos() * distort * 0.8;
            coords.push((-x, y)); // mirrored
        }
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Squircle (superellipse) centered at (0,0) with bounding box width×height.
    /// We use an exponent = 4.0 for "classic" squircle shape. `segments` controls the resolution.
    pub fn squircle(width: Real, height: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        let rx = 0.5 * width;
        let ry = 0.5 * height;
        let m = 4.0;
        let mut coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let t = TAU * (i as Real) / (segments as Real);
            let ct = t.cos().abs().powf(2.0 / m) * t.cos().signum();
            let st = t.sin().abs().powf(2.0 / m) * t.sin().signum();
            let x = rx * ct;
            let y = ry * st;
            coords.push((x, y));
        }
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
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
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
        }
        // 1) Circle
        let mut circle_coords = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let th = TAU * (i as Real) / (segments as Real);
            circle_coords.push((circle_radius * th.cos(), circle_radius * th.sin()));
        }
        circle_coords.push(circle_coords[0]);
        let circle_poly = GeoPolygon::new(LineString::from(circle_coords), vec![]);

        // 2) Rectangle (handle), from -hw/2..+hw/2 in X and 0..handle_height in Y
        let rect_coords = vec![
            (-0.5 * handle_width, 0.0),
            ( 0.5 * handle_width, 0.0),
            ( 0.5 * handle_width, handle_height),
            (-0.5 * handle_width, handle_height),
            (-0.5 * handle_width, 0.0),
        ];
        let rect_poly = GeoPolygon::new(LineString::from(rect_coords), vec![]);

        // 3) Union them with geo’s BooleanOps
        let mp1 = MultiPolygon(vec![circle_poly]);
        let mp2 = MultiPolygon(vec![rect_poly]);
        let multipolygon_2d = mp1.union(&mp2);

        CSG::from_geo(
            GeometryCollection(vec![Geometry::MultiPolygon(multipolygon_2d)]),
            metadata,
        )
    }

    /// Reuleaux polygon (constant–width curve) built as the *intersection* of
    /// `sides` equal–radius disks whose centres are the vertices of a regular
    /// n-gon.
    ///
    /// * `sides`                  ≥ 3  
    /// * `diameter`               desired constant width (equals the distance
    ///                            between adjacent vertices, i.e. the polygon’s
    ///                            edge length)
    /// * `circle_segments`        how many segments to use for each disk
    ///
    /// For `sides == 3` this gives the canonical Reuleaux triangle; for any
    /// larger `sides` it yields the natural generalisation (odd-sided shapes
    /// retain constant width, even-sided ones do not but are still smooth).
    pub fn reuleaux_polygon(
        sides: usize,
        diameter: Real,
        circle_segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {    
        if sides < 3 || circle_segments < 6 || diameter <= EPSILON {
            return CSG::new();
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
        let base = CSG::circle(diameter, circle_segments, metadata.clone())
            .translate(verts[0].0, verts[0].1, 0.0);
    
        let shape = verts.iter().skip(1).fold(base, |acc, &(x, y)| {
            let disk = CSG::circle(diameter, circle_segments, metadata.clone())
                .translate(x, y, 0.0);
            acc.intersection(&disk)
        });

        CSG {
            geometry: shape.geometry,
            polygons: shape.polygons,
            metadata,
        }
    }

    /// Ring with inner diameter = `id` and (radial) thickness = `thickness`.
    /// Outer diameter = `id + 2*thickness`. This yields an annulus in the XY plane.
    /// `segments` controls how smooth the outer/inner circles are.
    ///
    /// Internally, we do:
    ///   outer = circle(outer_radius)
    ///   inner = circle(inner_radius)
    ///   ring = outer.difference(inner)
    /// Then we call `flatten()` to unify into a single shape that has a hole.
    pub fn ring(id: Real, thickness: Real, segments: usize, metadata: Option<S>) -> CSG<S> {
        if id <= 0.0 || thickness <= 0.0 || segments < 3 {
            return CSG::new();
        }
        let inner_radius = 0.5 * id;
        let outer_radius = inner_radius + thickness;

        // Outer ring (CCW)
        let mut outer = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let th = TAU * (i as Real) / (segments as Real);
            let x = outer_radius * th.cos();
            let y = outer_radius * th.sin();
            outer.push((x, y));
        }
        outer.push(outer[0]);

        // Inner ring (must be opposite orientation for a hole in geo)
        let mut inner = Vec::with_capacity(segments + 1);
        for i in 0..segments {
            let th = TAU * (i as Real) / (segments as Real);
            let x = inner_radius * th.cos();
            let y = inner_radius * th.sin();
            inner.push((x, y));
        }
        inner.push(inner[0]);
        inner.reverse(); // ensure hole is opposite winding from outer

        let polygon_2d = GeoPolygon::new(LineString::from(outer), vec![LineString::from(inner)]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
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
    ) -> CSG<S> {
        if segments < 1 {
            return CSG::new();
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
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Create a 2D supershape in the XY plane, approximated by `segments` edges.
    /// The superformula parameters are typically:
    ///   r(θ) = [ (|cos(mθ/4)/a|^n2 + |sin(mθ/4)/b|^n3) ^ (-1/n1) ]
    /// Adjust as needed for your use-case.
    pub fn supershape(
        a: Real,
        b: Real,
        m: Real,
        n1: Real,
        n2: Real,
        n3: Real,
        segments: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if segments < 3 {
            return CSG::new();
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
        CSG::from_geo(
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
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());

        // 2. Construct the keyway rectangle:
        //    - width along X = key_depth
        //    - height along Y = key_width
        //    - its right edge at x = +radius
        //    - so it spans from x = (radius - key_depth) to x = radius
        //    - and from y = -key_width/2 to y = +key_width/2
        let key_rect = CSG::square(key_depth, key_width, metadata.clone()).translate(
            radius - key_depth,
            -key_width * 0.5,
            0.0,
        );

        circle.difference(&key_rect)
    }

    /// Creates a 2D "D" shape (circle with one flat chord).
    /// `radius` is the circle radius,
    /// `flat_dist` is how far from the center the flat chord is placed.
    ///   - If flat_dist == 0.0 => chord passes through center => a half-circle
    ///   - If flat_dist < radius => chord is inside the circle => typical "D" shape
    ///
    /// Solve for distance from center using width of flat:
    /// let half_c = chord_len / 2.0;
    /// let flat_dist = (radius*radius - half_c*half_c).sqrt();
    pub fn circle_with_flat(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());

        // 2. Build a large rectangle that cuts off everything below y = -flat_dist
        //    (i.e., we remove that portion to create a chord).
        //    Width = 2*radius is plenty to cover the circle’s X-range.
        //    Height = large enough, we just shift it so top edge is at y = -flat_dist.
        //    So that rectangle covers from y = -∞ up to y = -flat_dist.
        let cutter_height = 9999.0; // some large number
        let rect_cutter = CSG::square(2.0 * radius, cutter_height, metadata.clone())
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
    ) -> CSG<S> {
        // 1. Full circle
        let circle = CSG::circle(radius, segments, metadata.clone());

        // 2. Large rectangle to cut the TOP (above +flat_dist)
        let cutter_height = 9999.0;
        let top_rect = CSG::square(2.0 * radius, cutter_height, metadata.clone())
            // place bottom at y=flat_dist
            .translate(-radius, flat_dist, 0.0);

        // 3. Large rectangle to cut the BOTTOM (below -flat_dist)
        let bottom_rect = CSG::square(2.0 * radius, cutter_height, metadata.clone())
            // place top at y=-flat_dist => bottom extends downward
            .translate(-radius, -cutter_height - flat_dist, 0.0);

        // 4. Subtract both
        let with_top_flat = circle.difference(&top_rect);
        let with_both_flats = with_top_flat.difference(&bottom_rect);

        with_both_flats
    }
    
    /// Generate a NACA 4-digit airfoil (e.g. "2412", "0015").
    ///
    /// * `code` – 4 ASCII digits describing **camber**, **camber-pos**, **thickness**  
    /// * `chord` – physical chord length you want (same units as the rest of your model)  
    /// * `samples` – number of points per surface (≥ 10 is sensible; NP total = 2 × samples + 1)  
    ///
    /// The function returns a single closed polygon lying in the *XY* plane with its
    /// leading edge at the origin and the chord running along +X.
    pub fn airfoil(
        code: &str,
        chord: Real,
        samples: usize,
        metadata: Option<S>,
    ) -> CSG<S>
    where
        S: Clone + Send + Sync,
    {    
        assert!(
            code.len() == 4 && code.chars().all(|c| c.is_ascii_digit()),
            "NACA code must be exactly 4 digits"
        );
        assert!(samples >= 10, "Need at least 10 points per surface");
    
        // --- decode code -------------------------------------------------------
        let m  = code[0..1].parse::<Real>().unwrap() / 100.0; // max-camber %
        let p  = code[1..2].parse::<Real>().unwrap() / 10.0;  // camber-pos
        let tt = code[2..4].parse::<Real>().unwrap() / 100.0; // thickness %
    
        // thickness half-profile -----------------------------------------------
        let yt = |x: Real| -> Real {
            5.0 * tt
                * (0.2969 * x.sqrt()
                    - 0.1260 * x
                    - 0.3516 * x * x
                    + 0.2843 * x * x * x
                    - 0.1015 * x * x * x * x)
        };
    
        // mean-camber line & slope ---------------------------------------------
        let camber = |x: Real| -> (Real, Real) {
            if x < p {
                let yc = m / (p * p) * (2.0 * p * x - x * x);
                let dy = 2.0 * m / (p * p) * (p - x);
                (yc, dy)
            } else {
                let yc = m / ((1.0 - p).powi(2))
                    * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x);
                let dy = 2.0 * m / ((1.0 - p).powi(2)) * (p - x);
                (yc, dy)
            }
        };
    
        // --- sample upper & lower surfaces ------------------------------------
        let n = samples as Real;
        let mut coords: Vec<(Real, Real)> = Vec::with_capacity(2 * samples + 1);
    
        // leading-edge → trailing-edge (upper)
        for i in 0..=samples {
            let xc = i as Real / n;          // 0–1
            let x  = xc * chord;             // physical
            let t  = yt(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();
    
            let xu = x - t * theta.sin();
            let yu = chord * (yc_val + t * theta.cos());
            coords.push((xu, yu));
        }
    
        // trailing-edge → leading-edge (lower)
        for i in (1..samples).rev() {
            let xc = i as Real / n;
            let x  = xc * chord;
            let t  = yt(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();
    
            let xl = x + t * theta.sin();
            let yl = chord * (yc_val - t * theta.cos());
            coords.push((xl, yl));
        }
    
        coords.push(coords[0]); // close
    
        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }
    
    /// Sample an arbitrary-degree Bézier curve (de Casteljau).
    /// Returns a poly-line (closed if the first = last point).
    ///
    /// * `control`: list of 2-D control points
    /// * `segments`: number of straight-line segments used for the tessellation
    pub fn bezier(
        control: &[[Real; 2]],
        segments: usize,
        metadata: Option<S>,
    ) -> Self {
        if control.len() < 2 || segments < 1 {
            return CSG::new();
        }
    
        // de Casteljau evaluator -------------------------------------------------
        fn de_casteljau(ctrl: &[[Real; 2]], t: Real) -> (Real, Real) {
            let mut tmp: Vec<(Real, Real)> =
                ctrl.iter().map(|&[x, y]| (x, y)).collect();
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
        for i in 0..=segments {
            let t = i as Real / segments as Real;
            pts.push(de_casteljau(control, t));
        }
    
        // If the curve happens to be closed, make sure the polygon ring closes.
        let closed = (pts.first().unwrap().0 - pts.last().unwrap().0).abs() < EPSILON
            && (pts.first().unwrap().1 - pts.last().unwrap().1).abs() < EPSILON;
        if !closed {
            // open curve → produce a LineString geometry, *not* a filled polygon
            let ls: LineString<Real> = pts.into();
            let mut gc = GeometryCollection::default();
            gc.0.push(Geometry::LineString(ls));
            return CSG::from_geo(gc, metadata);
        }
    
        // closed curve → create a filled polygon
        let poly_2d = GeoPolygon::new(LineString::from(pts), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(poly_2d)]), metadata)
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
            return CSG::new();
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
    
        // Cox-de Boor basis evaluation ------------------------------------------
        fn basis(i: usize, p: usize, u: Real, knot: &[Real]) -> Real {
            if p == 0 {
                return if u >= knot[i] && u < knot[i + 1] { 1.0 } else { 0.0 };
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
    
        let span_count = n - p;                   // #inner knot spans
        let _max_u = span_count as Real;           // parametric upper bound
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
            return CSG::from_geo(gc, metadata);
        }
    
        let poly_2d = GeoPolygon::new(LineString::from(pts), vec![]);
        CSG::from_geo(GeometryCollection(vec![Geometry::Polygon(poly_2d)]), metadata)
    }
    
    // -------------------------------------------------------------------------------------------------
    // 2‑D gear outlines                                                                             //
    // -------------------------------------------------------------------------------------------------
    
    /// Involute gear outline (2‑D).
    #[allow(clippy::too_many_arguments)]
    pub fn involute_gear_2d(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
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
            let theta = ( -y).atan2(x);
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
    
        CSG::polygon(&outline, metadata)
    }
    
    /// (Epicyclic) cycloidal gear outline (2‑D).
    #[allow(clippy::too_many_arguments)]
    pub fn cycloidal_gear_2d(
        module_: Real,
        teeth: usize,
        pin_teeth: usize,
        clearance: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
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
    
        CSG::polygon(&outline, metadata)
    }
    
    /// Linear **involute rack** profile (lying in the *XY* plane, pitch‑line on *Y = 0*).
    /// The returned polygon is CCW and spans `num_teeth` pitches along +X.
    pub fn involute_rack_2d(
        module_: Real,
        num_teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        metadata: Option<S>,
    ) -> CSG<S> {
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
            [-half_t - run, root_y],  // root left beneath flank
            [-half_t, 0.0],           // pitch left
            [-half_t + run, tip_y],   // tip left
            [half_t - run, tip_y],    // tip right
            [half_t, 0.0],            // pitch right
            [half_t + run, root_y],   // root right
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
    
        CSG::polygon(&outline, metadata)
    }
    
    /// Linear **cycloidal rack** profile.
    /// The cycloidal rack is generated by rolling a circle of radius `r_p` along the
    /// rack’s pitch‑line.  The flanks become a *trochoid*; for practical purposes we
    /// approximate with the classic curtate cycloid equations.
    pub fn cycloidal_rack_2d(
        module_: Real,
        num_teeth: usize,
        generating_radius: Real, // usually = module_/2
        clearance: Real,
        segments_per_flank: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
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
    
        CSG::polygon(&outline, metadata)
    }
}

// -------------------------------------------------------------------------------------------------
// Involute helper                                                                               //
// -------------------------------------------------------------------------------------------------

#[inline]
fn involute_xy(rb: Real, phi: Real) -> (Real, Real) {
    // Classic parametric involute of a circle (rb = base‑circle radius).
    // x = rb( cosφ + φ·sinφ )
    // y = rb( sinφ – φ·cosφ )
    (
        rb * (phi.cos() + phi * phi.sin()),
        rb * (phi.sin() - phi * phi.cos()),
    )
}

#[inline]
fn involute_angle_at_radius(r: Real, rb: Real) -> Real {
    // φ = sqrt( (r/rb)^2 – 1 )
    ((r / rb).powi(2) - 1.0).max(0.0).sqrt()
}

// -------------------------------------------------------------------------------------------------
// Cycloid helpers                                                                               //
// -------------------------------------------------------------------------------------------------

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

#[inline]
fn hypocycloid_xy(r_g: Real, r_p: Real, theta: Real) -> (Real, Real) {
    // For root flank of a cycloidal tooth
    let k = (r_g - r_p) / r_p;
    (
        (r_g - r_p) * theta.cos() + r_p * (k * theta).cos(),
        (r_g - r_p) * theta.sin() - r_p * (k * theta).sin(),
    )
}
