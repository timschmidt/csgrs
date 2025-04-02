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

    /// Reuleaux polygon with `sides` and "radius".  Approximates constant-width shape.
    /// This is a simplified approximation that arcs from each vertex to the next.
    pub fn reuleaux_polygon(
        sides: usize,
        radius: Real,
        arc_segments_per_side: usize,
        metadata: Option<S>,
    ) -> CSG<S> {
        if sides < 3 || arc_segments_per_side < 1 {
            return CSG::new();
        }
        // Corner positions (the "center" of each arc is the next corner).
        let mut corners = Vec::with_capacity(sides);
        for i in 0..sides {
            let theta = TAU * (i as Real) / (sides as Real);
            corners.push((radius * theta.cos(), radius * theta.sin()));
        }

        // Build one big ring of points by tracing arcs corner->corner.
        let mut coords = Vec::new();
        for i in 0..sides {
            let i_next = (i + 1) % sides;
            let center = corners[i_next];
            let start_pt = corners[i];
            let end_pt = corners[(i + 2) % sides];

            let vx_s = start_pt.0 - center.0;
            let vy_s = start_pt.1 - center.1;
            let start_angle = vy_s.atan2(vx_s);

            let vx_e = end_pt.0 - center.0;
            let vy_e = end_pt.1 - center.1;
            let end_angle = vy_e.atan2(vx_e);

            let mut delta = end_angle - start_angle;
            while delta <= 0.0 {
                delta += TAU;
            }
            let step = delta / (arc_segments_per_side as Real);
            for seg_i in 0..arc_segments_per_side {
                let a = start_angle + (seg_i as Real) * step;
                let x = center.0 + radius * a.cos();
                let y = center.1 + radius * a.sin();
                coords.push((x, y));
            }
        }
        coords.push(coords[0]);

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![]);
        CSG::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
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
    ///
    /// The superformula parameters are typically:\
    ///   r(θ) = [ (|cos(mθ/4)/a|^n2 + |sin(mθ/4)/b|^n3) ^ (-1/n1) ]
    ///
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
}
