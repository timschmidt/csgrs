//! **Mathematical Foundations for Polygon Offsetting**
//!
//! This module implements robust polygon offsetting (buffering) operations based on
//! computational geometry algorithms. The operations grow or shrink polygons by a
//! specified distance while maintaining topological correctness.
//!
//! ## **Theoretical Foundation**
//!
//! ### **Minkowski Sum Definition**
//! For a polygon P and disk D of radius r, the offset operation computes:
//! ```text
//! P ⊕ D = {p + d | p ∈ P, d ∈ D}
//! ```
//! This is equivalent to:
//! - **Outward offset (r > 0)**: Expand polygon by distance r
//! - **Inward offset (r < 0)**: Shrink polygon by distance |r|
//!
//! ### **Geometric Interpretation**
//! The offset operation can be visualized as:
//! 1. **Straight Segments**: Move parallel to original by distance r
//! 2. **Convex Vertices**: Add circular arc of radius r
//! 3. **Concave Vertices**: May create self-intersections requiring resolution
//!
//! ### **Corner Treatment Options**
//! - **Rounded**: Use circular arcs at vertices (C¹ continuity)
//! - **Sharp**: Use angular joints (C⁰ continuity)
//!
//! ## **Algorithm Implementation**
//!
//! This compatibility implementation currently uses `geo`'s buffer operations
//! over the finite boundary view reconstructed from [`Sketch::region`] when the
//! legacy geometry cache is empty. The offset result is immediately promoted
//! back into hypercurve contours, keeping exact-aware topology as the Sketch
//! data model while this module migrates to native hypercurve offsets.
//!
//! Offset construction and trimming are classical CAD problems; see Farouki
//! and Neff, "Analytic properties of plane offset curves," *Computer Aided
//! Geometric Design* 7(1-4), 1990
//! (<https://doi.org/10.1016/0167-8396(90)90023-K>), and Tiller and Hanson,
//! "Offsets of two-dimensional profiles," *IEEE Computer Graphics and
//! Applications* 4(9), 1984 (<https://doi.org/10.1109/MCG.1984.276011>).
//!
//! `geo`'s buffer operations provide:
//! - **Robust intersection handling**: Resolves self-intersections
//! - **Topological correctness**: Maintains polygon validity
//! - **Multi-polygon support**: Handles complex geometry with holes
//! - **Numerical stability**: Handles degenerate cases gracefully
//!
//! ## **Applications**
//! - **Toolpath Generation**: CNC machining offset paths
//! - **Buffer Zones**: GIS proximity analysis
//! - **Collision Detection**: Expanded bounding regions
//! - **Typography**: Font outline generation
//!
//! All operations preserve the 3D polygon structure while applying 2D offsetting
//! to the planar projections stored in the geometry collection.
use crate::float_types::{Real, hreal_from_f64};
use crate::sketch::Sketch;
use geo::algorithm::map_coords::MapCoords;
use geo::{
    Buffer, Coord, Geometry, GeometryCollection, LineString, MultiPolygon, Polygon,
    algorithm::buffer::{BufferStyle, LineJoin},
};
use hypercurve::{
    Classification, CurvePolicy, FiniteProjectionOptions, Region2, finite_ring_signed_area,
};
use std::borrow::Cow;
use std::fmt::Debug;

#[allow(clippy::unnecessary_cast)]
fn skel_poly(poly: &Polygon<Real>, inward: bool) -> Vec<LineString<Real>> {
    let poly_f64 = poly.map_coords(|c| Coord {
        x: c.x as f64,
        y: c.y as f64,
    });

    geo_buffer::skeleton_of_polygon_to_linestring(&poly_f64, inward)
        .into_iter()
        .map(|ls| {
            ls.map_coords(|c| Coord {
                x: c.x as Real,
                y: c.y as Real,
            })
        })
        .collect()
}

#[allow(clippy::unnecessary_cast)]
fn skel_multi_poly(mpoly: &MultiPolygon<Real>, inward: bool) -> Vec<LineString<Real>> {
    let mpoly_f64 = mpoly.map_coords(|c| Coord {
        x: c.x as f64,
        y: c.y as f64,
    });

    geo_buffer::skeleton_of_multi_polygon_to_linestring(&mpoly_f64, inward)
        .into_iter()
        .map(|ls| {
            ls.map_coords(|c| Coord {
                x: c.x as Real,
                y: c.y as Real,
            })
        })
        .collect()
}

impl<M: Clone + Debug + Send + Sync> Sketch<M> {
    /// Finite offsetting/skeleton input regenerated from native hypercurve
    /// regions when possible.
    ///
    /// The current offset backend remains a `geo` compatibility bridge, but
    /// area topology is sourced from `Region2` and sidecar open paths are
    /// preserved as native `CurveString2` wires rather than being inferred from
    /// the temporary cache. This preserves csgrs' hyperreal boundary discipline
    /// while the native hypercurve offset path is being brought over; see Yap,
    /// "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>), and
    /// Farouki and Neff, "Analytic properties of plane offset curves,"
    /// *Computer Aided Geometric Design* 7(1-4), 1990
    /// (<https://doi.org/10.1016/0167-8396(90)90023-K>).
    fn offset_compat_geometry(&self) -> Cow<'_, GeometryCollection<Real>> {
        if !self.as_region().is_empty() && self.geometry.0.is_empty() {
            Cow::Owned(Self::geometry_from_native(self.as_region(), self.wires()))
        } else if !self.as_region().is_empty()
            && self.wires().is_empty()
            && self.compat_geometry_is_area_only()
        {
            Cow::Owned(Self::geometry_from_region(self.as_region()))
        } else {
            self.effective_geometry()
        }
    }

    fn preserve_offset_wires(&self, sketch: &mut Sketch<M>) {
        if !self.wires().is_empty() {
            sketch.wires.extend(self.wires().iter().cloned());
            sketch.geometry = Self::geometry_from_native(sketch.as_region(), sketch.wires());
        }
    }

    /// Try a native hypercurve sharp offset before falling back to `geo`.
    ///
    /// This covers the conservative material-only case where each contour can
    /// be offset independently and checked for self-contact by hypercurve. Full
    /// regularized offsets with holes, splits, and merged components still use
    /// the finite buffer bridge until the trim/rebuild stage is moved into the
    /// hyper crates. This follows the primitive-offset staging in Tiller and
    /// Hanson, "Offsets of Two-Dimensional Profiles," *IEEE Computer Graphics
    /// and Applications* 4(9), 1984 (<https://doi.org/10.1109/MCG.1984.276011>)
    /// and the offset-topology caveats in Farouki and Neff, "Analytic
    /// properties of plane offset curves," *Computer Aided Geometric Design*
    /// 7(1-4), 1990 (<https://doi.org/10.1016/0167-8396(90)90023-K>).
    fn native_sharp_offset(&self, distance: Real) -> Option<Sketch<M>> {
        if self.region.is_empty()
            || !self.wires.is_empty()
            || !Self::region_has_nonzero_area(&self.region)
            || !distance.is_finite()
            || !self.region.hole_contours().is_empty()
        {
            return None;
        }

        let options = FiniteProjectionOptions::try_new(1e-3)
            .expect("positive finite projection tolerance is valid");
        let profiles = match self.project_region_profiles(&options) {
            Ok(Classification::Decided(profiles)) => profiles,
            Ok(Classification::Uncertain(_)) | Err(_) => return None,
        };
        if profiles.len() != self.region.material_contours().len() {
            return None;
        }

        let policy = CurvePolicy::certified();
        let mut material = Vec::with_capacity(self.region.material_contours().len());
        for (contour, profile) in self.region.material_contours().iter().zip(profiles.iter()) {
            let signed_area = finite_ring_signed_area(profile.material().points());
            if signed_area.abs() <= f64::EPSILON {
                return None;
            }
            let outward_distance = if signed_area.is_sign_positive() {
                -distance
            } else {
                distance
            };
            let offset_distance = hreal_from_f64(outward_distance).ok()?;
            let offset = match contour.offset_left_checked(offset_distance, &policy).ok()? {
                Classification::Decided(offset) => offset,
                Classification::Uncertain(_) => return None,
            };
            material.push(offset);
        }

        let mut sketch = Sketch::from_region(
            Region2::from_material_contours(material),
            self.metadata.clone(),
        );
        sketch.origin = self.origin;
        sketch.origin_transform = self.origin_transform;
        Some(sketch)
    }

    /// **Mathematical Foundation: Sharp Corner Polygon Offsetting**
    ///
    /// Grows/shrinks/offsets all polygons in the XY plane by `distance` using georust.
    /// This implements the standard polygon offsetting algorithm with sharp corners.
    ///
    /// ## **Algorithm Details**
    ///
    /// ### **Edge Offset Calculation**
    /// For each edge e with unit normal n⃗:
    /// ```text
    /// e'(t) = e(t) + distance × n⃗
    /// ```
    /// where n⃗ is the outward normal perpendicular to the edge.
    ///
    /// ### **Vertex Joint Resolution**
    /// At vertices where two offset edges meet:
    /// 1. **Convex vertices**: Extend edges until intersection
    /// 2. **Concave vertices**: May require clipping or filling
    /// 3. **Collinear edges**: Handle degenerate cases
    ///
    /// ### **Self-Intersection Resolution**
    /// The algorithm automatically:
    /// - **Detects**: Self-intersecting offset curves
    /// - **Resolves**: Using winding number rules
    /// - **Simplifies**: Resulting polygon topology
    ///
    /// ## **Input Processing**
    /// For each Geometry in the collection:
    /// - **Polygon**: Buffer and convert to MultiPolygon
    /// - **MultiPolygon**: Buffer directly preserving holes
    /// - **Other geometries**: Excluded from processing
    ///
    /// ## **Mathematical Properties**
    /// - **Distance Preservation**: All points move exactly `distance` units
    /// - **Topology**: May change due to merging/splitting
    /// - **Orientation**: Preserved for valid input polygons
    /// - **Holes**: Correctly handled with opposite offset direction
    ///
    /// **Note**: Sharp corners may create very acute angles for large offset distances.
    #[allow(clippy::unnecessary_cast)]
    pub fn offset(&self, distance: Real) -> Sketch<M> {
        if let Some(sketch) = self.native_sharp_offset(distance) {
            return sketch;
        }

        let geometry = self.offset_compat_geometry();
        let offset_geoms = geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
                    let style = BufferStyle::new(distance).line_join(LineJoin::Miter(1.0));
                    let new_mpoly = poly.buffer_with_style(style);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::MultiPolygon(mpoly) => {
                    let style = BufferStyle::new(distance).line_join(LineJoin::Miter(1.0));
                    let new_mpoly = mpoly.buffer_with_style(style);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::Point(point) => Some(Geometry::MultiPolygon(point.buffer(distance))),
                _ => None,
            })
            .collect();

        // Construct a new GeometryCollection from the offset geometries, then
        // immediately recompose it into native hypercurve topology. The buffer
        // result is a finite compatibility boundary, not Sketch ownership.
        let new_collection = GeometryCollection::<Real>(offset_geoms);

        let mut sketch = Sketch::from_compat_bridge_geometry_with_origin(
            new_collection,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        );
        self.preserve_offset_wires(&mut sketch);
        sketch
    }

    /// **Mathematical Foundation: Rounded Corner Polygon Offsetting**
    ///
    /// Grows/shrinks/offsets all polygons in the XY plane by `distance` using georust.
    /// This implements rounded corner offsetting for smoother, more natural results.
    ///
    /// ## **Algorithm Details**
    ///
    /// ### **Edge Offset Calculation**
    /// Same as sharp offset: edges move parallel by distance d.
    ///
    /// ### **Rounded Vertex Treatment**
    /// At each vertex, instead of sharp intersection:
    /// 1. **Circular Arc**: Connect offset edges with radius = |distance|
    /// 2. **Arc Center**: Located at original vertex
    /// 3. **Arc Span**: From end of one offset edge to start of next
    /// 4. **Direction**: Outward for positive offset, inward for negative
    ///
    /// ### **Mathematical Formulation**
    /// For vertex V with incoming edge direction d₁ and outgoing direction d₂:
    /// ```text
    /// Arc center: C = V
    /// Arc radius: r = |distance|
    /// Start angle: θ₁ = atan2(d₁⊥)
    /// End angle: θ₂ = atan2(d₂⊥)
    /// Arc points: P(t) = C + r(cos(θ(t)), sin(θ(t)))
    /// ```
    ///
    /// ## **Advantages over Sharp Offset**
    /// - **C¹ Continuity**: Smooth derivative at vertex connections
    /// - **Aesthetic Quality**: More natural, visually pleasing curves
    /// - **Numerical Stability**: Avoids extreme angles and spikes
    /// - **Manufacturing**: Better for toolpath generation (reduces tool stress)
    ///
    /// ## **Applications**
    /// - **Font Rendering**: Smooth outline expansion
    /// - **CNC Machining**: Tool radius compensation
    /// - **Geographic Buffering**: Natural boundary expansion
    /// - **UI Design**: Smooth border effects
    ///
    /// ## **Performance Considerations**
    /// - **Arc Discretization**: More vertices for smoother curves
    /// - **Memory Usage**: Slightly higher than sharp offset
    /// - **Computation**: Additional trigonometric calculations
    ///
    /// Uses rounded corners for each convex vertex.
    /// For each Geometry in the collection:
    /// - **Polygon**: Buffer and convert to MultiPolygon  
    /// - **MultiPolygon**: Buffer directly preserving holes
    /// - **Other geometries**: Excluded from processing
    #[allow(clippy::unnecessary_cast)]
    pub fn offset_rounded(&self, distance: Real) -> Sketch<M> {
        let geometry = self.offset_compat_geometry();
        let offset_geoms = geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
                    let new_mpoly = poly.buffer(distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::MultiPolygon(mpoly) => {
                    let new_mpoly = mpoly.buffer(distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::Point(point) => Some(Geometry::MultiPolygon(point.buffer(distance))),
                _ => None,
            })
            .collect();

        // Construct a new GeometryCollection from the offset geometries, then
        // immediately recompose it into native hypercurve topology. Farouki
        // and Neff characterize the exact offset curve; this finite buffer is
        // only the current approximation boundary.
        let new_collection = GeometryCollection::<Real>(offset_geoms);

        let mut sketch = Sketch::from_compat_bridge_geometry_with_origin(
            new_collection,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        );
        self.preserve_offset_wires(&mut sketch);
        sketch
    }

    /// This function returns a Sketch which represents an instantiated straight skeleton of Sketch upon which it's called.
    /// Each segment of the straight skeleton is represented as a single `LineString`.
    /// If either endpoints of a `LineString` is infinitely far from the other, then this `LineString` will be clipped to one which has shorter length.
    /// The order of these `LineString`s is arbitrary. (There is no guaranteed order on segments of the straight skeleton.)
    ///
    /// Area topology is projected from native `Region2`, skeleton linework is
    /// promoted back into native `CurveString2` wires, and any pre-existing
    /// open wires are preserved as independent path topology. The straight
    /// skeleton construction follows Aichholzer et al., "A Novel Type of
    /// Skeleton for Polygons," *Journal of Universal Computer Science* 1(12),
    /// 1995 (<https://doi.org/10.3217/jucs-001-12-0752>), while the
    /// hyperreal/finite boundary follows Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    ///
    /// # Arguments
    ///
    /// + `orientation`: determines the region where the straight skeleton created. The value of this `boolean` variable will be:
    ///     * `true` to create the straight skeleton on the inward region of the polygon, and,
    ///     * `false` to create on the outward region of the polygon.
    pub fn straight_skeleton(&self, orientation: bool) -> Sketch<M> {
        let geometry = self.offset_compat_geometry();
        let skeleton = geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
                    let mls = geo::MultiLineString(skel_poly(poly, orientation));
                    Some(Geometry::MultiLineString(mls))
                },
                Geometry::MultiPolygon(mpoly) => {
                    let mls = geo::MultiLineString(skel_multi_poly(mpoly, orientation));
                    Some(Geometry::MultiLineString(mls))
                },
                _ => None,
            })
            .collect();

        // Construct a new GeometryCollection from skeleton linework, then
        // promote it to native open `CurveString2` wires.
        let new_collection = GeometryCollection::<Real>(skeleton);

        let mut sketch = Sketch::from_compat_bridge_geometry_with_origin(
            new_collection,
            self.metadata.clone(),
            self.origin,
            self.origin_transform,
        );
        self.preserve_offset_wires(&mut sketch);
        sketch
    }
}
