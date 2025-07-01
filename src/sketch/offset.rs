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
//! This implementation uses the `geo-buf` crate which provides:
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
use crate::float_types::Real;
use crate::sketch::Sketch;
use geo::{Geometry, GeometryCollection, MultiPolygon};
use geo_buf::{
    buffer_multi_polygon, buffer_multi_polygon_rounded, buffer_polygon, buffer_polygon_rounded, buffer_point,
    skeleton_of_polygon_to_linestring, skeleton_of_multi_polygon_to_linestring,
};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
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
    pub fn offset(&self, distance: Real) -> Sketch<S> {
        let offset_geoms = self
            .geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
                    let new_mpoly = buffer_polygon(poly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::MultiPolygon(mpoly) => {
                    let new_mpoly = buffer_multi_polygon(mpoly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::Point(point) => {
                    let new_poly = buffer_point(point, distance, 64); // todo: avoid hard coding resolution somehow
                    let new_mpoly = MultiPolygon::new(vec![new_poly]);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                _ => None, // no support for offsetting Lines or LineStrings in geo-buf yet
            })
            .collect();

        // Construct a new GeometryCollection from the offset geometries
        let new_collection = GeometryCollection::<Real>(offset_geoms);

        // Return a new CSG using the offset geometry collection and the old polygons/metadata
        Sketch {
            geometry: new_collection,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
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
    pub fn offset_rounded(&self, distance: Real) -> Sketch<S> {
        let offset_geoms = self
            .geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
                    let new_mpoly = buffer_polygon_rounded(poly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::MultiPolygon(mpoly) => {
                    let new_mpoly = buffer_multi_polygon_rounded(mpoly, distance);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                Geometry::Point(point) => {
                    let new_poly = buffer_point(point, distance, 64); // todo: avoid hard coding resolution somehow
                    let new_mpoly = MultiPolygon::new(vec![new_poly]);
                    Some(Geometry::MultiPolygon(new_mpoly))
                },
                _ => None, // no support for offsetting Lines or LineStrings in geo-buf yet
            })
            .collect();

        // Construct a new GeometryCollection from the offset geometries
        let new_collection = GeometryCollection::<Real>(offset_geoms);

        // Return a new Sketch using the offset geometry collection and the old polygons/metadata
        Sketch {
            geometry: new_collection,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
    }
    
    /// This function returns a Sketch which represents an instantiated straight skeleton of Sketch upon which it's called.
	/// Each segment of the straight skeleton is represented as a single `LineString`.
	/// If either endpoints of a `LineString` is infinitely far from the other, then this `LineString` will be clipped to one which has shorter length.
	/// The order of these `LineString`s is arbitrary. (There is no gauranteed order on segments of the straight skeleton.)
	///
	/// # Arguments
	///
	/// + `orientation`: determines the region where the straight skeleton created. The value of this `boolean` variable will be:
	///     * `true` to create the staright skeleton on the inward region of the polygon, and,
	///     * `false` to create on the outward region of the polygon.
    pub fn straight_skeleton(&self, orientation: bool) -> Sketch<S> {
		let skeleton = self
            .geometry
            .iter()
            .filter_map(|geom| match geom {
                Geometry::Polygon(poly) => {
					let mls = geo::MultiLineString(skeleton_of_polygon_to_linestring(poly, orientation));
					Some(Geometry::MultiLineString(mls))
				}
                Geometry::MultiPolygon(mpoly) => {
                    let mls = geo::MultiLineString(skeleton_of_multi_polygon_to_linestring(mpoly, orientation));
                    Some(Geometry::MultiLineString(mls))
                },
                _ => None, // ignore other geometry types
            })
            .collect();

        // Construct a new GeometryCollection from the offset geometries
        let new_collection = GeometryCollection::<Real>(skeleton);

        // Return a new Sketch using the offset geometry collection and the old polygons/metadata
        Sketch {
            geometry: new_collection,
            bounding_box: OnceLock::new(),
            metadata: self.metadata.clone(),
        }
	}
}
