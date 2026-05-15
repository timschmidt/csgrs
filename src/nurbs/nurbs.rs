//! NURBS-backed planar geometry.
//!
//! This module wraps Curvo's planar NURBS regions in a csgrs backend type. It is
//! intentionally core-first: wasm bindings can sit on top of this API instead of
//! owning NURBS behavior themselves.

use std::fmt::Debug;
use std::sync::OnceLock;

use curvo::prelude::operation::BooleanOperation;
use curvo::prelude::{
    Boolean, CompoundCurve2D, CurveIntersectionSolverOptions, Invertible, NurbsCurve2D,
    Region, Tessellation, Transformable,
};
use geo::{Geometry, GeometryCollection, LineString, MultiPolygon, Polygon as GeoPolygon};
use nalgebra::{Matrix3, Matrix4, Point2, Point3, Vector2, Vector3};

use crate::csg::CSG;
use crate::float_types::{Real, parry3d::bounding_volume::Aabb};
#[cfg(feature = "mesh")]
use crate::mesh::Mesh;
#[cfg(feature = "sketch")]
use crate::sketch::Sketch;

/// Boolean operation failures from the Curvo backend.
pub type NurbsResult<T> = Result<T, String>;

/// A planar NURBS geometry backend.
///
/// Each value can contain multiple disjoint planar regions. Each region has one
/// exterior compound curve and zero or more interior compound-curve holes.
#[derive(Clone, Debug)]
pub struct Nurbs<M: Clone + Send + Sync + Debug = ()> {
    regions: Vec<Region<Real>>,
    bounding_box: OnceLock<Aabb>,
    pub metadata: M,
}

impl<M: Clone + Send + Sync + Debug> Nurbs<M> {
    /// Create an empty NURBS geometry.
    pub fn empty(metadata: M) -> Self {
        Self {
            regions: Vec::new(),
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Create from Curvo regions.
    pub fn from_regions(regions: Vec<Region<Real>>, metadata: M) -> Self {
        Self {
            regions,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Return this NURBS geometry with replacement metadata.
    pub fn with_metadata<NewM: Clone + Send + Sync + Debug>(
        self,
        metadata: NewM,
    ) -> Nurbs<NewM> {
        Nurbs {
            regions: self.regions,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }

    /// Map this NURBS geometry's metadata while preserving its regions.
    pub fn map_metadata<NewM: Clone + Send + Sync + Debug, F>(self, f: F) -> Nurbs<NewM>
    where
        F: FnOnce(M) -> NewM,
    {
        Nurbs {
            regions: self.regions,
            bounding_box: OnceLock::new(),
            metadata: f(self.metadata),
        }
    }

    /// Borrow the underlying Curvo regions.
    pub fn regions(&self) -> &[Region<Real>] {
        &self.regions
    }

    /// Consume the backend and return its Curvo regions.
    pub fn into_regions(self) -> Vec<Region<Real>> {
        self.regions
    }

    /// Create from a single exterior compound curve and holes.
    pub fn from_compound(
        exterior: CompoundCurve2D<Real>,
        holes: Vec<CompoundCurve2D<Real>>,
        metadata: M,
    ) -> Self {
        Self::from_regions(vec![Region::new(exterior, holes)], metadata)
    }

    /// Create from a single closed curve.
    pub fn from_curve(curve: NurbsCurve2D<Real>, metadata: M) -> Self {
        Self::from_compound(curve.into(), Vec::new(), metadata)
    }

    /// Create from a polyline. The point list should be closed for region
    /// booleans; this method closes it if needed.
    pub fn polyline(points: &[Point2<Real>], metadata: M) -> NurbsResult<Self> {
        if points.len() < 3 {
            return Err("NURBS polyline needs at least three points".to_string());
        }

        let mut closed = points.to_vec();
        let first = closed[0];
        let last = *closed.last().unwrap();
        if (last - first).norm() > crate::float_types::tolerance() {
            closed.push(first);
        }

        Ok(Self::from_curve(
            NurbsCurve2D::polyline(&closed, true),
            metadata,
        ))
    }

    /// Axis-aligned rectangle centered at the origin.
    pub fn rectangle(width: Real, height: Real, metadata: M) -> Self {
        let hw = width * 0.5;
        let hh = height * 0.5;
        let points = [
            Point2::new(-hw, -hh),
            Point2::new(hw, -hh),
            Point2::new(hw, hh),
            Point2::new(-hw, hh),
            Point2::new(-hw, -hh),
        ];
        Self::from_curve(NurbsCurve2D::polyline(&points, true), metadata)
    }

    /// Exact NURBS circle centered at the origin.
    pub fn circle(radius: Real, metadata: M) -> NurbsResult<Self> {
        let curve =
            NurbsCurve2D::try_circle(&Point2::origin(), &Vector2::x(), &Vector2::y(), radius)
                .map_err(|e| format!("Failed to create NURBS circle: {e:?}"))?;
        Ok(Self::from_curve(curve, metadata))
    }

    /// Tessellate all region boundaries.
    ///
    /// Returns `(exterior, holes)` point rings per region. Rings are closed when
    /// Curvo returns closed tessellation.
    pub fn tessellate_regions(
        &self,
        tolerance: Option<Real>,
    ) -> Vec<(Vec<Point2<Real>>, Vec<Vec<Point2<Real>>>)> {
        self.regions
            .iter()
            .map(|region| {
                let exterior = region.exterior().tessellate(tolerance);
                let holes = region
                    .interiors()
                    .iter()
                    .map(|hole| hole.tessellate(tolerance))
                    .collect();
                (exterior, holes)
            })
            .collect()
    }

    /// Convert the NURBS regions into geo polygons via tessellation.
    pub fn to_multipolygon(&self, tolerance: Option<Real>) -> MultiPolygon<Real> {
        MultiPolygon(
            self.tessellate_regions(tolerance)
                .into_iter()
                .filter_map(|(exterior, holes)| {
                    if exterior.len() < 4 {
                        return None;
                    }
                    let exterior = LineString::from(
                        exterior.into_iter().map(|p| (p.x, p.y)).collect::<Vec<_>>(),
                    );
                    let holes = holes
                        .into_iter()
                        .filter(|ring| ring.len() >= 4)
                        .map(|ring| {
                            LineString::from(
                                ring.into_iter().map(|p| (p.x, p.y)).collect::<Vec<_>>(),
                            )
                        })
                        .collect();
                    Some(GeoPolygon::new(exterior, holes))
                })
                .collect(),
        )
    }

    /// Convert into a `Sketch` by tessellating the NURBS boundaries.
    #[cfg(feature = "sketch")]
    pub fn to_sketch(&self, tolerance: Option<Real>) -> Sketch<M> {
        let geometry = self
            .to_multipolygon(tolerance)
            .0
            .into_iter()
            .map(Geometry::Polygon)
            .collect();
        Sketch::from_geo(GeometryCollection(geometry), self.metadata.clone())
    }

    /// Convert into a `Mesh` by tessellating to a sketch and extruding.
    #[cfg(all(feature = "sketch", feature = "mesh"))]
    pub fn extrude_vector(
        &self,
        direction: Vector3<Real>,
        tolerance: Option<Real>,
    ) -> Mesh<M> {
        self.to_sketch(tolerance).extrude_vector(direction)
    }

    fn boolean_regions(
        &self,
        operation: BooleanOperation,
        other: &Self,
    ) -> NurbsResult<Vec<Region<Real>>> {
        if self.regions.is_empty() {
            return Ok(match operation {
                BooleanOperation::Union => other.regions.clone(),
                BooleanOperation::Intersection | BooleanOperation::Difference => Vec::new(),
            });
        }
        if other.regions.is_empty() {
            return Ok(match operation {
                BooleanOperation::Union | BooleanOperation::Difference => self.regions.clone(),
                BooleanOperation::Intersection => Vec::new(),
            });
        }

        match operation {
            BooleanOperation::Union => {
                let mut acc = self.regions.clone();
                for rhs in &other.regions {
                    let mut next = Vec::new();
                    let mut pending = vec![rhs.clone()];
                    for lhs in acc {
                        let mut merged_any = false;
                        let mut remaining = Vec::new();
                        for candidate in pending {
                            match lhs.boolean(
                                BooleanOperation::Union,
                                &candidate,
                                Some(default_solver_options()),
                            ) {
                                Ok(clip) => {
                                    let regions = clip.into_regions();
                                    if regions.len() == 1 {
                                        next.extend(regions);
                                        merged_any = true;
                                    } else {
                                        next.push(lhs.clone());
                                        remaining.extend(regions);
                                    }
                                },
                                Err(_) => {
                                    remaining.push(candidate);
                                    next.push(lhs.clone());
                                },
                            }
                        }
                        if !merged_any && pending_is_empty(&remaining) {
                            next.push(lhs);
                        }
                        pending = remaining;
                    }
                    next.extend(pending);
                    acc = next;
                }
                Ok(acc)
            },
            BooleanOperation::Intersection => {
                let mut out = Vec::new();
                for lhs in &self.regions {
                    for rhs in &other.regions {
                        let clip = lhs
                            .boolean(
                                BooleanOperation::Intersection,
                                rhs,
                                Some(default_solver_options()),
                            )
                            .map_err(|e| format!("NURBS intersection failed: {e:?}"))?;
                        out.extend(clip.into_regions());
                    }
                }
                Ok(out)
            },
            BooleanOperation::Difference => {
                let mut current = self.regions.clone();
                for rhs in &other.regions {
                    let mut next = Vec::new();
                    for lhs in &current {
                        let clip = lhs
                            .boolean(
                                BooleanOperation::Difference,
                                rhs,
                                Some(default_solver_options()),
                            )
                            .map_err(|e| format!("NURBS difference failed: {e:?}"))?;
                        next.extend(clip.into_regions());
                    }
                    current = next;
                }
                Ok(current)
            },
        }
    }

    /// Fallible union that preserves Curvo errors.
    pub fn try_union(&self, other: &Self) -> NurbsResult<Self> {
        Ok(Self::from_regions(
            self.boolean_regions(BooleanOperation::Union, other)?,
            self.metadata.clone(),
        ))
    }

    /// Fallible difference that preserves Curvo errors.
    pub fn try_difference(&self, other: &Self) -> NurbsResult<Self> {
        Ok(Self::from_regions(
            self.boolean_regions(BooleanOperation::Difference, other)?,
            self.metadata.clone(),
        ))
    }

    /// Fallible intersection that preserves Curvo errors.
    pub fn try_intersection(&self, other: &Self) -> NurbsResult<Self> {
        Ok(Self::from_regions(
            self.boolean_regions(BooleanOperation::Intersection, other)?,
            self.metadata.clone(),
        ))
    }
}

impl Nurbs<()> {
    /// Create an empty NURBS geometry with unit metadata.
    pub fn new() -> Self {
        Self::empty(())
    }
}

impl<M: Clone + Send + Sync + Debug> CSG for Nurbs<M> {
    fn union(&self, other: &Self) -> Self {
        self.try_union(other)
            .unwrap_or_else(|e| panic!("NURBS union failed: {e}"))
    }

    fn difference(&self, other: &Self) -> Self {
        self.try_difference(other)
            .unwrap_or_else(|e| panic!("NURBS difference failed: {e}"))
    }

    fn intersection(&self, other: &Self) -> Self {
        self.try_intersection(other)
            .unwrap_or_else(|e| panic!("NURBS intersection failed: {e}"))
    }

    fn xor(&self, other: &Self) -> Self {
        self.difference(other).union(&other.difference(self))
    }

    fn transform(&self, matrix: &Matrix4<Real>) -> Self {
        let affine_2d = Matrix3::new(
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 3)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 3)],
            Real::from(0.0),
            Real::from(0.0),
            Real::from(1.0),
        );

        let mut regions = self.regions.clone();
        for region in &mut regions {
            region.transform(&affine_2d);
        }
        Self::from_regions(regions, self.metadata.clone())
    }

    fn inverse(&self) -> Self {
        let mut regions = self.regions.clone();
        for region in &mut regions {
            region.invert();
        }
        Self::from_regions(regions, self.metadata.clone())
    }

    fn bounding_box(&self) -> Aabb {
        *self.bounding_box.get_or_init(|| {
            let mut min_x = Real::MAX;
            let mut min_y = Real::MAX;
            let mut max_x = -Real::MAX;
            let mut max_y = -Real::MAX;

            for (exterior, holes) in
                self.tessellate_regions(Some(crate::float_types::tolerance()))
            {
                for p in exterior.into_iter().chain(holes.into_iter().flatten()) {
                    min_x = min_x.min(p.x);
                    min_y = min_y.min(p.y);
                    max_x = max_x.max(p.x);
                    max_y = max_y.max(p.y);
                }
            }

            if min_x == Real::MAX {
                Aabb::new(Point3::origin(), Point3::origin())
            } else {
                Aabb::new(Point3::new(min_x, min_y, 0.0), Point3::new(max_x, max_y, 0.0))
            }
        })
    }

    fn invalidate_bounding_box(&mut self) {
        self.bounding_box = OnceLock::new();
    }
}

fn pending_is_empty<T>(values: &[T]) -> bool {
    values.is_empty()
}

fn default_solver_options() -> CurveIntersectionSolverOptions<Real> {
    CurveIntersectionSolverOptions {
        minimum_distance: Real::from(1e-4),
        knot_domain_division: 500,
        max_iters: 1000,
        step_size_tolerance: Real::from(1e-8),
        cost_tolerance: Real::from(1e-10),
    }
}
