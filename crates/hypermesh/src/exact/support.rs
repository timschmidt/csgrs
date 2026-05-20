//! Exact support-vertex k-DOP bounds.
//!
//! A discrete oriented polytope is a broad-phase object summary: each retained
//! axis stores the exact minimum and maximum support distances together with
//! the vertices that witnessed them. The distances deliberately use integer,
//! unnormalized directions so no square roots or primitive floats enter the
//! exact core. This follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): retain geometric-object structure
//! for scheduling and arithmetic-package selection, but leave topology changes
//! to certified predicates.

use std::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};
use hyperreal::Real;

use super::mesh::ExactMesh;
use super::provenance::MeshSource;

/// Exact integer direction used by a support slab.
///
/// The vector is intentionally not normalized. k-DOP comparisons only require
/// a consistent linear functional, and keeping integer directions preserves the
/// common-scale structure Yap identifies as useful for exact geometric object
/// packages. See Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportDopAxis3 {
    /// Integer coefficients of the support direction.
    pub direction: [i64; 3],
}

impl SupportDopAxis3 {
    /// Construct a support axis from exact integer coefficients.
    pub const fn new(direction: [i64; 3]) -> Self {
        Self { direction }
    }

    /// Return whether this axis has at least one nonzero coefficient.
    pub const fn is_nonzero(self) -> bool {
        self.direction[0] != 0 || self.direction[1] != 0 || self.direction[2] != 0
    }

    /// Validate that the direction can define a support functional.
    pub fn validate(self) -> Result<(), SupportDopValidationError> {
        if self.is_nonzero() {
            Ok(())
        } else {
            Err(SupportDopValidationError::ZeroAxis)
        }
    }

    /// Three axes that produce the same six planes as an exact AABB.
    pub const fn orthogonal_axes() -> [Self; 3] {
        [
            Self::new([1, 0, 0]),
            Self::new([0, 1, 0]),
            Self::new([0, 0, 1]),
        ]
    }

    /// Thirteen axes for a 26-DOP over axis, face-diagonal, and body-diagonal
    /// support directions.
    pub const fn kdop26_axes() -> [Self; 13] {
        [
            Self::new([1, 0, 0]),
            Self::new([0, 1, 0]),
            Self::new([0, 0, 1]),
            Self::new([1, 1, 0]),
            Self::new([1, -1, 0]),
            Self::new([1, 0, 1]),
            Self::new([1, 0, -1]),
            Self::new([0, 1, 1]),
            Self::new([0, 1, -1]),
            Self::new([1, 1, 1]),
            Self::new([1, 1, -1]),
            Self::new([1, -1, 1]),
            Self::new([-1, 1, 1]),
        ]
    }
}

/// Why a support-DOP consumer must conservatively expand the exact slabs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopExpansionKind {
    /// No expansion is needed; every slab is an exact summary of exact source
    /// coordinates.
    None,
    /// The source entered through a primitive-float or external adapter edge.
    LossyAdapter,
    /// Coordinates were rounded to an integer grid before support extraction.
    IntegerGridRounding,
}

/// Conservative-expansion metadata attached to support-DOP bounds.
///
/// Expansion reports are kept separate from support witnesses: the witness
/// distances remain exact object facts, while consumers that need conservative
/// broad-phase rejection can apply the expansion to each slab. This preserves
/// Yap's exact-object/adapter-boundary split from "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDopExpansionReport {
    /// Expansion source category.
    pub kind: SupportDopExpansionKind,
    /// Number of axes covered by this report.
    pub axis_count: usize,
    /// Number of slabs that must be treated as conservatively expanded.
    pub expanded_slabs: usize,
    /// Nonnegative distance expansion applied on both sides of each expanded
    /// slab.
    pub expansion: Real,
}

impl SupportDopExpansionReport {
    /// Build an exact no-expansion report for `axis_count` slabs.
    pub fn exact(axis_count: usize) -> Self {
        Self {
            kind: SupportDopExpansionKind::None,
            axis_count,
            expanded_slabs: 0,
            expansion: Real::from(0),
        }
    }

    /// Build the default report implied by mesh construction provenance.
    ///
    /// Exact input does not expand. Primitive-float and external adapter input
    /// keep an explicit adapter report even when the imported dyadic values are
    /// later exact, so downstream broad-phase code cannot forget the edge where
    /// approximation entered the system.
    pub fn for_mesh_source(source: MeshSource, axis_count: usize) -> Self {
        match source {
            MeshSource::Exact => Self::exact(axis_count),
            MeshSource::LossyF64
            | MeshSource::LegacyBoolmeshAdapter
            | MeshSource::ExternalAdapter => Self {
                kind: SupportDopExpansionKind::LossyAdapter,
                axis_count,
                expanded_slabs: axis_count,
                expansion: Real::from(0),
            },
        }
    }

    /// Build an integer-grid rounding expansion report.
    pub fn integer_grid_rounding(axis_count: usize, expansion: Real) -> Self {
        Self {
            kind: SupportDopExpansionKind::IntegerGridRounding,
            axis_count,
            expanded_slabs: axis_count,
            expansion,
        }
    }

    /// Validate internal report consistency.
    pub fn validate(&self) -> Result<(), SupportDopValidationError> {
        if !matches!(
            compare(&self.expansion, &Real::from(0)),
            Some(Ordering::Greater | Ordering::Equal)
        ) {
            return Err(SupportDopValidationError::NegativeExpansion);
        }
        match self.kind {
            SupportDopExpansionKind::None => {
                if self.expanded_slabs != 0
                    || !matches!(
                        compare(&self.expansion, &Real::from(0)),
                        Some(Ordering::Equal)
                    )
                {
                    return Err(SupportDopValidationError::ExpansionKindMismatch);
                }
            }
            SupportDopExpansionKind::LossyAdapter
            | SupportDopExpansionKind::IntegerGridRounding => {
                if self.expanded_slabs != self.axis_count {
                    return Err(SupportDopValidationError::ExpansionKindMismatch);
                }
            }
        }
        Ok(())
    }
}

/// Exact support witness for one side of one k-DOP slab.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportWitness3 {
    /// Vertex index that witnesses the retained support distance.
    pub vertex: usize,
    /// Exact point copied from the source mesh at the time of construction.
    pub point: Point3,
    /// Exact unnormalized support distance along the slab axis.
    pub distance: Real,
}

/// One min/max support slab of a k-DOP.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportSlab3 {
    /// Direction used by this support functional.
    pub axis: SupportDopAxis3,
    /// Minimum support witness.
    pub min: SupportWitness3,
    /// Maximum support witness.
    pub max: SupportWitness3,
}

impl SupportSlab3 {
    /// Return the conservative minimum distance after applying the expansion
    /// report carried by the containing k-DOP.
    pub fn conservative_min_distance(&self, expansion: &SupportDopExpansionReport) -> Real {
        self.min.distance.clone() - expansion.expansion.clone()
    }

    /// Return the conservative maximum distance after applying the expansion
    /// report carried by the containing k-DOP.
    pub fn conservative_max_distance(&self, expansion: &SupportDopExpansionReport) -> Real {
        self.max.distance.clone() + expansion.expansion.clone()
    }
}

/// Exact k-DOP bounds with support-vertex witnesses.
#[derive(Clone, Debug, PartialEq)]
pub struct SupportDop3 {
    /// Number of points summarized by this object.
    pub vertex_count: usize,
    /// One min/max support slab per retained axis.
    pub slabs: Vec<SupportSlab3>,
    /// Conservative adapter/rounding expansion metadata.
    pub expansion: SupportDopExpansionReport,
}

impl SupportDop3 {
    /// Build a support-DOP from exact points and an explicit expansion report.
    pub fn from_points_with_expansion(
        points: &[Point3],
        axes: &[SupportDopAxis3],
        expansion: SupportDopExpansionReport,
    ) -> Result<Self, SupportDopValidationError> {
        if points.is_empty() {
            return Err(SupportDopValidationError::EmptyPointSet);
        }
        if axes.is_empty() {
            return Err(SupportDopValidationError::EmptyAxisSet);
        }
        if expansion.axis_count != axes.len() {
            return Err(SupportDopValidationError::ExpansionAxisCountMismatch);
        }
        expansion.validate()?;
        let slabs = axes
            .iter()
            .map(|&axis| compute_slab(points, axis))
            .collect::<Result<Vec<_>, _>>()?;
        let support = Self {
            vertex_count: points.len(),
            slabs,
            expansion,
        };
        support.validate_against_points(points)?;
        Ok(support)
    }

    /// Build an exact no-expansion support-DOP from points.
    pub fn from_points(
        points: &[Point3],
        axes: &[SupportDopAxis3],
    ) -> Result<Self, SupportDopValidationError> {
        Self::from_points_with_expansion(points, axes, SupportDopExpansionReport::exact(axes.len()))
    }

    /// Build a support-DOP from an [`ExactMesh`] and retain the source-derived
    /// adapter expansion report.
    pub fn from_mesh(
        mesh: &ExactMesh,
        axes: &[SupportDopAxis3],
    ) -> Result<Self, SupportDopValidationError> {
        let points = mesh
            .vertices()
            .iter()
            .map(|point| point.to_hyperlimit_point())
            .collect::<Vec<_>>();
        let expansion =
            SupportDopExpansionReport::for_mesh_source(mesh.provenance().source.source, axes.len());
        Self::from_points_with_expansion(&points, axes, expansion)
    }

    /// Validate this k-DOP against exact source points.
    ///
    /// Validation replays every witness and scans all source points to ensure
    /// no point lies outside the retained support slab. This makes the object
    /// a replayable broad-phase fact rather than a stale cache entry.
    pub fn validate_against_points(
        &self,
        points: &[Point3],
    ) -> Result<(), SupportDopValidationError> {
        if points.is_empty() {
            return Err(SupportDopValidationError::EmptyPointSet);
        }
        if self.slabs.is_empty() {
            return Err(SupportDopValidationError::EmptyAxisSet);
        }
        if self.vertex_count != points.len() {
            return Err(SupportDopValidationError::VertexCountMismatch);
        }
        if self.expansion.axis_count != self.slabs.len() {
            return Err(SupportDopValidationError::ExpansionAxisCountMismatch);
        }
        self.expansion.validate()?;
        for slab in &self.slabs {
            validate_slab(slab, points)?;
        }
        Ok(())
    }

    /// Validate this k-DOP against the current exact mesh.
    pub fn validate_against_mesh(&self, mesh: &ExactMesh) -> Result<(), SupportDopValidationError> {
        let points = mesh
            .vertices()
            .iter()
            .map(|point| point.to_hyperlimit_point())
            .collect::<Vec<_>>();
        self.validate_against_points(&points)
    }

    /// Refresh slabs after a bounded set of point updates.
    ///
    /// Non-witness point updates can only extend a slab, so those axes update
    /// in place. If an updated point was a retained min or max witness, the
    /// axis is rebuilt from all points. This is the k-DOP support-witness rule
    /// called out in the porting plan: update only axes whose witnesses were
    /// invalidated, while still validating the final object by exact replay.
    pub fn refresh_for_changed_vertices(
        &mut self,
        points: &[Point3],
        changed_vertices: &[usize],
    ) -> Result<SupportDopRefreshReport, SupportDopValidationError> {
        if self.vertex_count != points.len() {
            return Err(SupportDopValidationError::VertexCountMismatch);
        }
        if self.expansion.axis_count != self.slabs.len() {
            return Err(SupportDopValidationError::ExpansionAxisCountMismatch);
        }
        for &vertex in changed_vertices {
            if vertex >= points.len() {
                return Err(SupportDopValidationError::ChangedVertexOutOfRange);
            }
        }

        let mut report = SupportDopRefreshReport {
            changed_vertices: changed_vertices.len(),
            axis_count: self.slabs.len(),
            axes_rebuilt: 0,
            axes_extended: 0,
            axes_unchanged: 0,
            invalidated_witness_axes: 0,
        };

        for slab in &mut self.slabs {
            let witness_invalidated = changed_vertices
                .iter()
                .any(|&vertex| vertex == slab.min.vertex || vertex == slab.max.vertex);
            if witness_invalidated {
                *slab = compute_slab(points, slab.axis)?;
                report.axes_rebuilt += 1;
                report.invalidated_witness_axes += 1;
                continue;
            }

            let mut extended = false;
            for &vertex in changed_vertices {
                let distance = support_distance(&points[vertex], slab.axis);
                if matches!(compare(&distance, &slab.min.distance), Some(Ordering::Less)) {
                    slab.min = witness(vertex, &points[vertex], distance.clone());
                    extended = true;
                }
                if matches!(
                    compare(&distance, &slab.max.distance),
                    Some(Ordering::Greater)
                ) {
                    slab.max = witness(vertex, &points[vertex], distance);
                    extended = true;
                }
            }

            if extended {
                report.axes_extended += 1;
            } else {
                report.axes_unchanged += 1;
            }
        }

        self.validate_against_points(points)?;
        Ok(report)
    }
}

/// Refresh summary returned by [`SupportDop3::refresh_for_changed_vertices`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SupportDopRefreshReport {
    /// Number of changed vertex indices supplied by the caller.
    pub changed_vertices: usize,
    /// Number of support axes retained by the k-DOP.
    pub axis_count: usize,
    /// Axes fully rebuilt because a min or max witness changed.
    pub axes_rebuilt: usize,
    /// Axes updated in place because a changed non-witness became more
    /// extreme.
    pub axes_extended: usize,
    /// Axes untouched after evaluating changed non-witness points.
    pub axes_unchanged: usize,
    /// Axes whose retained support witness was explicitly invalidated.
    pub invalidated_witness_axes: usize,
}

/// Error returned by exact support-DOP construction or replay.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SupportDopValidationError {
    /// No source points were supplied.
    EmptyPointSet,
    /// No support axes were supplied.
    EmptyAxisSet,
    /// An axis direction was zero.
    ZeroAxis,
    /// Retained vertex count disagrees with the source point count.
    VertexCountMismatch,
    /// Expansion report axis count disagrees with the slab count.
    ExpansionAxisCountMismatch,
    /// A support witness points outside the source vertex slice.
    WitnessOutOfRange,
    /// A retained witness point no longer matches the source point.
    WitnessPointMismatch,
    /// A retained witness distance no longer equals the axis dot product.
    WitnessDistanceMismatch,
    /// A min witness is not minimal for its axis.
    WitnessNotMinimal,
    /// A max witness is not maximal for its axis.
    WitnessNotMaximal,
    /// A min/max ordering could not be certified for a slab.
    UnknownSlabOrder,
    /// Expansion distance is negative or not certifiably nonnegative.
    NegativeExpansion,
    /// Expansion kind, slab count, or zero-expansion state is contradictory.
    ExpansionKindMismatch,
    /// A changed vertex index is outside the source point slice.
    ChangedVertexOutOfRange,
}

/// Build support-DOP bounds for an exact mesh.
pub fn support_dop_for_mesh(
    mesh: &ExactMesh,
    axes: &[SupportDopAxis3],
) -> Result<SupportDop3, SupportDopValidationError> {
    SupportDop3::from_mesh(mesh, axes)
}

fn compute_slab(
    points: &[Point3],
    axis: SupportDopAxis3,
) -> Result<SupportSlab3, SupportDopValidationError> {
    axis.validate()?;
    let first = points
        .first()
        .ok_or(SupportDopValidationError::EmptyPointSet)?;
    let first_distance = support_distance(first, axis);
    let mut min = witness(0, first, first_distance.clone());
    let mut max = witness(0, first, first_distance);
    for (vertex, point) in points.iter().enumerate().skip(1) {
        let distance = support_distance(point, axis);
        if matches!(compare(&distance, &min.distance), Some(Ordering::Less)) {
            min = witness(vertex, point, distance.clone());
        }
        if matches!(compare(&distance, &max.distance), Some(Ordering::Greater)) {
            max = witness(vertex, point, distance);
        }
    }
    Ok(SupportSlab3 { axis, min, max })
}

fn validate_slab(slab: &SupportSlab3, points: &[Point3]) -> Result<(), SupportDopValidationError> {
    slab.axis.validate()?;
    validate_witness(&slab.min, slab.axis, points)?;
    validate_witness(&slab.max, slab.axis, points)?;
    match compare(&slab.min.distance, &slab.max.distance) {
        Some(Ordering::Less | Ordering::Equal) => {}
        Some(Ordering::Greater) => return Err(SupportDopValidationError::WitnessNotMinimal),
        None => return Err(SupportDopValidationError::UnknownSlabOrder),
    }
    for point in points {
        let distance = support_distance(point, slab.axis);
        match compare(&distance, &slab.min.distance) {
            Some(Ordering::Less) => return Err(SupportDopValidationError::WitnessNotMinimal),
            Some(Ordering::Equal | Ordering::Greater) => {}
            None => return Err(SupportDopValidationError::UnknownSlabOrder),
        }
        match compare(&distance, &slab.max.distance) {
            Some(Ordering::Greater) => return Err(SupportDopValidationError::WitnessNotMaximal),
            Some(Ordering::Less | Ordering::Equal) => {}
            None => return Err(SupportDopValidationError::UnknownSlabOrder),
        }
    }
    Ok(())
}

fn validate_witness(
    retained: &SupportWitness3,
    axis: SupportDopAxis3,
    points: &[Point3],
) -> Result<(), SupportDopValidationError> {
    let point = points
        .get(retained.vertex)
        .ok_or(SupportDopValidationError::WitnessOutOfRange)?;
    if point != &retained.point {
        return Err(SupportDopValidationError::WitnessPointMismatch);
    }
    let replay_distance = support_distance(point, axis);
    if !matches!(
        compare(&replay_distance, &retained.distance),
        Some(Ordering::Equal)
    ) {
        return Err(SupportDopValidationError::WitnessDistanceMismatch);
    }
    Ok(())
}

fn witness(vertex: usize, point: &Point3, distance: Real) -> SupportWitness3 {
    SupportWitness3 {
        vertex,
        point: point.clone(),
        distance,
    }
}

fn support_distance(point: &Point3, axis: SupportDopAxis3) -> Real {
    point.x.clone() * Real::from(axis.direction[0])
        + point.y.clone() * Real::from(axis.direction[1])
        + point.z.clone() * Real::from(axis.direction[2])
}

fn compare(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals(left, right).value()
}
