//! Regularized Minkowski sums of bounded nonzero-winding solids.
//!
//! General inputs are decomposed by their supporting-plane arrangement. Each
//! full-dimensional convex cell has constant source winding. Pairwise sums of
//! the retained cells are convex hulls; accumulating their union uses the native
//! Boolean arrangement at each step. A closed convex source can bypass subdivision after certification.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicBool, Ordering};

use hyperlattice::HomogeneousLine3;

use crate::context::DecisionContext;
use crate::convex_hull::convex_hull_impl;
use crate::geometry::affine_projective_point_decision;
use crate::predicate::classify_point_decision;
use crate::winding::mesh_winding_number_decision;
use crate::{
    BooleanOp, BooleanProgram, Classification, HypermeshError, HypermeshResult, MeshContext,
    MeshOutcome, Plane, Point3, Real, Triangle, TriangleMesh, boolean, certify_convex_mesh,
};

/// Optional collection limits and cooperative cancellation for a solid sum.
///
/// Defaults impose no additional count limit. These limits are not bounds on
/// total work, retained heap bytes, scalar expression growth or elapsed time.
/// Cancellation is checked between construction steps and before publication;
/// validation, hull and Boolean calls cannot be interrupted internally.
#[derive(Clone, Copy, Debug)]
pub struct MinkowskiOptions<'a> {
    /// Maximum live decomposition cells for either operand, including exterior
    /// cells before winding selection. Convex operands require one cell.
    pub max_cells: usize,
    /// Maximum convex pair sums accumulated into the completed union.
    pub max_pair_sums: usize,
    /// Maximum generated vertex sums in any one convex pair.
    pub max_pair_points: usize,
    /// An optional flag set to `true` to request cooperative cancellation.
    pub cancellation: Option<&'a AtomicBool>,
}

impl Default for MinkowskiOptions<'_> {
    fn default() -> Self {
        Self {
            max_cells: usize::MAX,
            max_pair_sums: usize::MAX,
            max_pair_points: usize::MAX,
            cancellation: None,
        }
    }
}

impl MinkowskiOptions<'_> {
    fn check_cancelled(&self) -> HypermeshResult<()> {
        if self
            .cancellation
            .is_some_and(|flag| flag.load(Ordering::Relaxed))
        {
            Err(HypermeshError::OperationCancelled {
                operation: "Minkowski sum",
            })
        } else {
            Ok(())
        }
    }
}

/// Computes the regularized sum of two bounded closed-PWN solids.
///
/// Nonzero winding defines material, including reversed, nested, disconnected
/// and overlapping shells. Only referenced positions contribute geometry.
/// Canonical empty meshes are supported; both operands are validated before an
/// empty identity is applied. A successful result is reusable closed-PWN geometry
/// (or canonical empty), not a claim about an intended analytic surface.
///
/// The receipt covers decisions consumed by this operation. The native mesh
/// additionally retains the construction qualifications of both operands.
pub fn minkowski_sum(
    context: &MeshContext,
    left: &TriangleMesh,
    right: &TriangleMesh,
) -> HypermeshResult<MeshOutcome<TriangleMesh>> {
    minkowski_sum_with_options(context, left, right, &MinkowskiOptions::default())
}

/// Computes a solid sum with explicit collection limits and cancellation.
///
/// Limits fail with [`HypermeshError::ResourceLimitExceeded`]; cancellation
/// returns [`HypermeshError::OperationCancelled`]. No partial mesh is published.
/// See [`MinkowskiOptions`] for the units and remaining unbounded primitive work.
pub fn minkowski_sum_with_options(
    context: &MeshContext,
    left: &TriangleMesh,
    right: &TriangleMesh,
    options: &MinkowskiOptions<'_>,
) -> HypermeshResult<MeshOutcome<TriangleMesh>> {
    options.check_cancelled()?;
    let decisions = DecisionContext::new(context);
    let evidence = left
        .construction_evidence()
        .combine(right.construction_evidence());
    let left = prepare_source(context, &decisions, left, 0)?;
    let right = prepare_source(context, &decisions, right, 1)?;
    if left.triangles.is_empty() || right.triangles.is_empty() {
        options.check_cancelled()?;
        return Ok(decisions.finish(empty().with_inherited_construction(
            evidence.with_predicate_certainty(decisions.certainty()),
        )));
    }
    let left = convex_cells(context, &decisions, left, options)?;
    if left.is_empty() {
        options.check_cancelled()?;
        return Ok(decisions.finish(empty().with_inherited_construction(
            evidence.with_predicate_certainty(decisions.certainty()),
        )));
    }
    let right = convex_cells(context, &decisions, right, options)?;
    let count = product(left.len(), right.len(), "Minkowski convex pairs")?;
    check_limit("Minkowski convex pairs", count, options.max_pair_sums)?;
    let mut accumulated: Option<TriangleMesh> = None;
    for left in &left {
        for right in &right {
            options.check_cancelled()?;
            let sum = pair_hull(&decisions, &left.positions, &right.positions, options)?;
            accumulated = Some(match accumulated {
                None => sum,
                Some(previous) => {
                    options.check_cancelled()?;
                    let union = boolean(
                        context,
                        &[previous.as_ref(), sum.as_ref()],
                        BooleanProgram::Operation(BooleanOp::Union),
                    )?;
                    decisions.absorb(union.certainty);
                    union.value.into_triangle_meshes()?.pop().ok_or(
                        HypermeshError::SurfaceArrangementFailed {
                            reason: "Minkowski union produced no result row",
                        },
                    )?
                }
            });
        }
    }
    let result = accumulated.unwrap_or_else(empty);
    options.check_cancelled()?;
    Ok(decisions.finish(
        result
            .with_inherited_construction(evidence.with_predicate_certainty(decisions.certainty())),
    ))
}

/// Computes `conv({a+b | a in left, b in right})` for stored point sets.
///
/// This explicitly convexifies both sets. It is distinct from the sum of
/// arbitrary represented solids. Empty point sets yield canonical empty;
/// a nonempty lower-dimensional sum returns [`HypermeshError::DegeneratePointSet`].
pub fn minkowski_hull(
    context: &MeshContext,
    left: &[Point3],
    right: &[Point3],
) -> HypermeshResult<MeshOutcome<TriangleMesh>> {
    let decisions = DecisionContext::new(context);
    let result = if left.is_empty() || right.is_empty() {
        empty()
    } else {
        pair_hull(&decisions, left, right, &MinkowskiOptions::default())?
    };
    Ok(decisions.finish(
        result.with_inherited_construction(
            crate::ConstructionEvidence::DECLARED_COORDINATES
                .with_predicate_certainty(decisions.certainty()),
        ),
    ))
}

fn empty() -> TriangleMesh {
    TriangleMesh::from_construction(
        Vec::new(),
        Vec::new(),
        crate::ConstructionEvidence::DECLARED_COORDINATES,
    )
}

fn check_limit(resource: &'static str, required: usize, limit: usize) -> HypermeshResult<()> {
    if required > limit {
        Err(HypermeshError::ResourceLimitExceeded {
            resource,
            limit,
            required,
        })
    } else {
        Ok(())
    }
}

fn product(left: usize, right: usize, operation: &'static str) -> HypermeshResult<usize> {
    left.checked_mul(right)
        .ok_or(HypermeshError::CapacityOverflow { operation })
}

fn reserved<T>(count: usize, operation: &'static str) -> HypermeshResult<Vec<T>> {
    let mut values = Vec::new();
    values
        .try_reserve_exact(count)
        .map_err(|_| HypermeshError::CapacityOverflow { operation })?;
    Ok(values)
}

// Compact only by index usage, with no coordinate equality or numeric decision.
// The native validation then sees exactly the authored boundary, including its
// winding multiplicities. An unused undecidable coordinate has no geometric role.
fn prepare_source(
    context: &MeshContext,
    decisions: &DecisionContext,
    mesh: &TriangleMesh,
    mesh_index: usize,
) -> HypermeshResult<TriangleMesh> {
    if mesh.positions.is_empty() && mesh.triangles.is_empty() {
        return Ok(mesh.clone());
    }
    let mut mapping = reserved(mesh.positions.len(), "Minkowski source indices")?;
    mapping.resize(mesh.positions.len(), usize::MAX);
    for triangle in mesh.triangles.iter() {
        for index in triangle.indices() {
            *mapping
                .get_mut(index)
                .ok_or(HypermeshError::VertexIndexOutOfBounds {
                    index,
                    vertex_count: mesh.positions.len(),
                })? = 0;
        }
    }
    if mesh.triangles.is_empty() {
        return Err(HypermeshError::EmptyMesh { mesh_index });
    }
    let referenced = mapping.iter().filter(|index| **index == 0).count();
    let source = if referenced == mesh.positions.len() {
        mesh.clone()
    } else {
        let mut positions = reserved(referenced, "Minkowski source positions")?;
        for (index, point) in mapping.iter_mut().zip(mesh.positions.iter()) {
            if *index == 0 {
                *index = positions.len();
                positions.push(point.clone());
            }
        }
        let mut triangles = reserved(mesh.triangles.len(), "Minkowski source triangles")?;
        triangles.extend(mesh.triangles.iter().map(|triangle| {
            let [a, b, c] = triangle.indices().map(|index| mapping[index]);
            Triangle::new(a, b, c)
        }));
        TriangleMesh::from_construction(positions, triangles, mesh.construction_evidence())
    };
    decisions.absorb(source.validate_closed_pwn(context)?.certainty);
    Ok(source)
}

fn pair_hull(
    decisions: &DecisionContext,
    left: &[Point3],
    right: &[Point3],
    options: &MinkowskiOptions<'_>,
) -> HypermeshResult<TriangleMesh> {
    let count = product(left.len(), right.len(), "Minkowski vertex sums")?;
    check_limit("Minkowski vertex sums", count, options.max_pair_points)?;
    let mut points = reserved(count, "Minkowski vertex sums")?;
    for a in left {
        options.check_cancelled()?;
        for b in right {
            points.push(Point3::new(&a.x + &b.x, &a.y + &b.y, &a.z + &b.z));
        }
    }
    convex_hull_impl(decisions, &points, &[], None)
}

fn convex_cells(
    context: &MeshContext,
    decisions: &DecisionContext,
    source: TriangleMesh,
    options: &MinkowskiOptions<'_>,
) -> HypermeshResult<Vec<TriangleMesh>> {
    options.check_cancelled()?;
    check_limit("Minkowski decomposition cells", 1, options.max_cells)?;
    if !source_is_full_dimensional(decisions, &source)? {
        return Ok(Vec::new());
    }
    match certify_convex_mesh(context, source.as_ref()) {
        Ok(proof) => {
            decisions.absorb(proof.certainty);
            return Ok(vec![source]);
        }
        Err(HypermeshError::NonConvexInput | HypermeshError::PredicateUndecided { .. }) => {}
        Err(error) => return Err(error),
    }
    // Full dimension was proved above. A hull failure here is an error, never
    // an alternative way to manufacture successful empty geometry.
    let hull = convex_hull_impl(decisions, &source.positions, &[], None)?;
    let mut cells = vec![hull];
    for triangle in source.triangles.iter() {
        let [a, b, c] = source.triangle_points(*triangle)?;
        let plane = Plane::from_points(a, b, c);
        let old_count = cells.len();
        for index in 0..old_count {
            options.check_cancelled()?;
            let mut sides = reserved(cells[index].positions.len(), "Minkowski cell sides")?;
            for point in cells[index].positions.iter() {
                sides.push(classify_point_decision(decisions, point, &plane)?);
            }
            if !sides.contains(&Classification::Negative)
                || !sides.contains(&Classification::Positive)
            {
                continue;
            }
            let count = cells
                .len()
                .checked_add(1)
                .ok_or(HypermeshError::CapacityOverflow {
                    operation: "Minkowski decomposition cells",
                })?;
            check_limit("Minkowski decomposition cells", count, options.max_cells)?;
            cells
                .try_reserve(1)
                .map_err(|_| HypermeshError::CapacityOverflow {
                    operation: "Minkowski decomposition cells",
                })?;
            let [left, right] = split_cell(decisions, &cells[index], &sides, &plane)?;
            cells[index] = left;
            cells.push(right);
        }
    }
    let mut retained = reserved(cells.len(), "Minkowski interior cells")?;
    for cell in cells {
        options.check_cancelled()?;
        let denominator = Real::from(u64::try_from(cell.positions.len()).map_err(|_| {
            HypermeshError::CapacityOverflow {
                operation: "Minkowski cell centroid",
            }
        })?);
        let inverse = (Real::one() / denominator).expect("a full-dimensional hull has vertices");
        let mut centroid = Point3::origin();
        for point in cell.positions.iter() {
            centroid.x += &point.x;
            centroid.y += &point.y;
            centroid.z += &point.z;
        }
        centroid.x *= &inverse;
        centroid.y *= &inverse;
        centroid.z *= &inverse;
        if mesh_winding_number_decision(decisions, &source, &centroid, || {
            options.check_cancelled()
        })? != 0
        {
            retained.push(cell);
        }
    }
    Ok(retained)
}

fn source_is_full_dimensional(
    decisions: &DecisionContext,
    source: &TriangleMesh,
) -> HypermeshResult<bool> {
    let triangle = source.triangles[0];
    let [a, b, c] = source.triangle_points(triangle)?;
    let coplanar = decisions.isolated();
    let mut unknown = false;
    for (index, point) in source.positions.iter().enumerate() {
        if triangle.indices().contains(&index) {
            continue;
        }
        let local = decisions.isolated();
        match local.probe(hyperlimit::orient3(a, b, c, point, decisions.policy())) {
            Some(hyperlimit::Sign::Zero) => coplanar.absorb(local.certainty()),
            Some(hyperlimit::Sign::Positive | hyperlimit::Sign::Negative) => {
                decisions.absorb(local.certainty());
                return Ok(true);
            }
            None => unknown = true,
        }
    }
    if unknown {
        Err(HypermeshError::PredicateUndecided {
            predicate: "Minkowski source dimension",
        })
    } else {
        decisions.absorb(coplanar.certainty());
        Ok(false)
    }
}

fn split_cell(
    decisions: &DecisionContext,
    cell: &TriangleMesh,
    sides: &[Classification],
    plane: &Plane,
) -> HypermeshResult<[TriangleMesh; 2]> {
    let mut points: [Vec<Point3>; 2] = [Vec::new(), Vec::new()];
    let mut maps: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
    let mut groups: [Vec<Vec<usize>>; 2] = [Vec::new(), Vec::new()];
    let mut caps: [Vec<usize>; 2] = [Vec::new(), Vec::new()];
    for side in 0..2 {
        points[side] = reserved(cell.positions.len(), "Minkowski split vertices")?;
        maps[side] = reserved(cell.positions.len(), "Minkowski split indices")?;
        maps[side].resize(cell.positions.len(), usize::MAX);
        for (index, point) in cell.positions.iter().enumerate() {
            if sides[index] == Classification::On
                || (side == 0) == (sides[index] == Classification::Negative)
            {
                maps[side][index] = points[side].len();
                if sides[index] == Classification::On {
                    caps[side].push(points[side].len());
                }
                points[side].push(point.clone());
            }
        }
    }
    let mut crossings = BTreeMap::<[usize; 2], [usize; 2]>::new();
    for triangle in cell.triangles.iter() {
        let vertices = triangle.indices();
        let mut face = [Vec::new(), Vec::new()];
        for index in vertices {
            for side in 0..2 {
                if maps[side][index] != usize::MAX {
                    face[side].push(maps[side][index]);
                }
            }
        }
        for [a, b] in [
            [vertices[0], vertices[1]],
            [vertices[1], vertices[2]],
            [vertices[2], vertices[0]],
        ] {
            if sides[a] == sides[b]
                || sides[a] == Classification::On
                || sides[b] == Classification::On
            {
                continue;
            }
            let mut key = [a, b];
            key.sort_unstable();
            let ids = if let Some(ids) = crossings.get(&key) {
                *ids
            } else {
                let [a, b] = key.map(|index| &cell.positions[index]);
                let direction = b - a;
                let moment = a.to_vector().cross(&direction);
                let [x, y, z] = direction.0;
                let [mx, my, mz] = moment.0;
                let line = HomogeneousLine3::new(Point3::new(x, y, z), Point3::new(mx, my, mz));
                let point =
                    affine_projective_point_decision(decisions, &line.intersect_plane(plane))?;
                let ids = [points[0].len(), points[1].len()];
                for side in 0..2 {
                    points[side]
                        .try_reserve(1)
                        .map_err(|_| HypermeshError::CapacityOverflow {
                            operation: "Minkowski split vertices",
                        })?;
                    points[side].push(point.clone());
                    caps[side].push(ids[side]);
                }
                crossings.insert(key, ids);
                ids
            };
            for side in 0..2 {
                face[side].push(ids[side]);
            }
        }
        for side in 0..2 {
            if face[side].len() >= 3 {
                groups[side].push(std::mem::take(&mut face[side]));
            }
        }
    }
    for side in 0..2 {
        groups[side].push(std::mem::take(&mut caps[side]));
    }
    Ok([
        convex_hull_impl(decisions, &points[0], &groups[0], None)?,
        convex_hull_impl(decisions, &points[1], &groups[1], None)?,
    ])
}
