//! Exact affine-rectilinear arrangements for coplanar surface booleans.
//!
//! This module extends the orthogonal rectangular cell materializer to
//! non-axis-aligned parallelogram grids. Each source cell must certify as one
//! parallelogram in a shared exact affine basis. The operands are transformed
//! into exact `(u, v, 0)` rectangle meshes, replayed through
//! [`crate::exact::orthogonal_surface`], and transformed back to 3D only after
//! affine basis and transformed cell complex are retained exact object
//! structure, not a primitive-float normalization.
//!
//! The cell subdivision is the affine image of the planar-arrangement
//! Chapter 2. Affine coordinates are constructed with exact determinant ratios,
//! and every accepted source vertex must reconstruct exactly from the retained
//! basis before the orthogonal cell complex is allowed to decide topology.

use core::cmp::Ordering;

use hyperlimit::{
    Point2, Point3, Sign, compare_reals, orient2d_report, project_point3 as project_point,
};

use super::coplanar::CoplanarProjection;
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::ExactMesh;
use super::orthogonal_surface::{
    CoplanarOrthogonalSurfaceArrangement, CoplanarOrthogonalSurfaceComponent,
    CoplanarOrthogonalSurfaceOperation, arrange_coplanar_orthogonal_surface_difference,
    arrange_coplanar_orthogonal_surface_intersection, arrange_coplanar_orthogonal_surface_union,
    certify_axis_aligned_surface_cells,
};
use super::provenance::SourceProvenance;
use super::validation::ValidationPolicy;
use hyperreal::Real;

/// Exact affine basis used to normalize parallelogram source cells.
///
/// A point in retained cell coordinates is interpreted as
/// `origin + u * basis_u + v * basis_v`. The `projection` is only the exact
/// coordinate projection used to solve determinant ratios; accepted vertices
/// must reconstruct in full 3D.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarAffineSurfaceBasis {
    /// Coordinate projection used to solve exact affine coordinates.
    pub projection: CoplanarProjection,
    /// Exact 3D affine origin.
    pub origin: Point3,
    /// Exact 3D vector for the local `u` axis.
    pub basis_u: Point3,
    /// Exact 3D vector for the local `v` axis.
    pub basis_v: Point3,
}

/// Exact affine-rectilinear coplanar surface arrangement.
///
/// This is the non-axis-aligned counterpart to
/// [`CoplanarOrthogonalSurfaceArrangement`]. The retained basis is part of the
/// certificate: without it, a copied loop could not replay the exact
/// parallelogram cell complex that justified the boolean output.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarAffineSurfaceArrangement {
    /// Shared affine basis used to normalize source parallelograms.
    pub basis: CoplanarAffineSurfaceBasis,
    /// Boolean operation used to produce the retained cells.
    pub operation: CoplanarOrthogonalSurfaceOperation,
    /// Retained connected components with optional holes in original 3D space.
    pub components: Vec<CoplanarOrthogonalSurfaceComponent>,
    /// Exact triangulated open surface mesh in original 3D space.
    pub mesh: ExactMesh,
}

impl CoplanarAffineSurfaceArrangement {
    /// Validate retained affine loops, mesh area, and exact mesh state.
    ///
    /// Validation maps the output back into the retained affine basis and then
    /// reuses the orthogonal-cell validator. This keeps the affine output
    /// the normalized cell complex.
    pub fn validate(&self) -> Result<(), MeshError> {
        let uv_components = self
            .components
            .iter()
            .map(|component| component_to_uv(component, &self.basis))
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| {
                affine_error("affine output loop does not replay into retained basis")
            })?;
        let uv_mesh = mesh_to_uv(&self.mesh, &self.basis).ok_or_else(|| {
            affine_error("affine output mesh does not replay into retained basis")
        })?;
        let orthogonal = CoplanarOrthogonalSurfaceArrangement {
            projection: CoplanarProjection::Xy,
            operation: self.operation,
            components: uv_components,
            mesh: uv_mesh,
        };
        orthogonal.validate()?;
        self.mesh
            .validate_retained_state()
            .map_err(|err| affine_error(format!("affine output mesh is stale: {err:?}")))?;
        Ok(())
    }

    /// Validate this arrangement by replaying it from the original sources.
    ///
    /// Replaying the affine-basis discovery and cell arrangement prevents a
    /// locally valid transformed mesh from being relabeled as another source
    /// boundary: exact outputs remain tied to the exact source objects and
    /// predicates that produced them.
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_affine_surface(left, right, self.operation)
            .ok_or_else(|| affine_error("source replay did not reproduce affine arrangement"))?;
        if self == &replay {
            Ok(())
        } else {
            Err(affine_error(
                "retained affine arrangement does not match source replay",
            ))
        }
    }
}

/// Certify and materialize an affine-rectilinear coplanar surface union.
pub fn arrange_coplanar_affine_surface_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarAffineSurfaceArrangement> {
    arrange_coplanar_affine_surface(left, right, CoplanarOrthogonalSurfaceOperation::Union)
}

/// Certify and materialize an affine-rectilinear coplanar surface intersection.
pub fn arrange_coplanar_affine_surface_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarAffineSurfaceArrangement> {
    arrange_coplanar_affine_surface(
        left,
        right,
        CoplanarOrthogonalSurfaceOperation::Intersection,
    )
}

/// Certify and materialize an affine-rectilinear coplanar surface difference.
pub fn arrange_coplanar_affine_surface_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarAffineSurfaceArrangement> {
    arrange_coplanar_affine_surface(left, right, CoplanarOrthogonalSurfaceOperation::Difference)
}

fn arrange_coplanar_affine_surface(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: CoplanarOrthogonalSurfaceOperation,
) -> Option<CoplanarAffineSurfaceArrangement> {
    let basis = certify_affine_basis(left, right)?;
    let left_uv = mesh_to_uv(left, &basis)?;
    let right_uv = mesh_to_uv(right, &basis)?;
    let orthogonal = match operation {
        CoplanarOrthogonalSurfaceOperation::Union => {
            arrange_coplanar_orthogonal_surface_union(&left_uv, &right_uv)?
        }
        CoplanarOrthogonalSurfaceOperation::Intersection => {
            arrange_coplanar_orthogonal_surface_intersection(&left_uv, &right_uv)?
        }
        CoplanarOrthogonalSurfaceOperation::Difference => {
            arrange_coplanar_orthogonal_surface_difference(&left_uv, &right_uv)?
        }
    };
    let components = orthogonal
        .components
        .iter()
        .map(|component| component_from_uv(component, &basis))
        .collect::<Option<Vec<_>>>()?;
    let mesh = mesh_from_uv(&orthogonal.mesh, &basis)?;
    let arrangement = CoplanarAffineSurfaceArrangement {
        basis,
        operation,
        components,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

fn certify_affine_basis(left: &ExactMesh, right: &ExactMesh) -> Option<CoplanarAffineSurfaceBasis> {
    let left_quads = extract_parallelogram_quads(left).unwrap_or_default();
    let right_quads = extract_parallelogram_quads(right).unwrap_or_default();
    if left_quads.is_empty() && right_quads.is_empty() {
        return None;
    }

    // A paired parallelogram from either operand is enough to define a
    // EGC boundary is the replay step below: every source vertex must map back
    // exactly and both normalized meshes must certify as orthogonal cells
    // before this basis becomes evidence.
    for seed in left_quads.iter().chain(right_quads.iter()) {
        let Some(projection) = choose_projection(seed) else {
            continue;
        };
        let Some(first_ordered) = parallelogram_order(seed, projection) else {
            continue;
        };
        let Some(basis) = basis_from_ordered_parallelogram(&first_ordered, projection) else {
            continue;
        };
        if left_quads
            .iter()
            .chain(right_quads.iter())
            .all(|quad| validate_affine_rectangle_quad(quad, &basis).is_some())
            && certify_affine_mesh_cells(left, &basis)
            && certify_affine_mesh_cells(right, &basis)
        {
            return Some(basis);
        }
    }
    None
}

fn certify_affine_mesh_cells(mesh: &ExactMesh, basis: &CoplanarAffineSurfaceBasis) -> bool {
    mesh_to_uv(mesh, basis)
        .as_ref()
        .is_some_and(certify_axis_aligned_surface_cells)
}

fn extract_parallelogram_quads(mesh: &ExactMesh) -> Option<Vec<[Point3; 4]>> {
    if mesh.triangles().is_empty() || !mesh.triangles().len().is_multiple_of(2) {
        return None;
    }
    let points = mesh_points(mesh);
    let mut used = vec![false; mesh.triangles().len()];
    let mut quads = Vec::with_capacity(mesh.triangles().len() / 2);
    for left in 0..mesh.triangles().len() {
        if used[left] {
            continue;
        }
        let mut matched = None;
        for (right, right_used) in used.iter().enumerate().skip(left + 1) {
            if *right_used || shared_triangle_point_count(mesh, left, right) != 2 {
                continue;
            }
            let unique = unique_triangle_pair_points(&points, mesh, left, right)?;
            let projection = choose_projection(&unique)?;
            let ordered = parallelogram_order(&unique, projection)?;
            if triangle_pair_area_matches_quad(&points, mesh, left, right, &ordered, projection)? {
                matched = Some((right, ordered));
                break;
            }
        }
        let (right, ordered) = matched?;
        used[left] = true;
        used[right] = true;
        quads.push(ordered);
    }
    Some(quads)
}

fn shared_triangle_point_count(mesh: &ExactMesh, left: usize, right: usize) -> usize {
    let left_tri = mesh.triangles()[left].0;
    let right_tri = mesh.triangles()[right].0;
    left_tri
        .iter()
        .filter(|&&left_index| {
            right_tri.iter().any(|&right_index| {
                points_equal(
                    &mesh.vertices()[left_index].clone(),
                    &mesh.vertices()[right_index].clone(),
                )
            })
        })
        .count()
}

fn unique_triangle_pair_points(
    points: &[Point3],
    mesh: &ExactMesh,
    left: usize,
    right: usize,
) -> Option<[Point3; 4]> {
    let mut unique = Vec::with_capacity(4);
    for index in mesh.triangles()[left]
        .0
        .iter()
        .chain(mesh.triangles()[right].0.iter())
    {
        let point = points.get(*index)?.clone();
        if !unique
            .iter()
            .any(|candidate| points_equal(candidate, &point))
        {
            unique.push(point);
        }
    }
    <[Point3; 4]>::try_from(unique).ok()
}

fn parallelogram_order(
    points: &[Point3; 4],
    projection: CoplanarProjection,
) -> Option<[Point3; 4]> {
    for origin in 0..4 {
        let others = (0..4).filter(|index| *index != origin).collect::<Vec<_>>();
        for adjacent_a_slot in 0..3 {
            for adjacent_b_slot in adjacent_a_slot + 1..3 {
                let adjacent_a = others[adjacent_a_slot];
                let adjacent_b = others[adjacent_b_slot];
                let opposite = *others
                    .iter()
                    .find(|&&index| index != adjacent_a && index != adjacent_b)?;
                if !point2_sum_equal(
                    &project_point(&points[origin], projection),
                    &project_point(&points[opposite], projection),
                    &project_point(&points[adjacent_a], projection),
                    &project_point(&points[adjacent_b], projection),
                ) {
                    continue;
                }
                let origin2 = project_point(&points[origin], projection);
                let a2 = project_point(&points[adjacent_a], projection);
                let b2 = project_point(&points[adjacent_b], projection);
                let cross = cross2(&sub2(&a2, &origin2), &sub2(&b2, &origin2));
                match compare_reals(&cross, &Real::from(0)).value()? {
                    Ordering::Greater => {
                        return Some([
                            points[origin].clone(),
                            points[adjacent_a].clone(),
                            points[opposite].clone(),
                            points[adjacent_b].clone(),
                        ]);
                    }
                    Ordering::Less => {
                        return Some([
                            points[origin].clone(),
                            points[adjacent_b].clone(),
                            points[opposite].clone(),
                            points[adjacent_a].clone(),
                        ]);
                    }
                    Ordering::Equal => {}
                }
            }
        }
    }
    None
}

fn triangle_pair_area_matches_quad(
    points: &[Point3],
    mesh: &ExactMesh,
    left: usize,
    right: usize,
    ordered: &[Point3; 4],
    projection: CoplanarProjection,
) -> Option<bool> {
    let left_area = triangle_area2_abs(points, mesh.triangles()[left].0, projection)?;
    let right_area = triangle_area2_abs(points, mesh.triangles()[right].0, projection)?;
    let quad_area = projected_area2_abs(ordered, projection)?;
    Some(compare_reals(&add(&left_area, &right_area), &quad_area).value()? == Ordering::Equal)
}

fn basis_from_ordered_parallelogram(
    ordered: &[Point3; 4],
    projection: CoplanarProjection,
) -> Option<CoplanarAffineSurfaceBasis> {
    let origin = ordered[0].clone();
    let basis_u = sub3(&ordered[1], &origin);
    let basis_v = sub3(&ordered[3], &origin);
    let u2 = project_vector(&basis_u, projection);
    let v2 = project_vector(&basis_v, projection);
    if compare_reals(&cross2(&u2, &v2), &Real::from(0)).value()? == Ordering::Equal {
        return None;
    }
    Some(CoplanarAffineSurfaceBasis {
        projection,
        origin,
        basis_u,
        basis_v,
    })
}

fn validate_affine_rectangle_quad(
    quad: &[Point3; 4],
    basis: &CoplanarAffineSurfaceBasis,
) -> Option<()> {
    let coords = quad
        .iter()
        .map(|point| point_to_uv_checked(point, basis))
        .collect::<Option<Vec<_>>>()?;
    let mut us = Vec::new();
    let mut vs = Vec::new();
    for coord in &coords {
        push_unique_real(&mut us, coord.x.clone());
        push_unique_real(&mut vs, coord.y.clone());
    }
    sort_reals_and_dedup(&mut us)?;
    sort_reals_and_dedup(&mut vs)?;
    if us.len() != 2
        || vs.len() != 2
        || real_order(&us[0], &us[1])? != Ordering::Less
        || real_order(&vs[0], &vs[1])? != Ordering::Less
    {
        return None;
    }
    for u in &us {
        for v in &vs {
            if !coords
                .iter()
                .any(|coord| real_equal(&coord.x, u) && real_equal(&coord.y, v))
            {
                return None;
            }
        }
    }
    Some(())
}

fn component_to_uv(
    component: &CoplanarOrthogonalSurfaceComponent,
    basis: &CoplanarAffineSurfaceBasis,
) -> Option<CoplanarOrthogonalSurfaceComponent> {
    Some(CoplanarOrthogonalSurfaceComponent {
        outer: component
            .outer
            .iter()
            .map(|point| point_to_uv3_checked(point, basis))
            .collect::<Option<Vec<_>>>()?,
        holes: component
            .holes
            .iter()
            .map(|hole| {
                hole.iter()
                    .map(|point| point_to_uv3_checked(point, basis))
                    .collect::<Option<Vec<_>>>()
            })
            .collect::<Option<Vec<_>>>()?,
    })
}

fn component_from_uv(
    component: &CoplanarOrthogonalSurfaceComponent,
    basis: &CoplanarAffineSurfaceBasis,
) -> Option<CoplanarOrthogonalSurfaceComponent> {
    Some(CoplanarOrthogonalSurfaceComponent {
        outer: component
            .outer
            .iter()
            .map(|point| point_from_uv(&point.x, &point.y, basis))
            .collect(),
        holes: component
            .holes
            .iter()
            .map(|hole| {
                hole.iter()
                    .map(|point| point_from_uv(&point.x, &point.y, basis))
                    .collect::<Vec<_>>()
            })
            .collect(),
    })
}

fn mesh_to_uv(mesh: &ExactMesh, basis: &CoplanarAffineSurfaceBasis) -> Option<ExactMesh> {
    let vertices = mesh
        .vertices()
        .iter()
        .map(|point| {
            point_to_uv_checked(&point.clone(), basis)
                .map(|uv| Point3::new(uv.x, uv.y, Real::from(0)))
        })
        .collect::<Option<Vec<_>>>()?;
    ExactMesh::new_with_policy(
        vertices,
        mesh.triangles().to_vec(),
        SourceProvenance::exact("exact affine-normalized coplanar surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn mesh_from_uv(mesh: &ExactMesh, basis: &CoplanarAffineSurfaceBasis) -> Option<ExactMesh> {
    let vertices = mesh
        .vertices()
        .iter()
        .map(|point| {
            let point = point.clone();
            let lifted = point_from_uv(&point.x, &point.y, basis);
            Point3::new(lifted.x, lifted.y, lifted.z)
        })
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        mesh.triangles().to_vec(),
        SourceProvenance::exact("exact affine coplanar surface arrangement"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn point_to_uv3_checked(point: &Point3, basis: &CoplanarAffineSurfaceBasis) -> Option<Point3> {
    let uv = point_to_uv_checked(point, basis)?;
    Some(Point3::new(uv.x, uv.y, Real::from(0)))
}

fn point_to_uv_checked(point: &Point3, basis: &CoplanarAffineSurfaceBasis) -> Option<Point2> {
    let uv = point_to_uv(point, basis)?;
    let replay = point_from_uv(&uv.x, &uv.y, basis);
    if points_equal(&replay, point) {
        Some(uv)
    } else {
        None
    }
}

fn point_to_uv(point: &Point3, basis: &CoplanarAffineSurfaceBasis) -> Option<Point2> {
    let origin = project_point(&basis.origin, basis.projection);
    let point = project_point(point, basis.projection);
    let basis_u = project_vector(&basis.basis_u, basis.projection);
    let basis_v = project_vector(&basis.basis_v, basis.projection);
    let delta = sub2(&point, &origin);
    let denominator = cross2(&basis_u, &basis_v);
    if compare_reals(&denominator, &Real::from(0)).value()? == Ordering::Equal {
        return None;
    }
    let u = (cross2(&delta, &basis_v) / &denominator).ok()?;
    let v = (cross2(&basis_u, &delta) / &denominator).ok()?;
    Some(Point2::new(u, v))
}

fn point_from_uv(u: &Real, v: &Real, basis: &CoplanarAffineSurfaceBasis) -> Point3 {
    Point3::new(
        add(
            &basis.origin.x,
            &add(&mul(u, &basis.basis_u.x), &mul(v, &basis.basis_v.x)),
        ),
        add(
            &basis.origin.y,
            &add(&mul(u, &basis.basis_u.y), &mul(v, &basis.basis_v.y)),
        ),
        add(
            &basis.origin.z,
            &add(&mul(u, &basis.basis_u.z), &mul(v, &basis.basis_v.z)),
        ),
    )
}

fn choose_projection(points: &[Point3; 4]) -> Option<CoplanarProjection> {
    let candidates = [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ];
    for projection in candidates {
        for a in 0..points.len() {
            for b in a + 1..points.len() {
                for c in b + 1..points.len() {
                    let pa = project_point(&points[a], projection);
                    let pb = project_point(&points[b], projection);
                    let pc = project_point(&points[c], projection);
                    if !matches!(orient2d_report(&pa, &pb, &pc).value(), Some(Sign::Zero)) {
                        return Some(projection);
                    }
                }
            }
        }
    }
    None
}

fn triangle_area2_abs(
    points: &[Point3],
    triangle: [usize; 3],
    projection: CoplanarProjection,
) -> Option<Real> {
    let triangle = [
        points.get(triangle[0])?.clone(),
        points.get(triangle[1])?.clone(),
        points.get(triangle[2])?.clone(),
    ];
    projected_area2_abs(&triangle, projection)
}

fn projected_area2_abs(points: &[Point3], projection: CoplanarProjection) -> Option<Real> {
    let signed = projected_area2_signed(points, projection);
    match compare_reals(&signed, &Real::from(0)).value()? {
        Ordering::Less => Some(sub(&Real::from(0), &signed)),
        Ordering::Equal | Ordering::Greater => Some(signed),
    }
}

fn projected_area2_signed(points: &[Point3], projection: CoplanarProjection) -> Real {
    let mut sum = Real::from(0);
    for index in 0..points.len() {
        let current = project_point(&points[index], projection);
        let next = project_point(&points[(index + 1) % points.len()], projection);
        sum = add(
            &sum,
            &sub(&mul(&current.x, &next.y), &mul(&current.y, &next.x)),
        );
    }
    sum
}

fn project_vector(vector: &Point3, projection: CoplanarProjection) -> Point2 {
    project_point(vector, projection)
}

fn sub2(left: &Point2, right: &Point2) -> Point2 {
    Point2::new(sub(&left.x, &right.x), sub(&left.y, &right.y))
}

fn sub3(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub(&left.x, &right.x),
        sub(&left.y, &right.y),
        sub(&left.z, &right.z),
    )
}

fn cross2(left: &Point2, right: &Point2) -> Real {
    sub(&mul(&left.x, &right.y), &mul(&left.y, &right.x))
}

fn point2_sum_equal(left_a: &Point2, left_b: &Point2, right_a: &Point2, right_b: &Point2) -> bool {
    real_equal(&add(&left_a.x, &left_b.x), &add(&right_a.x, &right_b.x))
        && real_equal(&add(&left_a.y, &left_b.y), &add(&right_a.y, &right_b.y))
}

fn push_unique_real(values: &mut Vec<Real>, value: Real) {
    if !values.iter().any(|candidate| real_equal(candidate, &value)) {
        values.push(value);
    }
}

fn sort_reals_and_dedup(values: &mut Vec<Real>) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 && real_order(&values[cursor], &values[cursor - 1])? == Ordering::Less {
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    values.dedup_by(|right, left| real_equal(left, right));
    Some(())
}

fn mesh_points(mesh: &ExactMesh) -> Vec<Point3> {
    mesh.vertices().iter().cloned().collect()
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    real_equal(&left.x, &right.x) && real_equal(&left.y, &right.y) && real_equal(&left.z, &right.z)
}

fn real_order(left: &Real, right: &Real) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn real_equal(left: &Real, right: &Real) -> bool {
    real_order(left, right) == Some(Ordering::Equal)
}

fn add(left: &Real, right: &Real) -> Real {
    left.clone() + right
}

fn sub(left: &Real, right: &Real) -> Real {
    left.clone() - right
}

fn mul(left: &Real, right: &Real) -> Real {
    left.clone() * right
}

fn affine_error(message: impl Into<String>) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        message,
    ))
}
