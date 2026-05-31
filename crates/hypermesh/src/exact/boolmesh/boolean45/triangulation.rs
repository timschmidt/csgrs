//! Exact triangulation-prep for boolmesh `boolean45` face loops.
//!
//! Legacy boolmesh assembles halfedge loops, then triangulates each output face
//! boundary.  This module ports that handoff with exact source-face evidence:
//! simple components use `hypertri` earcut, while holed components lower their
//! retained boundary rings to `hypertri` CDT constraints.  Disjoint positive
//! contours remain separate components, matching legacy `EarClip` instead of
//! being flattened into artificial holes.  That separation follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): assembled boundary loops remain replayable topology, while exact
//! triangulation is a later certified object.  The simple-polygon basis is
//! Meisters, "Polygons Have Ears," *The American Mathematical Monthly* 82.6
//! (1975), and the exact ring containment rule follows the even-odd model in
//! Hormann and Agathos, "The Point in Polygon Problem for Arbitrary Polygons,"
//! *Computational Geometry* 20.3 (2001).

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::{Point2, Point3, RingPointLocation, classify_point_ring_even_odd, compare_reals};

use crate::exact::mesh::{ExactMesh, ExactPoint3};
use crate::exact::region::{choose_region_projection, project_for_hypertri};
use crate::exact::scalar::ExactReal;

use super::super::{
    ExactBoolMeshBoolean03, ExactBoolMeshFaceLoopAssemblyStage, ExactBoolMeshHalfedgeAssemblyStage,
    ExactBoolMeshLoopTriangulation, ExactBoolMeshLoopTriangulationStage,
    ExactBoolMeshOutputHalfedgeSource, ExactBoolMeshOutputVertexAllocation, ExactBoolMeshSide,
};
use super::geometry::output_vertex_point;

type CdtTriangulation = (Vec<usize>, Vec<[usize; 2]>, Vec<ExactPoint3>);

/// Triangulate assembled boolmesh output loops.
///
/// Legacy boolmesh's `general_triangulate` passes all loops of one output face
/// to its ear-clipper, with the outer loop and hole loops kept as separate
/// polygon rings.  Its `EarClip::new` then clips degenerate two-edge walks
/// before triangulation.  The exact port mirrors the same boundary by dropping
/// short walks and exactly zero-area rings when at least one positive-area ring
/// remains, partitioning positive-area rings by exact containment, and then
/// triangulating each connected component.  Single-loop components use
/// `hypertri` earcut; components with retained hole constraints use
/// `hypertri` CDT.  A face whose usable loops are all zero-area is recorded as
/// a regularized lower-dimensional deletion instead of a triangulation
/// failure.  This matches the exact-computation contract in Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
/// zero-area is a certified predicate result, not a tolerance outcome.  The
/// positive-area simple-loop handoff uses the exact earcut basis from Meisters,
/// "Polygons Have Ears," *The American Mathematical Monthly* 82.6 (1975).
pub(super) fn triangulate_output_face_loops(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_loops: &ExactBoolMeshFaceLoopAssemblyStage,
) -> ExactBoolMeshLoopTriangulationStage {
    let mut stage = ExactBoolMeshLoopTriangulationStage::default();
    let mut loops_by_face = BTreeMap::<usize, Vec<usize>>::new();
    for (loop_index, face_loop) in face_loops.loops.iter().enumerate() {
        loops_by_face
            .entry(face_loop.output_face)
            .or_default()
            .push(loop_index);
    }

    for loop_indices in loops_by_face.into_values() {
        triangulate_output_face_loop_group(
            &loop_indices,
            left,
            right,
            boolean03,
            allocation,
            halfedges,
            face_loops,
            &mut stage,
        );
    }

    stage
}

fn triangulate_output_face_loop_group(
    loop_indices: &[usize],
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_loops: &ExactBoolMeshFaceLoopAssemblyStage,
    stage: &mut ExactBoolMeshLoopTriangulationStage,
) {
    let short_loop_indices = loop_indices
        .iter()
        .copied()
        .filter(|loop_index| {
            let face_loop = &face_loops.loops[*loop_index];
            face_loop.vertices.len() < 3 || face_loop.halfedges.len() < 3
        })
        .collect::<Vec<_>>();
    let triangulatable_loop_indices = loop_indices
        .iter()
        .copied()
        .filter(|loop_index| {
            let face_loop = &face_loops.loops[*loop_index];
            face_loop.vertices.len() >= 3 && face_loop.halfedges.len() >= 3
        })
        .collect::<Vec<_>>();
    if triangulatable_loop_indices.is_empty() {
        if all_short_loops_are_face_pair_seams(loop_indices, halfedges, face_loops) {
            if let Some(face_loop) = loop_indices
                .first()
                .and_then(|loop_index| face_loops.loops.get(*loop_index))
            {
                stage.dropped_degenerate_faces.push(face_loop.output_face);
            }
            return;
        }
        stage.short_loops += 1;
        return;
    }

    let first_loop = &face_loops.loops[triangulatable_loop_indices[0]];
    let Some((source_side, source_face)) =
        loop_source_face(first_loop.halfedges.first().copied(), halfedges)
    else {
        stage.missing_source_faces += 1;
        return;
    };
    let Some(source_mesh) = source_mesh(source_side, source_face, left, right) else {
        stage.missing_source_faces += 1;
        return;
    };
    let Ok(projection) = choose_region_projection(source_mesh, source_face) else {
        stage.missing_source_faces += 1;
        return;
    };

    let mut rings = Vec::with_capacity(triangulatable_loop_indices.len());
    let mut degenerate_loop_indices = Vec::new();
    for &loop_index in &triangulatable_loop_indices {
        let face_loop = &face_loops.loops[loop_index];
        let Some(points) =
            output_loop_points(&face_loop.vertices, allocation, boolean03, left, right)
        else {
            stage.missing_vertex_coordinates += 1;
            return;
        };
        let projected = points
            .iter()
            .map(|point| project_for_hypertri(point, projection))
            .collect::<Vec<_>>();
        let Some(area_abs) = projected_area2_abs(&projected) else {
            stage.triangulation_failures += 1;
            return;
        };
        match compare_reals(&area_abs, &ExactReal::from(0)).value() {
            Some(Ordering::Greater) => {
                rings.push(ProjectedLoop {
                    loop_index,
                    vertices: face_loop.vertices.clone(),
                    projected,
                    area_abs,
                });
            }
            Some(Ordering::Equal) => {
                degenerate_loop_indices.push(loop_index);
            }
            Some(Ordering::Less) | None => {
                stage.triangulation_failures += 1;
                return;
            }
        }
    }
    if rings.is_empty() {
        stage.dropped_degenerate_faces.push(first_loop.output_face);
        return;
    }

    let (rings, mut clipped_loop_indices) = clip_boundary_covered_rings(rings);
    let Some(components) = partition_polygon_components(rings) else {
        stage.triangulation_failures += 1;
        return;
    };
    clipped_loop_indices.extend(short_loop_indices);
    clipped_loop_indices.extend(degenerate_loop_indices);
    clipped_loop_indices.sort_unstable();
    let mut clipped_loop_indices = Some(clipped_loop_indices);
    for component in components {
        let clipped_for_component = clipped_loop_indices.take().unwrap_or_default();
        let Some(triangulation) = triangulate_ring_component(
            first_loop.output_face,
            component,
            clipped_for_component,
            source_side,
            source_face,
            source_mesh,
            projection,
        ) else {
            stage.triangulation_failures += 1;
            return;
        };
        stage.triangulations.push(triangulation);
    }
}

fn all_short_loops_are_face_pair_seams(
    loop_indices: &[usize],
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_loops: &ExactBoolMeshFaceLoopAssemblyStage,
) -> bool {
    !loop_indices.is_empty()
        && loop_indices.iter().all(|loop_index| {
            let Some(face_loop) = face_loops.loops.get(*loop_index) else {
                return false;
            };
            (face_loop.vertices.len() < 3 || face_loop.halfedges.len() < 3)
                && !face_loop.halfedges.is_empty()
                && face_loop.halfedges.iter().all(|slot| {
                    halfedges
                        .output_halfedges
                        .get(*slot)
                        .is_some_and(|halfedge| {
                            halfedge.as_ref().is_some_and(|halfedge| {
                                matches!(
                                    halfedge.source,
                                    ExactBoolMeshOutputHalfedgeSource::NewFacePair { .. }
                                )
                            })
                        })
                })
        })
}

#[derive(Clone)]
struct ProjectedLoop {
    loop_index: usize,
    vertices: Vec<usize>,
    projected: Vec<hypertri::ExactPoint>,
    area_abs: ExactReal,
}

#[derive(Clone)]
struct RingComponent {
    exterior: ProjectedLoop,
    holes: Vec<ProjectedLoop>,
}

fn triangulate_ring_component(
    output_face: usize,
    component: RingComponent,
    clipped_loop_indices: Vec<usize>,
    source_side: ExactBoolMeshSide,
    source_face: usize,
    source_mesh: &ExactMesh,
    projection: hyperlimit::CoplanarProjection,
) -> Option<ExactBoolMeshLoopTriangulation> {
    let mut vertices = Vec::new();
    let mut projected = Vec::new();
    let mut hole_indices = Vec::new();
    let mut component_loop_indices = vec![component.exterior.loop_index];

    vertices.extend(component.exterior.vertices.iter().copied());
    projected.extend(component.exterior.projected.iter().cloned());
    for hole in &component.holes {
        hole_indices.push(projected.len());
        component_loop_indices.push(hole.loop_index);
        vertices.extend(hole.vertices.iter().copied());
        projected.extend(hole.projected.iter().cloned());
    }

    let (triangles, constraint_edges, steiner_points) = if hole_indices.is_empty() {
        (
            triangulate_simple_component(&projected)?,
            Vec::new(),
            Vec::new(),
        )
    } else {
        triangulate_component_with_cdt(
            &projected,
            &hole_indices,
            source_mesh,
            source_face,
            projection,
        )?
    };
    if triangles.is_empty() {
        return None;
    }

    Some(ExactBoolMeshLoopTriangulation {
        output_face,
        loop_index: component.exterior.loop_index,
        clipped_loop_indices,
        component_loop_indices,
        source_side,
        source_face,
        projection,
        vertices,
        steiner_points,
        constraint_edges,
        triangles,
    })
}

/// Triangulate one simple boolmesh face component.
///
/// Legacy boolmesh does not send every simple face through its general
/// ear-clipper.  `process_face` first handles three halfedges with
/// `single_triangulate`, then four halfedges with `square_triangulate`, and
/// only larger faces reach `general_triangulate`.  This exact port keeps that
/// control flow: triangles are copied directly, quadrilaterals choose the same
/// diagonal rule with exact projected orientation and exact squared lengths,
/// and only larger loops use the Meisters ear theorem implementation in
/// `hypertri::earcut`.  Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), is the boundary condition here: the
/// boolmesh topology branch is retained, while f64 epsilon orientation and
/// length comparisons are replaced by exact predicates.
fn triangulate_simple_component(projected: &[hypertri::ExactPoint]) -> Option<Vec<usize>> {
    match projected.len() {
        0..=2 => None,
        3 => Some(vec![0, 1, 2]),
        4 => triangulate_square_component(projected),
        _ => hypertri::earcut(projected, &[]).ok(),
    }
}

/// Port boolmesh `square_triangulate` with exact projected predicates.
///
/// Boolmesh first prefers the `(0, 2)` diagonal when both emitted triangles
/// preserve the face orientation.  If that diagonal is invalid, it switches to
/// `(1, 3)`; if both are valid, the longer first diagonal is avoided.  The
/// exact port measures validity by the sign of the projected triangle area
/// against the loop's exact signed area and compares squared diagonal lengths
/// in the selected projection.  If neither diagonal is certified as a
/// positive-area split, the loop is not a square-triangulate case in Yap's
/// exact-object sense and is handed to the general exact earcut path.
fn triangulate_square_component(projected: &[hypertri::ExactPoint]) -> Option<Vec<usize>> {
    debug_assert_eq!(projected.len(), 4);
    let loop_sign = square_orientation(projected)?;
    let choice0 = [[0, 1, 2], [0, 2, 3]];
    let choice1 = [[1, 2, 3], [0, 1, 3]];
    let choice0_valid = triangles_match_orientation(projected, &choice0, loop_sign);
    let choice1_valid = triangles_match_orientation(projected, &choice1, loop_sign);
    if !choice0_valid && !choice1_valid {
        return hypertri::earcut(projected, &[]).ok();
    }
    let use_choice1 = if !choice0_valid {
        true
    } else if choice1_valid {
        compare_reals(
            &squared_distance2(&projected[0], &projected[2]),
            &squared_distance2(&projected[1], &projected[3]),
        )
        .value()
        .is_some_and(|ordering| ordering == Ordering::Greater)
    } else {
        false
    };
    let triangles = if use_choice1 { choice1 } else { choice0 };
    Some(triangles.into_iter().flatten().collect())
}

fn square_orientation(projected: &[hypertri::ExactPoint]) -> Option<Ordering> {
    match compare_reals(&projected_area2_signed(projected), &ExactReal::from(0)).value()? {
        Ordering::Equal => None,
        ordering => Some(ordering),
    }
}

fn triangles_match_orientation(
    projected: &[hypertri::ExactPoint],
    triangles: &[[usize; 3]; 2],
    loop_sign: Ordering,
) -> bool {
    triangles.iter().all(|triangle| {
        compare_reals(
            &triangle_area2_signed(projected, *triangle),
            &ExactReal::from(0),
        )
        .value()
        .is_some_and(|triangle_sign| triangle_sign == loop_sign)
    })
}

fn squared_distance2(left: &hypertri::ExactPoint, right: &hypertri::ExactPoint) -> ExactReal {
    let dx = left.x.clone() - &right.x;
    let dy = left.y.clone() - &right.y;
    (dx.clone() * &dx) + &(dy.clone() * &dy)
}

fn point3_sub(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub_real(&left.x, &right.x),
        sub_real(&left.y, &right.y),
        sub_real(&left.z, &right.z),
    )
}

fn cross(left: &Point3, right: &Point3) -> Point3 {
    Point3::new(
        sub_real(&mul_real(&left.y, &right.z), &mul_real(&left.z, &right.y)),
        sub_real(&mul_real(&left.z, &right.x), &mul_real(&left.x, &right.z)),
        sub_real(&mul_real(&left.x, &right.y), &mul_real(&left.y, &right.x)),
    )
}

fn add_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn sub_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

fn div_real(numerator: ExactReal, denominator: &ExactReal) -> Option<ExactReal> {
    (numerator / denominator).ok()
}

/// Triangulate a nested boolmesh face component through exact CDT constraints.
///
/// Boolmesh's legacy `EarClip` receives the exterior and hole contours as
/// protected boundary topology.  For the exact port, the same topology is
/// lowered to a planar straight-line graph and routed through
/// `hypertri::cdt::constrained_delaunay`; the protected-edge legality follows
/// Lee and Lin, "Generalized Delaunay triangulation for planar graphs,"
/// *Discrete & Computational Geometry* 1 (1986), and the edge-recovery CDT
/// construction follows Shewchuk and Brown's constrained Delaunay treatment.
/// Yap's exact-computation model is the guardrail here: if CDT inserts a
/// planar Steiner point, the point is accepted only after it is lifted back to
/// the owning source face and reprojects exactly.
fn triangulate_component_with_cdt(
    projected: &[hypertri::ExactPoint],
    hole_indices: &[usize],
    source_mesh: &ExactMesh,
    source_face: usize,
    projection: hyperlimit::CoplanarProjection,
) -> Option<CdtTriangulation> {
    let mut ring_starts = Vec::with_capacity(hole_indices.len() + 1);
    ring_starts.push(0);
    ring_starts.extend(hole_indices.iter().copied());
    let mut constraints = Vec::new();
    for (ring_position, &start) in ring_starts.iter().enumerate() {
        let end = ring_starts
            .get(ring_position + 1)
            .copied()
            .unwrap_or(projected.len());
        if end.saturating_sub(start) < 3 {
            return None;
        }
        for local in start..end {
            let next = if local + 1 == end { start } else { local + 1 };
            constraints.push(hypertri::Constraint::new(local, next));
        }
    }

    let cdt = match hypertri::cdt::constrained_delaunay(projected, &constraints) {
        Ok(cdt) => cdt,
        Err(_) => {
            // `hypertri`'s closed-polygon CDT fast path can omit exact
            // collinear boundary vertices.  Boolmesh needs those subedges for
            // adjacent split faces, so refine the polygon triangles locally
            // before accepting the fallback.
            let triangles = triangulate_polygon_with_boundary_refinement(
                projected,
                hole_indices,
                &constraints,
            )?;
            return Some((
                triangles,
                constraints
                    .iter()
                    .map(|edge| [edge.from, edge.to])
                    .collect::<Vec<_>>(),
                Vec::new(),
            ));
        }
    };
    if cdt.points().len() < projected.len() {
        return None;
    }
    let steiner_points = cdt.points()[projected.len()..]
        .iter()
        .map(|point| lift_projected_boolmesh_steiner(source_mesh, source_face, projection, point))
        .collect::<Option<Vec<_>>>()?;
    Some((
        cdt.triangles()
            .iter()
            .flat_map(|triangle| triangle.iter().copied())
            .collect(),
        cdt.constraint_edges()
            .iter()
            .map(|edge| [edge.from, edge.to])
            .collect::<Vec<_>>(),
        steiner_points,
    ))
}

fn triangulate_polygon_with_boundary_refinement(
    projected: &[hypertri::ExactPoint],
    hole_indices: &[usize],
    constraints: &[hypertri::Constraint],
) -> Option<Vec<usize>> {
    let mut triangles = hypertri::earcut(projected, hole_indices).ok()?;
    let max_passes = constraints.len().saturating_mul(projected.len()).max(1);
    for _ in 0..max_passes {
        let Some(missing) = constraints
            .iter()
            .find(|constraint| !triangles_have_edge(&triangles, constraint.from, constraint.to))
        else {
            return Some(triangles);
        };
        if !split_collinear_boundary_edge(projected, &mut triangles, missing.from, missing.to) {
            return None;
        }
    }
    constraints
        .iter()
        .all(|constraint| triangles_have_edge(&triangles, constraint.from, constraint.to))
        .then_some(triangles)
}

fn triangles_have_edge(triangles: &[usize], from: usize, to: usize) -> bool {
    triangles.chunks_exact(3).any(|triangle| {
        (0..3).any(|edge| {
            let a = triangle[edge];
            let b = triangle[(edge + 1) % 3];
            (a == from && b == to) || (a == to && b == from)
        })
    })
}

fn split_collinear_boundary_edge(
    projected: &[hypertri::ExactPoint],
    triangles: &mut Vec<usize>,
    from: usize,
    to: usize,
) -> bool {
    let mut triangle_index = 0;
    while triangle_index + 2 < triangles.len() {
        let triangle = [
            triangles[triangle_index],
            triangles[triangle_index + 1],
            triangles[triangle_index + 2],
        ];
        for edge in 0..3 {
            let a = triangle[edge];
            let b = triangle[(edge + 1) % 3];
            let opposite = triangle[(edge + 2) % 3];
            for mid in [from, to] {
                if point_lies_strictly_between(projected, mid, a, b)
                    && split_triangle_edge(
                        projected,
                        triangles,
                        triangle_index,
                        a,
                        mid,
                        b,
                        opposite,
                    )
                {
                    return true;
                }
            }
        }
        triangle_index += 3;
    }
    false
}

fn split_triangle_edge(
    projected: &[hypertri::ExactPoint],
    triangles: &mut Vec<usize>,
    triangle_index: usize,
    a: usize,
    mid: usize,
    b: usize,
    opposite: usize,
) -> bool {
    let first = [a, mid, opposite];
    let second = [mid, b, opposite];
    if !triangle_is_non_degenerate(projected, first)
        || !triangle_is_non_degenerate(projected, second)
    {
        return false;
    }
    triangles.splice(
        triangle_index..triangle_index + 3,
        first.into_iter().chain(second),
    );
    true
}

fn triangle_is_non_degenerate(projected: &[hypertri::ExactPoint], triangle: [usize; 3]) -> bool {
    compare_reals(
        &triangle_area2_signed(projected, triangle),
        &ExactReal::from(0),
    )
    .value()
    .is_some_and(|ordering| ordering != Ordering::Equal)
}

fn point_lies_strictly_between(
    projected: &[hypertri::ExactPoint],
    point: usize,
    start: usize,
    end: usize,
) -> bool {
    if point == start
        || point == end
        || projected
            .get(point)
            .zip(projected.get(start))
            .is_none_or(|(point, start)| exact_points_equal(point, start))
        || projected
            .get(point)
            .zip(projected.get(end))
            .is_none_or(|(point, end)| exact_points_equal(point, end))
    {
        return false;
    }
    let Some(point) = projected.get(point) else {
        return false;
    };
    let Some(start) = projected.get(start) else {
        return false;
    };
    let Some(end) = projected.get(end) else {
        return false;
    };
    let area = ((start.x.clone() * &point.y) - &(start.y.clone() * &point.x))
        + &((point.x.clone() * &end.y) - &(point.y.clone() * &end.x))
        + &((end.x.clone() * &start.y) - &(end.y.clone() * &start.x));
    compare_reals(&area, &ExactReal::from(0)).value() == Some(Ordering::Equal)
        && real_between_inclusive(&point.x, &start.x, &end.x)
        && real_between_inclusive(&point.y, &start.y, &end.y)
}

fn real_between_inclusive(value: &ExactReal, start: &ExactReal, end: &ExactReal) -> bool {
    let Some(start_to_end) = compare_reals(start, end).value() else {
        return false;
    };
    match start_to_end {
        Ordering::Less | Ordering::Equal => {
            compare_reals(start, value)
                .value()
                .is_some_and(|ordering| ordering != Ordering::Greater)
                && compare_reals(value, end)
                    .value()
                    .is_some_and(|ordering| ordering != Ordering::Greater)
        }
        Ordering::Greater => {
            compare_reals(end, value)
                .value()
                .is_some_and(|ordering| ordering != Ordering::Greater)
                && compare_reals(value, start)
                    .value()
                    .is_some_and(|ordering| ordering != Ordering::Greater)
        }
    }
}

/// Lift a projected CDT Steiner point onto the exact source face plane.
///
/// The formula mirrors the face-cell CDT lift used elsewhere in hypermesh: the
/// chosen projection has exact nonzero area, so the omitted coordinate is
/// recovered from the retained source plane equation.  The result must
/// reproject to the original CDT point before the boolmesh export stage can use
/// it as topology, following Yap's exact object boundary.
fn lift_projected_boolmesh_steiner(
    mesh: &ExactMesh,
    face: usize,
    projection: hyperlimit::CoplanarProjection,
    point: &hypertri::ExactPoint,
) -> Option<ExactPoint3> {
    let triangle = mesh.triangles().get(face)?.0;
    let a = mesh.vertices().get(triangle[0])?.to_hyperlimit_point();
    let b = mesh.vertices().get(triangle[1])?.to_hyperlimit_point();
    let c = mesh.vertices().get(triangle[2])?.to_hyperlimit_point();
    let ab = point3_sub(&b, &a);
    let ac = point3_sub(&c, &a);
    let normal = cross(&ab, &ac);
    let plane_value = add_real(
        &add_real(&mul_real(&normal.x, &a.x), &mul_real(&normal.y, &a.y)),
        &mul_real(&normal.z, &a.z),
    );

    let lifted = match projection {
        hyperlimit::CoplanarProjection::Xy => {
            let x = point.x.clone();
            let y = point.y.clone();
            let z = div_real(
                sub_real(
                    &sub_real(&plane_value, &mul_real(&normal.x, &x)),
                    &mul_real(&normal.y, &y),
                ),
                &normal.z,
            )?;
            Point3::new(x, y, z)
        }
        hyperlimit::CoplanarProjection::Xz => {
            let x = point.x.clone();
            let z = point.y.clone();
            let y = div_real(
                sub_real(
                    &sub_real(&plane_value, &mul_real(&normal.x, &x)),
                    &mul_real(&normal.z, &z),
                ),
                &normal.y,
            )?;
            Point3::new(x, y, z)
        }
        hyperlimit::CoplanarProjection::Yz => {
            let y = point.x.clone();
            let z = point.y.clone();
            let x = div_real(
                sub_real(
                    &sub_real(&plane_value, &mul_real(&normal.y, &y)),
                    &mul_real(&normal.z, &z),
                ),
                &normal.x,
            )?;
            Point3::new(x, y, z)
        }
    };
    exact_points_equal(&project_for_hypertri(&lifted, projection), point)
        .then(|| ExactPoint3::new(lifted.x, lifted.y, lifted.z))
}

fn partition_polygon_components(rings: Vec<ProjectedLoop>) -> Option<Vec<RingComponent>> {
    if rings.is_empty() {
        return Some(Vec::new());
    }

    let mut parents = vec![None; rings.len()];
    for child in 0..rings.len() {
        let mut parent = None;
        for candidate in 0..rings.len() {
            if child == candidate {
                continue;
            }
            match point_in_projected_ring(&rings[candidate].projected, &rings[child].projected[0])?
            {
                RingPointLocation::Outside => {}
                RingPointLocation::Boundary => return None,
                RingPointLocation::Inside => {
                    parent = match parent {
                        None => Some(candidate),
                        Some(current) => match compare_reals(
                            &rings[candidate].area_abs,
                            &rings[current].area_abs,
                        )
                        .value()?
                        {
                            Ordering::Less => Some(candidate),
                            Ordering::Equal | Ordering::Greater => Some(current),
                        },
                    };
                }
            }
        }
        parents[child] = parent;
    }

    let depths = (0..rings.len())
        .map(|index| ring_depth(index, &parents))
        .collect::<Option<Vec<_>>>()?;
    let mut components = Vec::new();
    let mut exterior_to_component = BTreeMap::new();
    for index in 0..rings.len() {
        if depths[index] % 2 == 0 {
            exterior_to_component.insert(index, components.len());
            components.push(RingComponent {
                exterior: rings[index].clone(),
                holes: Vec::new(),
            });
        }
    }
    for index in 0..rings.len() {
        if depths[index] % 2 == 1 {
            let exterior = parents[index]?;
            let component = exterior_to_component.get(&exterior).copied()?;
            components[component].holes.push(rings[index].clone());
        }
    }
    Some(components)
}

fn ring_depth(index: usize, parents: &[Option<usize>]) -> Option<usize> {
    let mut depth = 0;
    let mut current = index;
    let mut seen = Vec::new();
    while let Some(parent) = parents[current] {
        if seen.contains(&parent) {
            return None;
        }
        seen.push(parent);
        depth += 1;
        current = parent;
    }
    Some(depth)
}

fn point_in_projected_ring(
    ring: &[hypertri::ExactPoint],
    point: &hypertri::ExactPoint,
) -> Option<RingPointLocation> {
    let ring = ring
        .iter()
        .map(|point| Point2::new(point.x.clone(), point.y.clone()))
        .collect::<Vec<_>>();
    let point = Point2::new(point.x.clone(), point.y.clone());
    classify_point_ring_even_odd(&ring, &point).value()
}

/// Drop rings already covered by a larger retained boundary ring.
///
/// Legacy boolmesh's `EarClip::new` calls `clip_degenerate` before
/// `cut_key_hole`: a coplanar overlap seam whose projected ring vertices are
/// all already present on a larger circular list is clipped as boundary
/// degeneracy instead of being bridged as a geometric hole or triangulated as a
/// separate contour.  This exact pre-filter ports that behavior while keeping
/// the dropped loop ids replayable for validation.  The separation follows
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997): exact predicate decisions select topology before any polygon
/// triangulation output is trusted.
fn clip_boundary_covered_rings(rings: Vec<ProjectedLoop>) -> (Vec<ProjectedLoop>, Vec<usize>) {
    if rings.len() < 2 {
        return (rings, Vec::new());
    }
    let mut active = Vec::with_capacity(rings.len());
    let mut clipped = Vec::new();
    for ring in &rings {
        if rings.iter().any(|candidate| {
            candidate.loop_index != ring.loop_index
                && compare_reals(&candidate.area_abs, &ring.area_abs).value()
                    == Some(Ordering::Greater)
                && projected_ring_vertices_are_covered_by(&ring.projected, &candidate.projected)
        }) {
            clipped.push(ring.loop_index);
        } else {
            active.push(ring.clone());
        }
    }
    (active, clipped)
}

fn projected_ring_vertices_are_covered_by(
    ring: &[hypertri::ExactPoint],
    boundary: &[hypertri::ExactPoint],
) -> bool {
    ring.iter().all(|point| {
        boundary
            .iter()
            .any(|candidate| exact_points_equal(point, candidate))
    })
}

fn exact_points_equal(left: &hypertri::ExactPoint, right: &hypertri::ExactPoint) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
}

#[cfg(any(test, feature = "internal-fuzzing"))]
fn cdt_crossing_constraint_steiner_lift_probe() -> bool {
    fn p(x: i64, y: i64) -> hypertri::ExactPoint {
        hypertri::ExactPoint::new(ExactReal::from(x), ExactReal::from(y))
    }

    let Ok(source) = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        crate::exact::validation::ValidationPolicy::ALLOW_BOUNDARY,
    ) else {
        return false;
    };
    let Some((triangles, constraint_edges, steiner_points)) = triangulate_component_with_cdt(
        &[p(0, 0), p(2, 2), p(0, 2), p(2, 0)],
        &[],
        &source,
        0,
        hyperlimit::CoplanarProjection::Xy,
    ) else {
        return false;
    };

    let Some(lifted) = steiner_points.first().map(ExactPoint3::to_hyperlimit_point) else {
        return false;
    };
    steiner_points.len() == 1
        && compare_reals(&lifted.x, &ExactReal::from(1)).value() == Some(Ordering::Equal)
        && compare_reals(&lifted.y, &ExactReal::from(1)).value() == Some(Ordering::Equal)
        && compare_reals(&lifted.z, &ExactReal::from(0)).value() == Some(Ordering::Equal)
        && triangles.contains(&4)
        && triangles.iter().all(|index| *index < 5)
        && constraint_edges.iter().any(|edge| edge.contains(&4))
}

fn projected_area2_abs(points: &[hypertri::ExactPoint]) -> Option<ExactReal> {
    let signed = projected_area2_signed(points);
    match compare_reals(&signed, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(ExactReal::from(0) - &signed),
        Ordering::Equal | Ordering::Greater => Some(signed),
    }
}

fn projected_area2_signed(points: &[hypertri::ExactPoint]) -> ExactReal {
    let mut signed = ExactReal::from(0);
    for index in 0..points.len() {
        let current = &points[index];
        let next = &points[(index + 1) % points.len()];
        signed += &((current.x.clone() * &next.y) - &(current.y.clone() * &next.x));
    }
    signed
}

fn triangle_area2_signed(points: &[hypertri::ExactPoint], triangle: [usize; 3]) -> ExactReal {
    let a = &points[triangle[0]];
    let b = &points[triangle[1]];
    let c = &points[triangle[2]];
    ((a.x.clone() * &b.y) - &(a.y.clone() * &b.x))
        + &((b.x.clone() * &c.y) - &(b.y.clone() * &c.x))
        + &((c.x.clone() * &a.y) - &(c.y.clone() * &a.x))
}

fn loop_source_face(
    halfedge_slot: Option<usize>,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
) -> Option<(ExactBoolMeshSide, usize)> {
    let source = &halfedges
        .output_halfedges
        .get(halfedge_slot?)?
        .as_ref()?
        .source;
    match source {
        ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge {
            side, source_face, ..
        }
        | ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
            side, source_face, ..
        }
        | ExactBoolMeshOutputHalfedgeSource::NewFacePair {
            side, source_face, ..
        } => Some((*side, *source_face)),
    }
}

fn source_mesh<'a>(
    side: ExactBoolMeshSide,
    face: usize,
    left: &'a ExactMesh,
    right: &'a ExactMesh,
) -> Option<&'a ExactMesh> {
    let mesh = match side {
        ExactBoolMeshSide::Left => left,
        ExactBoolMeshSide::Right => right,
    };
    (face < mesh.triangles().len()).then_some(mesh)
}

fn output_loop_points(
    vertices: &[usize],
    allocation: &ExactBoolMeshOutputVertexAllocation,
    boolean03: &ExactBoolMeshBoolean03,
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<Vec<Point3>> {
    vertices
        .iter()
        .map(|vertex| output_vertex_point(*vertex, allocation, boolean03, left, right))
        .collect()
}

/// Exercise exact `boolean45` simple-loop triangulation branches for fuzz builds.
///
/// The normal boolmesh workspace owns face-loop construction.  This probe is
/// intentionally gated behind `internal-fuzzing` so adversarial builds can keep
/// the direct ports of boolmesh `single_triangulate` and `square_triangulate`
/// compiled and checked without exporting a partial triangulation API.
#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    fn p(x: i64, y: i64) -> hypertri::ExactPoint {
        hypertri::ExactPoint::new(ExactReal::from(x), ExactReal::from(y))
    }

    match selector % 4 {
        0 => triangulate_simple_component(&[p(0, 0), p(2, 0), p(0, 2)]) == Some(vec![0, 1, 2]),
        1 => {
            triangulate_simple_component(&[p(0, 0), p(3, 0), p(4, 2), p(0, 1)])
                == Some(vec![1, 2, 3, 0, 1, 3])
        }
        2 => {
            triangulate_simple_component(&[p(0, 0), p(4, 0), p(3, 1), p(0, 2)])
                == Some(vec![0, 1, 2, 0, 2, 3])
        }
        _ => cdt_crossing_constraint_steiner_lift_probe(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::validation::ValidationPolicy;
    use crate::exact::{
        ExactBoolMeshOutputFaceLoop, ExactBoolMeshOutputHalfedge,
        ExactBoolMeshOutputHalfedgeSource, ExactBoolMeshOutputVertexOrigin,
        ExactBoolMeshSourceVertex,
    };

    fn planar_source() -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
                3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn planar_source_with_collinear_vertex() -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
                3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0, //
                5, 0, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn planar_source_with_disjoint_squares() -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
                20, 0, 0, 30, 0, 0, 30, 10, 0, 20, 10, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn planar_source_with_irregular_quad() -> ExactMesh {
        ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, //
                3, 0, 0, //
                4, 2, 0, //
                0, 1, 0,
            ],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn empty_mesh() -> ExactMesh {
        ExactMesh::from_i64_triangles(&[], &[]).unwrap()
    }

    fn p(x: i64, y: i64) -> hypertri::ExactPoint {
        hypertri::ExactPoint::new(ExactReal::from(x), ExactReal::from(y))
    }

    fn q(xn: i64, xd: i64, yn: i64, yd: i64) -> hypertri::ExactPoint {
        hypertri::ExactPoint::new(
            (ExactReal::from(xn) / &ExactReal::from(xd)).expect("nonzero denominator"),
            (ExactReal::from(yn) / &ExactReal::from(yd)).expect("nonzero denominator"),
        )
    }

    fn source_allocation(vertex_count: usize) -> ExactBoolMeshOutputVertexAllocation {
        ExactBoolMeshOutputVertexAllocation {
            left_vertex_output_starts: (0..vertex_count).map(Some).collect(),
            right_vertex_output_starts: Vec::new(),
            p1q2_output_starts: Vec::new(),
            p2q1_output_starts: Vec::new(),
            output_vertex_origins: (0..vertex_count)
                .map(|vertex| ExactBoolMeshOutputVertexOrigin::SourceVertex {
                    source: ExactBoolMeshSourceVertex {
                        side: ExactBoolMeshSide::Left,
                        vertex,
                    },
                    copy: 0,
                })
                .collect(),
        }
    }

    fn empty_boolean03(left_vertices: usize) -> ExactBoolMeshBoolean03 {
        ExactBoolMeshBoolean03 {
            p1q2: Vec::new(),
            p2q1: Vec::new(),
            x12: Vec::new(),
            x21: Vec::new(),
            v12: Vec::new(),
            v21: Vec::new(),
            w03: vec![0; left_vertices],
            w30: Vec::new(),
        }
    }

    fn face_halfedges(
        vertices: &[usize],
        start: usize,
    ) -> Vec<Option<ExactBoolMeshOutputHalfedge>> {
        vertices
            .iter()
            .enumerate()
            .map(|(local, &tail)| {
                let head = vertices[(local + 1) % vertices.len()];
                Some(ExactBoolMeshOutputHalfedge {
                    tail,
                    head,
                    pair: start + local,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
                        side: ExactBoolMeshSide::Left,
                        source_halfedge: local,
                        source_face: 0,
                        edge: [tail, head],
                        fragment: 0,
                        forward: true,
                    },
                })
            })
            .collect()
    }

    fn new_face_pair_halfedge(
        tail: usize,
        head: usize,
        face: usize,
        slot: usize,
    ) -> Option<ExactBoolMeshOutputHalfedge> {
        Some(ExactBoolMeshOutputHalfedge {
            tail,
            head,
            pair: slot,
            face,
            source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                side: ExactBoolMeshSide::Left,
                source_face: face,
                opposite_face: 0,
                fragment: slot,
                forward: true,
            },
        })
    }

    #[test]
    fn holed_face_refines_collinear_hole_boundary_edges_after_earcut() {
        let projected = vec![
            p(7, 1),
            p(1, 7),
            p(1, 1),
            q(14, 5, 14, 5),
            q(18, 5, 2, 1),
            q(30, 11, 2, 1),
            p(2, 2),
            q(2, 1, 18, 5),
        ];
        let constraints = [
            hypertri::Constraint::new(0, 1),
            hypertri::Constraint::new(1, 2),
            hypertri::Constraint::new(2, 0),
            hypertri::Constraint::new(3, 4),
            hypertri::Constraint::new(4, 5),
            hypertri::Constraint::new(5, 6),
            hypertri::Constraint::new(6, 7),
            hypertri::Constraint::new(7, 3),
        ];

        assert!(hypertri::cdt::constrained_delaunay(&projected, &constraints).is_err());
        let triangles =
            triangulate_polygon_with_boundary_refinement(&projected, &[3], &constraints)
                .expect("fallback should split collinear hole boundary edges");

        for constraint in constraints {
            assert!(
                triangles_have_edge(&triangles, constraint.from, constraint.to),
                "missing refined constraint {constraint:?} from {triangles:?}"
            );
        }
    }

    #[test]
    fn simple_triangle_uses_boolmesh_single_triangulate_branch() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let output_halfedges = face_halfedges(&[0, 1, 2], 0);
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![ExactBoolMeshOutputFaceLoop {
                output_face: 0,
                halfedges: vec![0, 1, 2],
                vertices: vec![0, 1, 2],
            }],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.triangulations.len(), 1);
        let triangulation = &stage.triangulations[0];
        assert_eq!(triangulation.vertices, vec![0, 1, 2]);
        assert_eq!(triangulation.triangles, vec![0, 1, 2]);
        assert!(triangulation.constraint_edges.is_empty());
    }

    #[test]
    fn quadrilateral_uses_exact_boolmesh_square_diagonal_rule() {
        let left = planar_source_with_irregular_quad();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let output_halfedges = face_halfedges(&[0, 1, 2, 3], 0);
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![ExactBoolMeshOutputFaceLoop {
                output_face: 0,
                halfedges: vec![0, 1, 2, 3],
                vertices: vec![0, 1, 2, 3],
            }],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.triangulations.len(), 1);
        let triangulation = &stage.triangulations[0];
        assert_eq!(triangulation.vertices, vec![0, 1, 2, 3]);
        assert_eq!(
            triangulation.triangles,
            vec![1, 2, 3, 0, 1, 3],
            "boolmesh square_triangulate should avoid the longer exact (0,2) diagonal"
        );
        assert!(triangulation.constraint_edges.is_empty());
    }

    #[test]
    fn triangulates_holed_face_even_when_hole_loop_arrives_first() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[4, 5, 6, 7], 0);
        output_halfedges.extend(face_halfedges(&[0, 1, 2, 3], 4));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2, 3],
                    vertices: vec![4, 5, 6, 7],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![4, 5, 6, 7],
                    vertices: vec![0, 1, 2, 3],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.multi_loop_faces, 0);
        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.triangulations.len(), 1);
        let triangulation = &stage.triangulations[0];
        assert_eq!(triangulation.loop_index, 1);
        assert_eq!(triangulation.component_loop_indices, vec![1, 0]);
        assert_eq!(triangulation.constraint_edges.len(), 8);
        assert_eq!(triangulation.vertices, vec![0, 1, 2, 3, 4, 5, 6, 7]);
        assert_eq!(triangulation.triangles.len(), 24);
        assert!(triangulation.triangles.iter().all(|index| *index < 8));
    }

    #[test]
    fn cdt_steiner_vertices_are_lifted_to_exact_boolmesh_output_points() {
        assert!(
            cdt_crossing_constraint_steiner_lift_probe(),
            "crossing CDT constraints must append, lift, and replay an exact Steiner vertex"
        );
    }

    #[test]
    fn holed_face_ignores_short_ring_when_positive_area_ring_remains() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[0, 1, 2, 3], 0);
        output_halfedges.extend(face_halfedges(&[4, 5], 4));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2, 3],
                    vertices: vec![0, 1, 2, 3],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![4, 5],
                    vertices: vec![4, 5],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.short_loops, 0);
        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.multi_loop_faces, 0);
        assert_eq!(stage.triangulations.len(), 1);
        let triangulation = &stage.triangulations[0];
        assert_eq!(triangulation.loop_index, 0);
        assert_eq!(triangulation.component_loop_indices, vec![0]);
        assert!(triangulation.constraint_edges.is_empty());
        assert_eq!(triangulation.vertices, vec![0, 1, 2, 3]);
        assert_eq!(triangulation.triangles.len(), 6);
        assert!(triangulation.triangles.iter().all(|index| *index < 4));
    }

    #[test]
    fn boundary_covered_hole_ring_is_clipped_before_hypertri() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[0, 1, 5, 4, 5, 6, 2, 3], 0);
        output_halfedges.extend(face_halfedges(&[1, 5, 4], 8));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2, 3, 4, 5, 6, 7],
                    vertices: vec![0, 1, 5, 4, 5, 6, 2, 3],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![8, 9, 10],
                    vertices: vec![1, 5, 4],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.short_loops, 0);
        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.triangulations.len(), 1);
        let triangulation = &stage.triangulations[0];
        assert_eq!(triangulation.loop_index, 0);
        assert_eq!(triangulation.clipped_loop_indices, vec![1]);
        assert_eq!(triangulation.component_loop_indices, vec![0]);
        assert_eq!(triangulation.vertices, vec![0, 1, 5, 4, 5, 6, 2, 3]);
        assert!(triangulation.triangles.iter().all(|index| *index < 8));
    }

    #[test]
    fn disjoint_positive_area_loops_remain_separate_boolmesh_components() {
        let left = planar_source_with_disjoint_squares();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[0, 1, 2, 3], 0);
        output_halfedges.extend(face_halfedges(&[4, 5, 6, 7], 4));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2, 3],
                    vertices: vec![0, 1, 2, 3],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![4, 5, 6, 7],
                    vertices: vec![4, 5, 6, 7],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.multi_loop_faces, 0);
        assert_eq!(stage.triangulations.len(), 2);
        assert!(
            stage
                .triangulations
                .iter()
                .all(|triangulation| triangulation.output_face == 0
                    && triangulation.component_loop_indices.len() == 1
                    && triangulation.triangles.len() == 6
                    && triangulation.constraint_edges.is_empty())
        );
        assert_eq!(stage.triangulations[0].vertices, vec![0, 1, 2, 3]);
        assert_eq!(stage.triangulations[1].vertices, vec![4, 5, 6, 7]);
    }

    #[test]
    fn exact_zero_area_face_is_dropped_instead_of_blocking_triangulation() {
        let left = planar_source_with_collinear_vertex();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let output_halfedges = face_halfedges(&[0, 8, 1], 0);
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![ExactBoolMeshOutputFaceLoop {
                output_face: 0,
                halfedges: vec![0, 1, 2],
                vertices: vec![0, 8, 1],
            }],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert!(stage.triangulations.is_empty());
        assert_eq!(stage.short_loops, 0);
        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.dropped_degenerate_faces, vec![0]);
    }

    #[test]
    fn exact_zero_area_hole_ring_is_replayed_as_clipped_when_area_remains() {
        let left = planar_source_with_collinear_vertex();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[0, 1, 2, 3], 0);
        output_halfedges.extend(face_halfedges(&[0, 8, 1], 4));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2, 3],
                    vertices: vec![0, 1, 2, 3],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![4, 5, 6],
                    vertices: vec![0, 8, 1],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.dropped_degenerate_faces, Vec::<usize>::new());
        assert_eq!(stage.triangulation_failures, 0);
        assert_eq!(stage.triangulations.len(), 1);
        let triangulation = &stage.triangulations[0];
        assert_eq!(triangulation.loop_index, 0);
        assert_eq!(triangulation.clipped_loop_indices, vec![1]);
        assert_eq!(triangulation.vertices, vec![0, 1, 2, 3]);
        assert_eq!(triangulation.triangles.len(), 6);
    }

    #[test]
    fn all_short_face_pair_seams_are_dropped_as_degenerate_faces() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![
                new_face_pair_halfedge(0, 0, 0, 0),
                new_face_pair_halfedge(1, 1, 0, 1),
            ],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0],
                    vertices: vec![0],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![1],
                    vertices: vec![1],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert!(stage.triangulations.is_empty());
        assert_eq!(stage.short_loops, 0);
        assert_eq!(stage.dropped_degenerate_faces, vec![0]);
    }

    #[test]
    fn positive_face_clips_short_face_pair_seam_loops() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let mut output_halfedges = face_halfedges(&[0, 1, 2], 0);
        output_halfedges.push(new_face_pair_halfedge(3, 3, 0, 3));
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![0, 1, 2],
                    vertices: vec![0, 1, 2],
                },
                ExactBoolMeshOutputFaceLoop {
                    output_face: 0,
                    halfedges: vec![3],
                    vertices: vec![3],
                },
            ],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.short_loops, 0);
        assert_eq!(stage.dropped_degenerate_faces, Vec::<usize>::new());
        assert_eq!(stage.triangulations.len(), 1);
        assert_eq!(stage.triangulations[0].clipped_loop_indices, vec![1]);
    }

    #[test]
    fn all_short_face_remains_blocked() {
        let left = planar_source();
        let right = empty_mesh();
        let boolean03 = empty_boolean03(left.vertices().len());
        let allocation = source_allocation(left.vertices().len());
        let output_halfedges = face_halfedges(&[0, 1], 0);
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges,
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };
        let face_loops = ExactBoolMeshFaceLoopAssemblyStage {
            loops: vec![ExactBoolMeshOutputFaceLoop {
                output_face: 0,
                halfedges: vec![0, 1],
                vertices: vec![0, 1],
            }],
            ..ExactBoolMeshFaceLoopAssemblyStage::default()
        };

        let stage = triangulate_output_face_loops(
            &left,
            &right,
            &boolean03,
            &allocation,
            &halfedges,
            &face_loops,
        );

        assert_eq!(stage.triangulations.len(), 0);
        assert_eq!(stage.short_loops, 1);
        assert_eq!(stage.multi_loop_faces, 0);
    }
}
