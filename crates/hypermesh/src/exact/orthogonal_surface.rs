//! Exact orthogonal-cell arrangements for coplanar surface booleans.
//!
//! This module is a bounded general planar-arrangement step for `hypermesh`.
//! The accepted operands are coplanar axis-aligned rectangular surface pieces on
//! one coordinate plane. They are lowered into an exact rectilinear cell complex,
//! the requested boolean is evaluated per cell, and retained boundary loops are
//! extracted before `hypertri` sees any polygon. That ordering follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997):
//! combinatorial topology is promoted from exact object structure, not from a
//! primitive-float polygon repair pass.
//!
//! The grid subdivision is the orthogonal analogue of the arrangement viewpoint
//! in de Berg, Cheong, van Kreveld, and Overmars, *Computational Geometry:
//! Algorithms and Applications*, 3rd ed. (2008), Chapter 2. Earcut is only the
//! final triangulation handoff; retained loops and cell occupancy are the
//! certified topology artifact.

use core::cmp::Ordering;

use hyperlimit::{
    Point2, Point3, SegmentIntersection, Sign, classify_segment_intersection, compare_reals,
    orient2d_report, project_point3 as project_point,
};

use super::coplanar::CoplanarProjection;
use super::error::{DiagnosticKind, MeshDiagnostic, MeshError, Severity};
use super::mesh::{ExactMesh, ExactPoint3, Triangle};
use super::provenance::SourceProvenance;
use super::scalar::ExactReal;
use super::validation::ValidationPolicy;

/// Boolean operation handled by the orthogonal coplanar surface cell complex.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CoplanarOrthogonalSurfaceOperation {
    /// Materialize the union of occupied rectangular surface cells.
    Union,
    /// Materialize the positive-area intersection of occupied surface cells.
    Intersection,
    /// Materialize cells occupied by the left operand and not by the right.
    Difference,
}

/// One connected orthogonal output component.
///
/// The outer loop is retained counter-clockwise and holes are retained
/// clockwise. The loops are exact topology evidence for the cell complex. They
/// are not reconstructed from the triangulated mesh after the fact; that keeps
/// the output aligned with Yap's exact-object boundary.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarOrthogonalSurfaceComponent {
    /// Counter-clockwise outer boundary of this connected component.
    pub outer: Vec<Point3>,
    /// Clockwise hole boundaries strictly inside [`Self::outer`].
    pub holes: Vec<Vec<Point3>>,
}

/// Exact orthogonal-cell coplanar surface arrangement.
///
/// This artifact represents bounded rectilinear arrangements that are wider
/// than the convex/simple-loop shortcuts: multi-component outputs, nonconvex
/// outer loops, retained holes, and cutter/hole contact graphs are accepted when
/// an exact axis-aligned cell replay proves the topology. Cases outside that
/// model stay explicit planar-arrangement work instead of being decided by a
/// tolerance polygon kernel.
#[derive(Clone, Debug, PartialEq)]
pub struct CoplanarOrthogonalSurfaceArrangement {
    /// Projection plane used by the exact cell grid.
    pub projection: CoplanarProjection,
    /// Boolean operation used to produce the retained cells.
    pub operation: CoplanarOrthogonalSurfaceOperation,
    /// Retained connected components with optional holes.
    pub components: Vec<CoplanarOrthogonalSurfaceComponent>,
    /// Exact triangulated open surface mesh for the retained components.
    pub mesh: ExactMesh,
}

impl CoplanarOrthogonalSurfaceArrangement {
    /// Validate retained loops, hole nesting, mesh area, and exact mesh state.
    ///
    /// The validation is intentionally about the retained artifact, not only the
    /// triangle soup: loops must be simple, oriented, mutually disjoint, and the
    /// triangulated mesh must replay their signed area. This mirrors Yap's
    /// requirement that exact computation preserve enough combinatorial history
    /// to audit later decisions.
    pub fn validate(&self) -> Result<(), MeshError> {
        validate_components(self.projection, &self.components)?;
        validate_component_mesh(self.projection, &self.components, &self.mesh)?;
        self.mesh
            .validate_retained_state()
            .map_err(|err| orthogonal_error(format!("orthogonal output mesh is stale: {err:?}")))?;
        Ok(())
    }

    /// Validate this arrangement by replaying it from the original sources.
    ///
    /// A locally valid cell complex cannot be transplanted to another boolean
    /// request. Rebuilding the exact grid from the source meshes keeps the
    /// retained loops tied to the exact rectangular source objects that produced
    /// them, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn validate_against_sources(
        &self,
        left: &ExactMesh,
        right: &ExactMesh,
    ) -> Result<(), MeshError> {
        self.validate()?;
        let replay = arrange_coplanar_orthogonal_surface(left, right, self.operation)
            .ok_or_else(|| orthogonal_error("source replay did not reproduce arrangement"))?;
        if self == &replay {
            Ok(())
        } else {
            Err(orthogonal_error(
                "retained orthogonal arrangement does not match source replay",
            ))
        }
    }
}

/// Certify and materialize an orthogonal coplanar surface union.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_orthogonal_surface_union(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarOrthogonalSurfaceArrangement> {
    arrange_coplanar_orthogonal_surface(left, right, CoplanarOrthogonalSurfaceOperation::Union)
}

/// Certify and materialize an orthogonal coplanar surface intersection.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_orthogonal_surface_intersection(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarOrthogonalSurfaceArrangement> {
    arrange_coplanar_orthogonal_surface(
        left,
        right,
        CoplanarOrthogonalSurfaceOperation::Intersection,
    )
}

/// Certify and materialize an orthogonal coplanar surface difference.
#[cfg(feature = "exact-triangulation")]
pub fn arrange_coplanar_orthogonal_surface_difference(
    left: &ExactMesh,
    right: &ExactMesh,
) -> Option<CoplanarOrthogonalSurfaceArrangement> {
    arrange_coplanar_orthogonal_surface(left, right, CoplanarOrthogonalSurfaceOperation::Difference)
}

#[cfg(feature = "exact-triangulation")]
fn arrange_coplanar_orthogonal_surface(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: CoplanarOrthogonalSurfaceOperation,
) -> Option<CoplanarOrthogonalSurfaceArrangement> {
    let left_rectangles = extract_axis_aligned_rectangles(left)?;
    let right_rectangles = extract_axis_aligned_rectangles(right)?;
    let projection = left_rectangles.projection;
    if right_rectangles.projection != projection
        || !real_equal(&left_rectangles.dropped, &right_rectangles.dropped)
    {
        return None;
    }

    let cell_complex = build_orthogonal_cell_complex(
        projection,
        &left_rectangles.rectangles,
        &right_rectangles.rectangles,
        operation,
    )?;
    let components = extract_components_from_cells(&cell_complex)?;
    if components.is_empty() {
        return None;
    }
    let mesh = components_to_mesh(&components, projection)?;
    let arrangement = CoplanarOrthogonalSurfaceArrangement {
        projection,
        operation,
        components,
        mesh,
    };
    arrangement.validate().ok()?;
    Some(arrangement)
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct RectangleExtraction {
    projection: CoplanarProjection,
    dropped: ExactReal,
    rectangles: Vec<ProjectedRectangle>,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct ProjectedRectangle {
    min: Point2,
    max: Point2,
    dropped: ExactReal,
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct RectangleGroup {
    rectangle: ProjectedRectangle,
    area2: ExactReal,
    corners: Vec<Point2>,
    triangles: usize,
}

/// Extract exact rectangular source cells from a triangulated open surface.
///
/// The accepted input grammar is deliberately small: every triangle must be one
/// half of an axis-aligned rectangle on a shared coordinate plane, and grouped
/// halves must replay to exactly one rectangle area. This is the same
/// proof-before-promotion discipline as Yap's EGC model; a more general
/// rectilinear polygon import should arrive as its own certified source object
/// rather than as guessed triangle topology.
#[cfg(feature = "exact-triangulation")]
fn extract_axis_aligned_rectangles(mesh: &ExactMesh) -> Option<RectangleExtraction> {
    if mesh.triangles().is_empty() {
        return None;
    }
    let points = mesh
        .vertices()
        .iter()
        .map(ExactPoint3::to_hyperlimit_point)
        .collect::<Vec<_>>();
    let (projection, dropped) = choose_coordinate_plane(&points)?;
    let mut groups = Vec::<RectangleGroup>::new();
    for triangle in mesh.triangles() {
        let tri_points = triangle
            .0
            .iter()
            .map(|&index| points.get(index).cloned())
            .collect::<Option<Vec<_>>>()?;
        let rectangle = triangle_axis_aligned_rectangle(&tri_points, projection, &dropped)?;
        let area2 = projected_area2_abs(&tri_points, projection)?;
        let Some(group) = groups
            .iter_mut()
            .find(|group| rectangles_equal(&group.rectangle, &rectangle))
        else {
            groups.push(RectangleGroup {
                rectangle,
                area2,
                corners: tri_points
                    .iter()
                    .map(|point| project_point(point, projection))
                    .collect(),
                triangles: 1,
            });
            continue;
        };
        group.area2 = add(&group.area2, &area2);
        for point in tri_points
            .iter()
            .map(|point| project_point(point, projection))
        {
            if !group
                .corners
                .iter()
                .any(|corner| point2_equal(corner, &point))
            {
                group.corners.push(point);
            }
        }
        group.triangles += 1;
    }

    let mut rectangles = Vec::with_capacity(groups.len());
    for group in groups {
        let rect_area2 = rectangle_area2(&group.rectangle);
        if group.triangles < 2
            || group.corners.len() != 4
            || compare_reals(&group.area2, &rect_area2).value() != Some(Ordering::Equal)
        {
            return None;
        }
        rectangles.push(group.rectangle);
    }
    rectangles.sort_by(|left, right| {
        compare_point2(&left.min, &right.min)
            .and_then(|ordering| {
                if ordering == Ordering::Equal {
                    compare_point2(&left.max, &right.max)
                } else {
                    Some(ordering)
                }
            })
            .unwrap_or(Ordering::Equal)
    });
    Some(RectangleExtraction {
        projection,
        dropped,
        rectangles,
    })
}

#[cfg(feature = "exact-triangulation")]
fn choose_coordinate_plane(points: &[Point3]) -> Option<(CoplanarProjection, ExactReal)> {
    let first = points.first()?;
    if points.iter().all(|point| real_equal(&point.z, &first.z)) {
        Some((CoplanarProjection::Xy, first.z.clone()))
    } else if points.iter().all(|point| real_equal(&point.y, &first.y)) {
        Some((CoplanarProjection::Xz, first.y.clone()))
    } else if points.iter().all(|point| real_equal(&point.x, &first.x)) {
        Some((CoplanarProjection::Yz, first.x.clone()))
    } else {
        None
    }
}

#[cfg(feature = "exact-triangulation")]
fn triangle_axis_aligned_rectangle(
    triangle: &[Point3],
    projection: CoplanarProjection,
    dropped: &ExactReal,
) -> Option<ProjectedRectangle> {
    if triangle.len() != 3
        || triangle
            .iter()
            .any(|point| !real_equal(&dropped_coordinate(point, projection), dropped))
    {
        return None;
    }
    let mut min = project_point(triangle.first()?, projection);
    let mut max = min.clone();
    for point in triangle
        .iter()
        .skip(1)
        .map(|point| project_point(point, projection))
    {
        if real_order(&point.x, &min.x)? == Ordering::Less {
            min.x = point.x.clone();
        }
        if real_order(&point.y, &min.y)? == Ordering::Less {
            min.y = point.y.clone();
        }
        if real_order(&point.x, &max.x)? == Ordering::Greater {
            max.x = point.x.clone();
        }
        if real_order(&point.y, &max.y)? == Ordering::Greater {
            max.y = point.y.clone();
        }
    }
    if real_order(&min.x, &max.x)? != Ordering::Less
        || real_order(&min.y, &max.y)? != Ordering::Less
    {
        return None;
    }
    let rectangle = ProjectedRectangle {
        min,
        max,
        dropped: dropped.clone(),
    };
    let corners = rectangle_corners2(&rectangle);
    let mut unique = Vec::new();
    for projected in triangle
        .iter()
        .map(|point| project_point(point, projection))
    {
        if !corners
            .iter()
            .any(|corner| point2_equal(corner, &projected))
        {
            return None;
        }
        if !unique.iter().any(|point| point2_equal(point, &projected)) {
            unique.push(projected);
        }
    }
    if unique.len() != 3 {
        return None;
    }
    let tri_area2 = projected_area2_abs(triangle, projection)?;
    let double_tri_area = add(&tri_area2, &tri_area2);
    if compare_reals(&double_tri_area, &rectangle_area2(&rectangle)).value()
        != Some(Ordering::Equal)
    {
        return None;
    }
    Some(rectangle)
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct OrthogonalCellComplex {
    projection: CoplanarProjection,
    dropped: ExactReal,
    xs: Vec<ExactReal>,
    ys: Vec<ExactReal>,
    y_cells: usize,
    occupied: Vec<bool>,
}

/// Build the exact cell complex and evaluate one boolean operation.
///
/// Each open cell is classified by the exact midpoint of its bounding interval.
/// Because all source boundaries are grid lines from exact rectangle
/// coordinates, midpoint membership is a certified cell-level predicate rather
/// than a sampling heuristic. This is the orthogonal-cell specialization of the
/// arrangement model in de Berg et al., with Yap's requirement that the decision
/// path remain explicit.
#[cfg(feature = "exact-triangulation")]
fn build_orthogonal_cell_complex(
    projection: CoplanarProjection,
    left: &[ProjectedRectangle],
    right: &[ProjectedRectangle],
    operation: CoplanarOrthogonalSurfaceOperation,
) -> Option<OrthogonalCellComplex> {
    if left.is_empty() || right.is_empty() {
        return None;
    }
    let dropped = left.first()?.dropped.clone();
    if left
        .iter()
        .chain(right.iter())
        .any(|rect| !real_equal(&rect.dropped, &dropped))
    {
        return None;
    }
    reject_positive_area_overlaps(left)?;
    reject_positive_area_overlaps(right)?;

    let mut xs = Vec::new();
    let mut ys = Vec::new();
    for rect in left.iter().chain(right.iter()) {
        xs.push(rect.min.x.clone());
        xs.push(rect.max.x.clone());
        ys.push(rect.min.y.clone());
        ys.push(rect.max.y.clone());
    }
    sort_reals_and_dedup(&mut xs)?;
    sort_reals_and_dedup(&mut ys)?;
    if xs.len() < 2 || ys.len() < 2 {
        return None;
    }

    let x_cells = xs.len() - 1;
    let y_cells = ys.len() - 1;
    let mut occupied = vec![false; x_cells * y_cells];
    for x in 0..x_cells {
        for y in 0..y_cells {
            if real_order(&xs[x], &xs[x + 1])? != Ordering::Less
                || real_order(&ys[y], &ys[y + 1])? != Ordering::Less
            {
                return None;
            }
            let midpoint = Point2::new(
                midpoint_real(&xs[x], &xs[x + 1]),
                midpoint_real(&ys[y], &ys[y + 1]),
            );
            let in_left = left
                .iter()
                .filter(|rect| point_strictly_inside_projected_rectangle(&midpoint, rect))
                .count();
            let in_right = right
                .iter()
                .filter(|rect| point_strictly_inside_projected_rectangle(&midpoint, rect))
                .count();
            if in_left > 1 || in_right > 1 {
                return None;
            }
            occupied[x * y_cells + y] = match operation {
                CoplanarOrthogonalSurfaceOperation::Union => in_left == 1 || in_right == 1,
                CoplanarOrthogonalSurfaceOperation::Intersection => in_left == 1 && in_right == 1,
                CoplanarOrthogonalSurfaceOperation::Difference => in_left == 1 && in_right == 0,
            };
        }
    }
    if !occupied.iter().any(|cell| *cell) {
        return None;
    }
    Some(OrthogonalCellComplex {
        projection,
        dropped,
        xs,
        ys,
        y_cells,
        occupied,
    })
}

#[cfg(feature = "exact-triangulation")]
fn reject_positive_area_overlaps(rectangles: &[ProjectedRectangle]) -> Option<()> {
    for left in 0..rectangles.len() {
        for right in left + 1..rectangles.len() {
            let x_min = exact_max_real(&rectangles[left].min.x, &rectangles[right].min.x)?;
            let x_max = exact_min_real(&rectangles[left].max.x, &rectangles[right].max.x)?;
            let y_min = exact_max_real(&rectangles[left].min.y, &rectangles[right].min.y)?;
            let y_max = exact_min_real(&rectangles[left].max.y, &rectangles[right].max.y)?;
            if real_order(&x_min, &x_max)? == Ordering::Less
                && real_order(&y_min, &y_max)? == Ordering::Less
            {
                return None;
            }
        }
    }
    Some(())
}

#[cfg(feature = "exact-triangulation")]
fn extract_components_from_cells(
    complex: &OrthogonalCellComplex,
) -> Option<Vec<CoplanarOrthogonalSurfaceComponent>> {
    let x_cells = complex.xs.len() - 1;
    let y_cells = complex.y_cells;
    let mut seen = vec![false; complex.occupied.len()];
    let mut components = Vec::new();
    for x in 0..x_cells {
        for y in 0..y_cells {
            let index = cell_index(x, y, y_cells);
            if seen[index] || !complex.occupied[index] {
                continue;
            }
            let mut stack = vec![(x, y)];
            let mut cells = Vec::new();
            seen[index] = true;
            while let Some((cx, cy)) = stack.pop() {
                cells.push((cx, cy));
                for (nx, ny) in cell_neighbors(cx, cy, x_cells, y_cells) {
                    let neighbor = cell_index(nx, ny, y_cells);
                    if !seen[neighbor] && complex.occupied[neighbor] {
                        seen[neighbor] = true;
                        stack.push((nx, ny));
                    }
                }
            }
            let mut loops = loops_for_component(complex, &cells)?;
            let mut outer = None;
            let mut holes = Vec::new();
            for mut loop_points in loops.drain(..) {
                loop_points = simplify_projected_polygon(loop_points, complex.projection);
                let signed = projected_area2_signed(&loop_points, complex.projection)?;
                match compare_reals(&signed, &ExactReal::from(0)).value()? {
                    Ordering::Greater => {
                        if outer.replace(loop_points).is_some() {
                            return None;
                        }
                    }
                    Ordering::Less => holes.push(loop_points),
                    Ordering::Equal => return None,
                }
            }
            let outer = outer?;
            holes.sort_by(|left, right| {
                compare_point2(
                    &polygon_min_projected_point(left, complex.projection),
                    &polygon_min_projected_point(right, complex.projection),
                )
                .unwrap_or(Ordering::Equal)
            });
            components.push(CoplanarOrthogonalSurfaceComponent { outer, holes });
        }
    }
    components.sort_by(|left, right| {
        compare_point2(
            &polygon_min_projected_point(&left.outer, complex.projection),
            &polygon_min_projected_point(&right.outer, complex.projection),
        )
        .unwrap_or(Ordering::Equal)
    });
    Some(components)
}

#[cfg(feature = "exact-triangulation")]
fn cell_neighbors(
    x: usize,
    y: usize,
    x_cells: usize,
    y_cells: usize,
) -> impl Iterator<Item = (usize, usize)> {
    let mut neighbors = Vec::with_capacity(4);
    if x > 0 {
        neighbors.push((x - 1, y));
    }
    if x + 1 < x_cells {
        neighbors.push((x + 1, y));
    }
    if y > 0 {
        neighbors.push((x, y - 1));
    }
    if y + 1 < y_cells {
        neighbors.push((x, y + 1));
    }
    neighbors.into_iter()
}

#[cfg(feature = "exact-triangulation")]
#[derive(Clone, Debug)]
struct DirectedFragment {
    start: Point3,
    end: Point3,
}

#[cfg(feature = "exact-triangulation")]
fn loops_for_component(
    complex: &OrthogonalCellComplex,
    cells: &[(usize, usize)],
) -> Option<Vec<Vec<Point3>>> {
    let mut fragments = Vec::new();
    for &(x, y) in cells {
        let x0 = &complex.xs[x];
        let x1 = &complex.xs[x + 1];
        let y0 = &complex.ys[y];
        let y1 = &complex.ys[y + 1];
        let bottom_empty = y == 0 || !complex.occupied[cell_index(x, y - 1, complex.y_cells)];
        let top_empty =
            y + 1 == complex.y_cells || !complex.occupied[cell_index(x, y + 1, complex.y_cells)];
        let left_empty = x == 0 || !complex.occupied[cell_index(x - 1, y, complex.y_cells)];
        let right_empty = x + 1 == complex.xs.len() - 1
            || !complex.occupied[cell_index(x + 1, y, complex.y_cells)];
        if bottom_empty {
            fragments.push(DirectedFragment {
                start: point_from_projection(x0, y0, &complex.dropped, complex.projection),
                end: point_from_projection(x1, y0, &complex.dropped, complex.projection),
            });
        }
        if right_empty {
            fragments.push(DirectedFragment {
                start: point_from_projection(x1, y0, &complex.dropped, complex.projection),
                end: point_from_projection(x1, y1, &complex.dropped, complex.projection),
            });
        }
        if top_empty {
            fragments.push(DirectedFragment {
                start: point_from_projection(x1, y1, &complex.dropped, complex.projection),
                end: point_from_projection(x0, y1, &complex.dropped, complex.projection),
            });
        }
        if left_empty {
            fragments.push(DirectedFragment {
                start: point_from_projection(x0, y1, &complex.dropped, complex.projection),
                end: point_from_projection(x0, y0, &complex.dropped, complex.projection),
            });
        }
    }
    stitch_loops(fragments, complex.projection)
}

#[cfg(feature = "exact-triangulation")]
fn stitch_loops(
    mut fragments: Vec<DirectedFragment>,
    projection: CoplanarProjection,
) -> Option<Vec<Vec<Point3>>> {
    let mut loops = Vec::new();
    while !fragments.is_empty() {
        let start_index = fragments
            .iter()
            .enumerate()
            .min_by(|(_, left), (_, right)| {
                compare_point2(
                    &project_point(&left.start, projection),
                    &project_point(&right.start, projection),
                )
                .unwrap_or(Ordering::Equal)
            })?
            .0;
        let first = fragments.remove(start_index);
        let start = first.start.clone();
        let mut current = first.end;
        let mut loop_points = vec![start.clone(), current.clone()];
        while !points_equal(&current, &start) {
            let matching = fragments
                .iter()
                .enumerate()
                .filter(|(_, fragment)| points_equal(&fragment.start, &current))
                .map(|(index, _)| index)
                .collect::<Vec<_>>();
            if matching.len() != 1 {
                return None;
            }
            let fragment = fragments.remove(matching[0]);
            current = fragment.end;
            if !points_equal(&current, &start)
                && loop_points
                    .iter()
                    .any(|point| points_equal(point, &current))
            {
                return None;
            }
            loop_points.push(current.clone());
        }
        loop_points.pop();
        let simplified = simplify_projected_polygon(loop_points, projection);
        if simplified.len() < 4 {
            return None;
        }
        loops.push(simplified);
    }
    Some(loops)
}

#[cfg(feature = "exact-triangulation")]
fn components_to_mesh(
    components: &[CoplanarOrthogonalSurfaceComponent],
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for component in components {
        let offset = vertices.len();
        let mesh = component_to_mesh(component, projection)?;
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(mesh.triangles().iter().map(|triangle| {
            let [a, b, c] = triangle.0;
            Triangle([a + offset, b + offset, c + offset])
        }));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact coplanar orthogonal surface arrangement"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn component_to_mesh(
    component: &CoplanarOrthogonalSurfaceComponent,
    projection: CoplanarProjection,
) -> Option<ExactMesh> {
    let mut points = component.outer.clone();
    let mut hole_indices = Vec::with_capacity(component.holes.len());
    for hole in &component.holes {
        hole_indices.push(points.len());
        points.extend(hole.iter().cloned());
    }
    let vertices2 = points
        .iter()
        .map(|point| project_for_hypertri(point, projection))
        .collect::<Vec<_>>();
    let indices = hypertri::earcut(&vertices2, &hole_indices).ok()?;
    if indices.is_empty() || indices.len() % 3 != 0 {
        return None;
    }
    let vertices = points
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = indices
        .chunks_exact(3)
        .map(|chunk| Triangle([chunk[0], chunk[1], chunk[2]]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("exact coplanar orthogonal surface component"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn validate_components(
    projection: CoplanarProjection,
    components: &[CoplanarOrthogonalSurfaceComponent],
) -> Result<(), MeshError> {
    if components.is_empty() {
        return Err(orthogonal_error(
            "orthogonal arrangement must retain at least one component",
        ));
    }
    for component in components {
        validate_loop(
            &component.outer,
            projection,
            Sign::Positive,
            "orthogonal outer loop",
        )?;
        for hole in &component.holes {
            validate_loop(hole, projection, Sign::Negative, "orthogonal hole loop")?;
            let witness = hole
                .first()
                .ok_or_else(|| orthogonal_error("orthogonal hole loop is empty"))?;
            if !point_strictly_inside_loop(witness, &component.outer, projection)? {
                return Err(orthogonal_error(
                    "orthogonal hole loop is not strictly inside its outer loop",
                ));
            }
            if loop_edges_intersect(hole, &component.outer, projection)? {
                return Err(orthogonal_error(
                    "orthogonal hole loop touches or crosses its outer loop",
                ));
            }
        }
        for left in 0..component.holes.len() {
            for right in left + 1..component.holes.len() {
                if loop_edges_intersect(
                    &component.holes[left],
                    &component.holes[right],
                    projection,
                )? {
                    return Err(orthogonal_error(
                        "orthogonal hole loops touch or cross each other",
                    ));
                }
                let witness = component.holes[right]
                    .first()
                    .ok_or_else(|| orthogonal_error("orthogonal hole loop is empty"))?;
                if point_strictly_inside_loop(witness, &component.holes[left], projection)? {
                    return Err(orthogonal_error("orthogonal hole loops are nested"));
                }
            }
        }
    }
    for left in 0..components.len() {
        for right in left + 1..components.len() {
            if loop_edges_intersect(
                &components[left].outer,
                &components[right].outer,
                projection,
            )? {
                return Err(orthogonal_error(
                    "orthogonal output components touch or cross",
                ));
            }
            let witness = components[right]
                .outer
                .first()
                .ok_or_else(|| orthogonal_error("orthogonal outer loop is empty"))?;
            if point_strictly_inside_loop(witness, &components[left].outer, projection)? {
                return Err(orthogonal_error("orthogonal output components are nested"));
            }
        }
    }
    Ok(())
}

fn validate_component_mesh(
    projection: CoplanarProjection,
    components: &[CoplanarOrthogonalSurfaceComponent],
    mesh: &ExactMesh,
) -> Result<(), MeshError> {
    if mesh.triangles().is_empty() {
        return Err(orthogonal_error(
            "orthogonal arrangement mesh has no triangulation",
        ));
    }
    let retained_points = components
        .iter()
        .flat_map(|component| {
            component
                .outer
                .iter()
                .chain(component.holes.iter().flat_map(|hole| hole.iter()))
        })
        .collect::<Vec<_>>();
    if mesh.vertices().len() != retained_points.len() {
        return Err(orthogonal_error(
            "orthogonal mesh vertex count does not match retained loops",
        ));
    }
    for (mesh_point, retained_point) in mesh.vertices().iter().zip(retained_points) {
        if !points_equal(&mesh_point.to_hyperlimit_point(), retained_point) {
            return Err(orthogonal_error(
                "orthogonal mesh vertex does not match retained loop point",
            ));
        }
    }
    let mut retained_signed_area = ExactReal::from(0);
    for component in components {
        retained_signed_area = add(
            &retained_signed_area,
            &projected_area2_signed(&component.outer, projection)
                .ok_or_else(|| orthogonal_error("orthogonal outer area was undecided"))?,
        );
        for hole in &component.holes {
            retained_signed_area = add(
                &retained_signed_area,
                &projected_area2_signed(hole, projection)
                    .ok_or_else(|| orthogonal_error("orthogonal hole area was undecided"))?,
            );
        }
    }
    let mesh_signed_area = projected_mesh_area2_signed(mesh, projection)
        .ok_or_else(|| orthogonal_error("orthogonal mesh area was undecided"))?;
    if compare_reals(&retained_signed_area, &mesh_signed_area).value() != Some(Ordering::Equal) {
        return Err(orthogonal_error(
            "orthogonal mesh signed area does not match retained loops",
        ));
    }
    Ok(())
}

fn validate_loop(
    loop_points: &[Point3],
    projection: CoplanarProjection,
    expected: Sign,
    label: &'static str,
) -> Result<(), MeshError> {
    if loop_points.len() < 4 {
        return Err(orthogonal_error(format!(
            "{label} has fewer than four vertices"
        )));
    }
    for left in 0..loop_points.len() {
        for right in left + 1..loop_points.len() {
            if points_equal(&loop_points[left], &loop_points[right]) {
                return Err(orthogonal_error(format!("{label} repeats an exact point")));
            }
        }
    }
    validate_simple_loop(loop_points, projection)?;
    let area = projected_area2_signed(loop_points, projection)
        .ok_or_else(|| orthogonal_error(format!("{label} signed area was undecided")))?;
    match compare_reals(&area, &ExactReal::from(0)).value() {
        Some(Ordering::Greater) if expected == Sign::Positive => Ok(()),
        Some(Ordering::Less) if expected == Sign::Negative => Ok(()),
        Some(_) => Err(orthogonal_error(format!("{label} has wrong orientation"))),
        None => Err(orthogonal_error(format!(
            "{label} orientation was undecided"
        ))),
    }
}

fn validate_simple_loop(
    loop_points: &[Point3],
    projection: CoplanarProjection,
) -> Result<(), MeshError> {
    for left in 0..loop_points.len() {
        let left_next = (left + 1) % loop_points.len();
        let left_start = project_point(&loop_points[left], projection);
        let left_end = project_point(&loop_points[left_next], projection);
        for right in left + 1..loop_points.len() {
            let right_next = (right + 1) % loop_points.len();
            if left_next == right || right_next == left {
                continue;
            }
            let right_start = project_point(&loop_points[right], projection);
            let right_end = project_point(&loop_points[right_next], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()
            {
                Some(SegmentIntersection::Disjoint) => {}
                Some(
                    SegmentIntersection::Proper
                    | SegmentIntersection::EndpointTouch
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => {
                    return Err(orthogonal_error(
                        "orthogonal loop has non-adjacent edge contact",
                    ));
                }
                None => {
                    return Err(orthogonal_error(
                        "orthogonal loop simplicity predicate was undecided",
                    ));
                }
            }
        }
    }
    Ok(())
}

fn loop_edges_intersect(
    left: &[Point3],
    right: &[Point3],
    projection: CoplanarProjection,
) -> Result<bool, MeshError> {
    for left_edge in 0..left.len() {
        let left_start = project_point(&left[left_edge], projection);
        let left_end = project_point(&left[(left_edge + 1) % left.len()], projection);
        for right_edge in 0..right.len() {
            let right_start = project_point(&right[right_edge], projection);
            let right_end = project_point(&right[(right_edge + 1) % right.len()], projection);
            match classify_segment_intersection(&left_start, &left_end, &right_start, &right_end)
                .value()
            {
                Some(SegmentIntersection::Disjoint) => {}
                Some(
                    SegmentIntersection::Proper
                    | SegmentIntersection::EndpointTouch
                    | SegmentIntersection::CollinearOverlap
                    | SegmentIntersection::Identical,
                ) => return Ok(true),
                None => {
                    return Err(orthogonal_error(
                        "orthogonal cross-loop intersection predicate was undecided",
                    ));
                }
            }
        }
    }
    Ok(false)
}

fn point_strictly_inside_loop(
    point: &Point3,
    loop_points: &[Point3],
    projection: CoplanarProjection,
) -> Result<bool, MeshError> {
    let query = project_point(point, projection);
    if loop_points.iter().enumerate().any(|(index, start)| {
        point_on_projected_segment(
            start,
            &loop_points[(index + 1) % loop_points.len()],
            point,
            projection,
        )
    }) {
        return Ok(false);
    }
    let mut crossings = 0_usize;
    for edge in 0..loop_points.len() {
        let a = project_point(&loop_points[edge], projection);
        let b = project_point(&loop_points[(edge + 1) % loop_points.len()], projection);
        let ay_gt = real_order(&a.y, &query.y)
            .ok_or_else(|| orthogonal_error("orthogonal point-in-loop comparison was undecided"))?
            == Ordering::Greater;
        let by_gt = real_order(&b.y, &query.y)
            .ok_or_else(|| orthogonal_error("orthogonal point-in-loop comparison was undecided"))?
            == Ordering::Greater;
        if ay_gt == by_gt {
            continue;
        }
        let dy = sub(&b.y, &a.y);
        let t = (sub(&query.y, &a.y) / &dy)
            .ok()
            .ok_or_else(|| orthogonal_error("orthogonal point-in-loop division failed"))?;
        let x_at_y = add(&a.x, &mul(&t, &sub(&b.x, &a.x)));
        if real_order(&query.x, &x_at_y)
            .ok_or_else(|| orthogonal_error("orthogonal point-in-loop crossing was undecided"))?
            == Ordering::Less
        {
            crossings += 1;
        }
    }
    Ok(crossings % 2 == 1)
}

fn project_for_hypertri(point: &Point3, projection: CoplanarProjection) -> hypertri::ExactPoint {
    match projection {
        CoplanarProjection::Xy => hypertri::ExactPoint::new(point.x.clone(), point.y.clone()),
        CoplanarProjection::Xz => hypertri::ExactPoint::new(point.x.clone(), point.z.clone()),
        CoplanarProjection::Yz => hypertri::ExactPoint::new(point.y.clone(), point.z.clone()),
    }
}

#[cfg(feature = "exact-triangulation")]
fn cell_index(x: usize, y: usize, y_cells: usize) -> usize {
    x * y_cells + y
}

fn point_from_projection(
    u: &ExactReal,
    v: &ExactReal,
    dropped: &ExactReal,
    projection: CoplanarProjection,
) -> Point3 {
    match projection {
        CoplanarProjection::Xy => Point3::new(u.clone(), v.clone(), dropped.clone()),
        CoplanarProjection::Xz => Point3::new(u.clone(), dropped.clone(), v.clone()),
        CoplanarProjection::Yz => Point3::new(dropped.clone(), u.clone(), v.clone()),
    }
}

fn dropped_coordinate(point: &Point3, projection: CoplanarProjection) -> ExactReal {
    match projection {
        CoplanarProjection::Xy => point.z.clone(),
        CoplanarProjection::Xz => point.y.clone(),
        CoplanarProjection::Yz => point.x.clone(),
    }
}

#[cfg(feature = "exact-triangulation")]
fn rectangle_corners2(rectangle: &ProjectedRectangle) -> [Point2; 4] {
    [
        Point2::new(rectangle.min.x.clone(), rectangle.min.y.clone()),
        Point2::new(rectangle.max.x.clone(), rectangle.min.y.clone()),
        Point2::new(rectangle.max.x.clone(), rectangle.max.y.clone()),
        Point2::new(rectangle.min.x.clone(), rectangle.max.y.clone()),
    ]
}

#[cfg(feature = "exact-triangulation")]
fn rectangle_area2(rectangle: &ProjectedRectangle) -> ExactReal {
    let area = mul(
        &sub(&rectangle.max.x, &rectangle.min.x),
        &sub(&rectangle.max.y, &rectangle.min.y),
    );
    add(&area, &area)
}

#[cfg(feature = "exact-triangulation")]
fn rectangles_equal(left: &ProjectedRectangle, right: &ProjectedRectangle) -> bool {
    real_equal(&left.dropped, &right.dropped)
        && point2_equal(&left.min, &right.min)
        && point2_equal(&left.max, &right.max)
}

#[cfg(feature = "exact-triangulation")]
fn point_strictly_inside_projected_rectangle(point: &Point2, rect: &ProjectedRectangle) -> bool {
    real_order(&rect.min.x, &point.x) == Some(Ordering::Less)
        && real_order(&point.x, &rect.max.x) == Some(Ordering::Less)
        && real_order(&rect.min.y, &point.y) == Some(Ordering::Less)
        && real_order(&point.y, &rect.max.y) == Some(Ordering::Less)
}

fn point_on_projected_segment(
    start: &Point3,
    end: &Point3,
    point: &Point3,
    projection: CoplanarProjection,
) -> bool {
    let start = project_point(start, projection);
    let end = project_point(end, projection);
    let point = project_point(point, projection);
    if orient2d_report(&start, &end, &point).value() != Some(Sign::Zero) {
        return false;
    }
    let Some(min_x) = exact_min_real(&start.x, &end.x) else {
        return false;
    };
    let Some(max_x) = exact_max_real(&start.x, &end.x) else {
        return false;
    };
    let Some(min_y) = exact_min_real(&start.y, &end.y) else {
        return false;
    };
    let Some(max_y) = exact_max_real(&start.y, &end.y) else {
        return false;
    };
    matches!(
        (real_order(&min_x, &point.x), real_order(&point.x, &max_x)),
        (
            Some(Ordering::Less | Ordering::Equal),
            Some(Ordering::Less | Ordering::Equal)
        )
    ) && matches!(
        (real_order(&min_y, &point.y), real_order(&point.y, &max_y)),
        (
            Some(Ordering::Less | Ordering::Equal),
            Some(Ordering::Less | Ordering::Equal)
        )
    )
}

fn projected_area2_abs(points: &[Point3], projection: CoplanarProjection) -> Option<ExactReal> {
    let signed = projected_area2_signed(points, projection)?;
    match compare_reals(&signed, &ExactReal::from(0)).value()? {
        Ordering::Less => Some(sub(&ExactReal::from(0), &signed)),
        Ordering::Equal | Ordering::Greater => Some(signed),
    }
}

fn projected_area2_signed(points: &[Point3], projection: CoplanarProjection) -> Option<ExactReal> {
    if points.len() < 3 {
        return Some(ExactReal::from(0));
    }
    let mut sum = ExactReal::from(0);
    for index in 0..points.len() {
        let current = project_point(&points[index], projection);
        let next = project_point(&points[(index + 1) % points.len()], projection);
        sum = add(
            &sum,
            &sub(&mul(&current.x, &next.y), &mul(&current.y, &next.x)),
        );
    }
    Some(sum)
}

fn projected_mesh_area2_signed(
    mesh: &ExactMesh,
    projection: CoplanarProjection,
) -> Option<ExactReal> {
    let mut area = ExactReal::from(0);
    for triangle in mesh.triangles() {
        let points = triangle
            .0
            .iter()
            .map(|&index| {
                mesh.vertices()
                    .get(index)
                    .map(ExactPoint3::to_hyperlimit_point)
            })
            .collect::<Option<Vec<_>>>()?;
        area = add(&area, &projected_area2_signed(&points, projection)?);
    }
    Some(area)
}

fn polygon_min_projected_point(polygon: &[Point3], projection: CoplanarProjection) -> Point2 {
    polygon
        .iter()
        .map(|point| project_point(point, projection))
        .min_by(|left, right| compare_point2(left, right).unwrap_or(Ordering::Equal))
        .unwrap_or_else(|| Point2::new(ExactReal::from(0), ExactReal::from(0)))
}

fn simplify_projected_polygon(
    mut polygon: Vec<Point3>,
    projection: CoplanarProjection,
) -> Vec<Point3> {
    remove_duplicate_neighbors(&mut polygon);
    loop {
        let original_len = polygon.len();
        if original_len < 3 {
            return polygon;
        }
        let mut simplified = Vec::with_capacity(original_len);
        for index in 0..original_len {
            let previous = &polygon[(index + original_len - 1) % original_len];
            let current = &polygon[index];
            let next = &polygon[(index + 1) % original_len];
            let pa = project_point(previous, projection);
            let pb = project_point(current, projection);
            let pc = project_point(next, projection);
            if orient2d_report(&pa, &pb, &pc).value() != Some(Sign::Zero) {
                simplified.push(current.clone());
            }
        }
        remove_duplicate_neighbors(&mut simplified);
        if simplified.len() == original_len {
            return simplified;
        }
        polygon = simplified;
    }
}

fn remove_duplicate_neighbors(points: &mut Vec<Point3>) {
    points.dedup_by(|right, left| points_equal(left, right));
    if points.len() > 1 && points_equal(points.first().unwrap(), points.last().unwrap()) {
        points.pop();
    }
}

#[cfg(feature = "exact-triangulation")]
fn sort_reals_and_dedup(values: &mut Vec<ExactReal>) -> Option<()> {
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

#[cfg(feature = "exact-triangulation")]
fn midpoint_real(left: &ExactReal, right: &ExactReal) -> ExactReal {
    let half = (ExactReal::from(1) / &ExactReal::from(2)).expect("2 is nonzero");
    mul(&add(left, right), &half)
}

fn compare_point2(left: &Point2, right: &Point2) -> Option<Ordering> {
    match compare_reals(&left.x, &right.x).value()? {
        Ordering::Equal => compare_reals(&left.y, &right.y).value(),
        ordering => Some(ordering),
    }
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    real_equal(&left.x, &right.x) && real_equal(&left.y, &right.y) && real_equal(&left.z, &right.z)
}

fn point2_equal(left: &Point2, right: &Point2) -> bool {
    real_equal(&left.x, &right.x) && real_equal(&left.y, &right.y)
}

fn real_order(left: &ExactReal, right: &ExactReal) -> Option<Ordering> {
    compare_reals(left, right).value()
}

fn real_equal(left: &ExactReal, right: &ExactReal) -> bool {
    real_order(left, right) == Some(Ordering::Equal)
}

fn exact_min_real(left: &ExactReal, right: &ExactReal) -> Option<ExactReal> {
    match real_order(left, right)? {
        Ordering::Less | Ordering::Equal => Some(left.clone()),
        Ordering::Greater => Some(right.clone()),
    }
}

fn exact_max_real(left: &ExactReal, right: &ExactReal) -> Option<ExactReal> {
    match real_order(left, right)? {
        Ordering::Greater | Ordering::Equal => Some(left.clone()),
        Ordering::Less => Some(right.clone()),
    }
}

fn add(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() + right
}

fn sub(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() - right
}

fn mul(left: &ExactReal, right: &ExactReal) -> ExactReal {
    left.clone() * right
}

fn orthogonal_error(message: impl Into<String>) -> MeshError {
    MeshError::one(MeshDiagnostic::new(
        Severity::Error,
        DiagnosticKind::UnsupportedExactOperation,
        message,
    ))
}
