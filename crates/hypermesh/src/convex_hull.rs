//! Exact three-dimensional convex hull construction.
//!
//! Hull topology is decided with certified point-plane predicates. A static
//! point BVH accelerates outside-set discovery by accepting or rejecting whole
//! exact AABBs against each newly created hull face.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::Classification;
use crate::{ExactPointBvh, InputMesh, Point3, Real, Triangle};

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct PositionBucket([Option<u64>; 3]);

impl PositionBucket {
    fn new(point: &Point3) -> Self {
        Self([&point.x, &point.y, &point.z].map(|coordinate| {
            coordinate.to_f64_lossy().map(|value| {
                if value == 0.0 {
                    0.0_f64.to_bits()
                } else {
                    value.to_bits()
                }
            })
        }))
    }
}

#[derive(Clone, Debug)]
struct HullFace {
    vertices: [usize; 3],
    outside: Vec<usize>,
    active: bool,
}

/// Computes the exact three-dimensional convex hull of `input`.
///
/// Duplicate points are certified coordinate-by-coordinate and removed. The
/// returned mesh contains only hull vertices and outward-wound triangles.
/// Inputs with fewer than four unique non-coplanar points return
/// [`HypermeshError::DegeneratePointSet`].
pub fn convex_hull(input: &[Point3]) -> HypermeshResult<InputMesh> {
    convex_hull_impl(input, &[], None)
}

/// Computes an exact convex hull while retaining certified source coplanarity.
///
/// Each group contains input point indices known to lie on one common plane.
/// These facts are consulted only when the general `orient3d` predicate is
/// undecidable, preserving transformed polygon structure without sampling.
pub fn convex_hull_with_coplanar_groups(
    input: &[Point3],
    coplanar_groups: &[Vec<usize>],
) -> HypermeshResult<InputMesh> {
    convex_hull_impl(input, coplanar_groups, None)
}

/// Computes an exact convex hull with retained construction identities.
///
/// Each identity is `[x, y, z, ruled_surface, generator]`. Equal coordinate
/// identities certify coordinate equality. Equal surface and generator pairs
/// certify points on one generator of the same conical or cylindrical surface.
pub fn convex_hull_with_retained_facts(
    input: &[Point3],
    coplanar_groups: &[Vec<usize>],
    coordinate_ids: &[[u64; 5]],
) -> HypermeshResult<InputMesh> {
    if input.len() != coordinate_ids.len() {
        return Err(HypermeshError::PointCountMismatch {
            expected: input.len(),
            actual: coordinate_ids.len(),
        });
    }
    convex_hull_impl(input, coplanar_groups, Some(coordinate_ids))
}

fn convex_hull_impl(
    input: &[Point3],
    coplanar_groups: &[Vec<usize>],
    input_coordinate_ids: Option<&[[u64; 5]]>,
) -> HypermeshResult<InputMesh> {
    let (points, memberships, coordinate_ids) =
        deduplicate_points(input, coplanar_groups, input_coordinate_ids)?;
    let memberships = memberships.as_deref();
    let coordinate_ids = coordinate_ids.as_deref();
    let seed = hull_stage(
        seed_tetrahedron(&points, memberships, coordinate_ids),
        "seed selection",
    )?;
    let interior = hull_stage(tetrahedron_centroid(&points, seed), "seed centroid")?;
    let approximate_points = points
        .iter()
        .map(|point| {
            [
                point.x.to_f64_lossy().unwrap_or(0.0),
                point.y.to_f64_lossy().unwrap_or(0.0),
                point.z.to_f64_lossy().unwrap_or(0.0),
            ]
        })
        .collect::<Vec<_>>();
    let point_bvh = match ExactPointBvh::build_with_approximate(&points, &approximate_points) {
        Ok(point_bvh) => Some(point_bvh),
        Err(HypermeshError::UnknownClassification) => None,
        Err(error) => return Err(error),
    };
    let mut processed = vec![false; points.len()];
    for index in seed {
        processed[index] = true;
    }
    let mut faces = Vec::with_capacity(8);
    for vertices in [
        [seed[0], seed[1], seed[2]],
        [seed[0], seed[3], seed[1]],
        [seed[0], seed[2], seed[3]],
        [seed[1], seed[3], seed[2]],
    ] {
        faces.push(hull_stage(
            make_face(
                vertices,
                &points,
                memberships,
                coordinate_ids,
                point_bvh.as_ref(),
                &interior,
                &processed,
            ),
            "initial face construction",
        )?);
    }

    while let Some((source_face, eye)) = faces.iter_mut().enumerate().find_map(|(index, face)| {
        if !face.active {
            return None;
        }
        pop_farthest_outside(face, &approximate_points, &processed).map(|point| (index, point))
    }) {
        processed[eye] = true;
        let mut visible = Vec::new();
        for (index, face) in faces.iter().enumerate() {
            if face.active
                && hull_stage(
                    orientation_index(&points, memberships, coordinate_ids, face.vertices, eye),
                    "visible face classification",
                )? == Classification::Negative
            {
                visible.push(index);
            }
        }
        if visible.is_empty() {
            faces[source_face].outside.retain(|&point| point != eye);
            continue;
        }

        let mut horizon = BTreeSet::new();
        for &face_index in &visible {
            let [a, b, c] = faces[face_index].vertices;
            for edge in [(a, b), (b, c), (c, a)] {
                if !horizon.remove(&(edge.1, edge.0)) {
                    horizon.insert(edge);
                }
            }
            faces[face_index].active = false;
            faces[face_index].outside.clear();
        }

        for (a, b) in horizon {
            faces.push(hull_stage(
                make_face(
                    [a, b, eye],
                    &points,
                    memberships,
                    coordinate_ids,
                    point_bvh.as_ref(),
                    &interior,
                    &processed,
                ),
                "horizon face construction",
            )?);
        }
    }

    compact_hull(points, faces)
}

fn pop_farthest_outside(
    face: &mut HullFace,
    points: &[[f64; 3]],
    processed: &[bool],
) -> Option<usize> {
    face.outside.retain(|&point| !processed[point]);
    let farthest = face
        .outside
        .iter()
        .enumerate()
        .max_by(|left, right| {
            let left = *left.1;
            let right = *right.1;
            approximate_face_distance(points, face.vertices, left)
                .total_cmp(&approximate_face_distance(points, face.vertices, right))
        })?
        .0;
    Some(face.outside.swap_remove(farthest))
}

fn approximate_face_distance(points: &[[f64; 3]], face: [usize; 3], point: usize) -> f64 {
    let [a, b, c] = face.map(|index| points[index]);
    let p = points[point];
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let ap = [p[0] - a[0], p[1] - a[1], p[2] - a[2]];
    let determinant = (ab[1] * ac[2] - ab[2] * ac[1]) * ap[0]
        + (ab[2] * ac[0] - ab[0] * ac[2]) * ap[1]
        + (ab[0] * ac[1] - ab[1] * ac[0]) * ap[2];
    if determinant.is_finite() {
        determinant.abs()
    } else {
        0.0
    }
}

fn hull_stage<T>(result: HypermeshResult<T>, stage: &'static str) -> HypermeshResult<T> {
    result.map_err(|error| match error {
        HypermeshError::UnknownClassification => HypermeshError::ConvexHullPredicate { stage },
        other => other,
    })
}

fn deduplicate_points(
    input: &[Point3],
    coplanar_groups: &[Vec<usize>],
    input_coordinate_ids: Option<&[[u64; 5]]>,
) -> HypermeshResult<(
    Vec<Point3>,
    Option<Vec<BTreeSet<usize>>>,
    Option<Vec<[u64; 5]>>,
)> {
    if input.is_empty() {
        return Err(HypermeshError::EmptyInput);
    }
    let mut points = Vec::with_capacity(input.len());
    let mut memberships =
        (!coplanar_groups.is_empty()).then(|| Vec::<BTreeSet<usize>>::with_capacity(input.len()));
    let mut coordinate_ids =
        input_coordinate_ids.map(|_| Vec::<[u64; 5]>::with_capacity(input.len()));
    let mut input_memberships = if coplanar_groups.is_empty() {
        None
    } else {
        Some(vec![Vec::new(); input.len()])
    };
    if let Some(input_memberships) = &mut input_memberships {
        for (group_index, group) in coplanar_groups.iter().enumerate() {
            for &point_index in group {
                let Some(point_memberships) = input_memberships.get_mut(point_index) else {
                    return Err(HypermeshError::VertexIndexOutOfBounds {
                        index: point_index,
                        vertex_count: input.len(),
                    });
                };
                point_memberships.push(group_index);
            }
        }
    }
    let mut buckets = HashMap::<PositionBucket, usize>::with_capacity(input.len());
    let mut next_in_bucket = Vec::<Option<usize>>::with_capacity(input.len());
    for (input_index, point) in input.iter().enumerate() {
        let bucket = PositionBucket::new(point);
        let mut duplicate = None;
        let mut candidate = buckets.get(&bucket).copied();
        while let Some(candidate_index) = candidate {
            if points_equal(&points[candidate_index], point) {
                duplicate = Some(candidate_index);
                break;
            }
            candidate = next_in_bucket[candidate_index];
        }
        if let Some(candidate) = duplicate {
            if let (Some(memberships), Some(input_memberships)) =
                (&mut memberships, &input_memberships)
            {
                memberships[candidate].extend(input_memberships[input_index].iter().copied());
            }
        } else {
            let point_index = points.len();
            next_in_bucket.push(buckets.insert(bucket, point_index));
            points.push(point.clone());
            if let (Some(coordinate_ids), Some(input_coordinate_ids)) =
                (&mut coordinate_ids, input_coordinate_ids)
            {
                coordinate_ids.push(input_coordinate_ids[input_index]);
            }
            if let (Some(memberships), Some(input_memberships)) =
                (&mut memberships, &input_memberships)
            {
                memberships.push(input_memberships[input_index].iter().copied().collect());
            }
        }
    }
    Ok((points, memberships, coordinate_ids))
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    if left == right {
        return true;
    }
    hyperlimit::point3_equal(left, right)
        .value()
        .unwrap_or(false)
}

fn seed_tetrahedron(
    points: &[Point3],
    memberships: Option<&[BTreeSet<usize>]>,
    coordinate_ids: Option<&[[u64; 5]]>,
) -> HypermeshResult<[usize; 4]> {
    if points.len() < 4 {
        return Err(HypermeshError::DegeneratePointSet);
    }
    let p0 = 0;
    let p1 = 1;
    let mut p2 = None;
    for (candidate, point) in points.iter().enumerate().skip(2) {
        let cross = (point - &points[p0]).cross(&(&points[p1] - &points[p0]));
        if cross.structural_facts().squared_norm_zero_status() == hyperlattice::ZeroStatus::NonZero
        {
            p2 = Some(candidate);
            break;
        }
    }
    let p2 = p2.ok_or(HypermeshError::DegeneratePointSet)?;
    let mut p3 = None;
    for (candidate, _) in points.iter().enumerate().skip(2) {
        if candidate != p2
            && orientation_index(points, memberships, coordinate_ids, [p0, p1, p2], candidate)?
                != Classification::On
        {
            p3 = Some(candidate);
            break;
        }
    }
    let p3 = p3.ok_or(HypermeshError::DegeneratePointSet)?;
    Ok([p0, p1, p2, p3])
}

fn tetrahedron_centroid(points: &[Point3], seed: [usize; 4]) -> HypermeshResult<Point3> {
    let four = Real::from(4_u8);
    let average = |coordinate: fn(&Point3) -> &Real| {
        seed.iter()
            .map(|&index| coordinate(&points[index]))
            .fold(Real::zero(), |sum, value| sum + value)
            / &four
    };
    Ok(Point3::new(
        average(|point| &point.x).map_err(|_| HypermeshError::UnknownClassification)?,
        average(|point| &point.y).map_err(|_| HypermeshError::UnknownClassification)?,
        average(|point| &point.z).map_err(|_| HypermeshError::UnknownClassification)?,
    ))
}

fn make_face(
    mut vertices: [usize; 3],
    points: &[Point3],
    memberships: Option<&[BTreeSet<usize>]>,
    coordinate_ids: Option<&[[u64; 5]]>,
    point_bvh: Option<&ExactPointBvh>,
    interior: &Point3,
    processed: &[bool],
) -> HypermeshResult<HullFace> {
    if hull_stage(
        orientation(points, vertices, interior),
        "face interior orientation",
    )? == Classification::Negative
    {
        vertices.swap(1, 2);
    }
    let mut outside = Vec::new();
    if let Some(point_bvh) = point_bvh {
        let query = point_bvh.query_negative_oriented_plane(
            points,
            &points[vertices[0]],
            &points[vertices[1]],
            &points[vertices[2]],
            |point| {
                if !processed[point] {
                    outside.push(point);
                }
            },
        );
        match query {
            Ok(()) => {}
            Err(HypermeshError::UnknownClassification) => {
                outside.clear();
                linear_outside_query(
                    points,
                    memberships,
                    coordinate_ids,
                    vertices,
                    &mut outside,
                    processed,
                )?;
            }
            Err(error) => return Err(error),
        }
    } else {
        linear_outside_query(
            points,
            memberships,
            coordinate_ids,
            vertices,
            &mut outside,
            processed,
        )?;
    }
    Ok(HullFace {
        vertices,
        outside,
        active: true,
    })
}

fn linear_outside_query(
    points: &[Point3],
    memberships: Option<&[BTreeSet<usize>]>,
    coordinate_ids: Option<&[[u64; 5]]>,
    vertices: [usize; 3],
    outside: &mut Vec<usize>,
    processed: &[bool],
) -> HypermeshResult<()> {
    for (point_index, &is_processed) in processed.iter().enumerate() {
        if is_processed {
            continue;
        }
        if hull_stage(
            orientation_index(points, memberships, coordinate_ids, vertices, point_index),
            "face outside linear query",
        )? == Classification::Negative
        {
            outside.push(point_index);
        }
    }
    Ok(())
}

fn orientation(
    points: &[Point3],
    face: [usize; 3],
    point: &Point3,
) -> HypermeshResult<Classification> {
    match hyperlimit::orient3d(&points[face[0]], &points[face[1]], &points[face[2]], point).value()
    {
        Some(hyperlimit::Sign::Negative) => Ok(Classification::Negative),
        Some(hyperlimit::Sign::Zero) => Ok(Classification::On),
        Some(hyperlimit::Sign::Positive) => Ok(Classification::Positive),
        None => Err(HypermeshError::UnknownClassification),
    }
}

fn orientation_index(
    points: &[Point3],
    memberships: Option<&[BTreeSet<usize>]>,
    coordinate_ids: Option<&[[u64; 5]]>,
    face: [usize; 3],
    point_index: usize,
) -> HypermeshResult<Classification> {
    if face.contains(&point_index) {
        return Ok(Classification::On);
    }
    if let Some(memberships) = memberships
        && share_coplanar_group(memberships, face, point_index)
    {
        return Ok(Classification::On);
    }
    if let Some(coordinate_ids) = coordinate_ids
        && (opposite_edges_share_axis_coordinates(coordinate_ids, face, point_index)
            || opposite_edges_share_ruled_surface(coordinate_ids, face, point_index))
    {
        return Ok(Classification::On);
    }
    orientation(points, face, &points[point_index])
}

fn opposite_edges_share_axis_coordinates(
    coordinate_ids: &[[u64; 5]],
    face: [usize; 3],
    point_index: usize,
) -> bool {
    let [a, b, c] = face.map(|index| coordinate_ids[index]);
    let d = coordinate_ids[point_index];
    let equal_except_axis = |left: [u64; 5], right: [u64; 5], axis| {
        (0..3).all(|coordinate| coordinate == axis || left[coordinate] == right[coordinate])
    };
    (0..3).any(|axis| {
        (equal_except_axis(a, b, axis) && equal_except_axis(c, d, axis))
            || (equal_except_axis(a, c, axis) && equal_except_axis(b, d, axis))
            || (equal_except_axis(a, d, axis) && equal_except_axis(b, c, axis))
    })
}

fn opposite_edges_share_ruled_surface(
    retained_ids: &[[u64; 5]],
    face: [usize; 3],
    point_index: usize,
) -> bool {
    let [a, b, c] = face.map(|index| retained_ids[index]);
    let d = retained_ids[point_index];
    let same_generator =
        |left: [u64; 5], right: [u64; 5]| left[3] == right[3] && left[4] == right[4];
    let same_surface = |left: [u64; 5], right: [u64; 5]| left[3] == right[3];
    (same_generator(a, b) && same_generator(c, d) && same_surface(a, c))
        || (same_generator(a, c) && same_generator(b, d) && same_surface(a, b))
        || (same_generator(a, d) && same_generator(b, c) && same_surface(a, b))
}

fn share_coplanar_group(
    memberships: &[BTreeSet<usize>],
    face: [usize; 3],
    point_index: usize,
) -> bool {
    memberships[face[0]].iter().any(|group| {
        memberships[face[1]].contains(group)
            && memberships[face[2]].contains(group)
            && memberships[point_index].contains(group)
    })
}

fn compact_hull(points: Vec<Point3>, faces: Vec<HullFace>) -> HypermeshResult<InputMesh> {
    let active_faces = faces
        .into_iter()
        .filter(|face| face.active)
        .collect::<Vec<_>>();
    if active_faces.len() < 4 {
        return Err(HypermeshError::DegeneratePointSet);
    }
    let mut remap = BTreeMap::new();
    for face in &active_faces {
        for &vertex in &face.vertices {
            let next = remap.len();
            remap.entry(vertex).or_insert(next);
        }
    }
    let mut positions = vec![Point3::new(Real::zero(), Real::zero(), Real::zero()); remap.len()];
    for (&old, &new) in &remap {
        positions[new] = points[old].clone();
    }
    let triangles = active_faces
        .into_iter()
        .map(|face| {
            Triangle::new(
                remap[&face.vertices[0]],
                remap[&face.vertices[1]],
                remap[&face.vertices[2]],
            )
        })
        .collect();
    Ok(InputMesh::new(positions, triangles))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Plane, classify_point};

    fn p(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn tetrahedron_hull_is_closed_and_outward() {
        let input = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)];
        let hull = convex_hull(&input).unwrap();
        assert_eq!(hull.positions.len(), 4);
        assert_eq!(hull.triangles.len(), 4);
        crate::prepare_input(&[hull.as_ref()]).unwrap();
    }

    #[test]
    fn orient3d_sign_is_opposite_hull_plane_expression() {
        let points = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0), p(0, 0, 2)];
        let plane = Plane::from_points(&points[0], &points[1], &points[2]);
        assert_eq!(
            orientation(&points, [0, 1, 2], &points[3]).unwrap(),
            Classification::Negative
        );
        assert_eq!(
            classify_point(&points[3], &plane).unwrap(),
            Classification::Positive
        );
    }

    #[test]
    fn cube_hull_rejects_interior_and_duplicate_points() {
        let mut input = Vec::new();
        for x in [-1, 1] {
            for y in [-1, 1] {
                for z in [-1, 1] {
                    input.push(p(x, y, z));
                }
            }
        }
        input.extend([p(0, 0, 0), p(1, 1, 1)]);
        let hull = convex_hull(&input).unwrap();
        assert_eq!(hull.positions.len(), 8);
        assert_eq!(hull.triangles.len(), 12);
        crate::prepare_input(&[hull.as_ref()]).unwrap();
    }

    #[test]
    fn dense_grid_rejects_all_interior_points() {
        let input = (-4..=4)
            .flat_map(|x| (-4..=4).flat_map(move |y| (-4..=4).map(move |z| p(x, y, z))))
            .collect::<Vec<_>>();

        let hull = convex_hull(&input).unwrap();
        let boundary = Real::from(4);
        assert!(hull.positions.iter().all(|point| {
            point.x == boundary
                || point.x == -boundary.clone()
                || point.y == boundary
                || point.y == -boundary.clone()
                || point.z == boundary
                || point.z == -boundary.clone()
        }));
        crate::prepare_input(&[hull.as_ref()]).unwrap();
    }

    #[test]
    fn moment_curve_retains_every_extreme_point() {
        let input = (-16_i64..16)
            .map(|t| p(t, t * t, t * t * t))
            .collect::<Vec<_>>();

        let hull = convex_hull(&input).unwrap();
        assert_eq!(hull.positions.len(), input.len());
        assert_eq!(hull.triangles.len(), input.len() * 2 - 4);
        crate::prepare_input(&[hull.as_ref()]).unwrap();
    }

    #[test]
    fn hull_retains_exact_offsets_beyond_f64_resolution() {
        let base = Real::from(1_i64 << 60);
        let input = vec![
            Point3::new(base.clone(), Real::zero(), Real::zero()),
            Point3::new(base.clone() + Real::one(), Real::zero(), Real::zero()),
            Point3::new(base.clone(), Real::one(), Real::zero()),
            Point3::new(base.clone(), Real::zero(), Real::one()),
        ];
        let hull = convex_hull(&input).unwrap();
        assert!(
            hull.positions
                .iter()
                .any(|point| point.x == base.clone() + Real::one())
        );
    }

    #[test]
    fn coplanar_input_is_rejected() {
        let input = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(1, 1, 0)];
        assert_eq!(convex_hull(&input), Err(HypermeshError::DegeneratePointSet));
    }

    #[test]
    fn irregular_cloud_is_contained_by_every_hull_halfspace() {
        let mut input = vec![
            p(-20, 0, 0),
            p(20, 0, 0),
            p(0, -20, 0),
            p(0, 20, 0),
            p(0, 0, -20),
            p(0, 0, 20),
        ];
        input.extend((0..96).map(|index| {
            p(
                (index * 17 % 31) - 15,
                (index * 23 % 29) - 14,
                (index * 11 % 27) - 13,
            )
        }));

        let hull = convex_hull(&input).unwrap();
        crate::prepare_input(&[hull.as_ref()]).unwrap();
        for triangle in &hull.triangles {
            let [a, b, c] = triangle.indices();
            let plane =
                Plane::from_points(&hull.positions[a], &hull.positions[b], &hull.positions[c]);
            assert!(input.iter().all(|point| {
                classify_point(point, &plane).unwrap() != Classification::Positive
            }));
        }
    }

    #[test]
    fn retained_ruled_surface_generators_certify_coplanarity() {
        let retained = [
            [0, 1, 2, 20, 30],
            [3, 4, 5, 20, 30],
            [6, 7, 8, 20, 31],
            [9, 10, 11, 20, 31],
        ];
        assert!(opposite_edges_share_ruled_surface(&retained, [0, 1, 2], 3));
        let points = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(0, 0, 1)];
        assert_eq!(
            orientation_index(&points, None, Some(&retained), [0, 1, 2], 3).unwrap(),
            Classification::On
        );
    }
}
