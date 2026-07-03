//! Boolean result extraction and triangulation helpers.

use std::collections::{BTreeMap, BTreeSet};

use crate::error::{HypermeshError, HypermeshResult};
use crate::geometry::{Classification, compare_real};
use crate::mesh::{OutputVertex, PolygonSoup};
use crate::polygon::ConvexPolygon;
use crate::winding::{BooleanOp, WindingPair};
use hyperlattice::Real;

/// Polygon plus its boolean output classification.
#[derive(Clone, Debug, PartialEq)]
pub struct ClassifiedPolygon {
    /// Classified polygon.
    pub polygon: ConvexPolygon,
    /// `+1` emits as-is, `-1` emits inverted.
    pub classification: i8,
    /// Optional front/back winding evidence.
    pub winding: Option<WindingPair>,
    /// Whether this polygon came from face-local BSP splitting.
    pub is_bsp_fragment: bool,
}

impl ClassifiedPolygon {
    /// Constructs a classified polygon.
    pub fn new(polygon: ConvexPolygon, classification: i8) -> Self {
        Self {
            polygon,
            classification,
            winding: None,
            is_bsp_fragment: false,
        }
    }
}

/// Result of a boolean operation.
#[derive(Clone, Debug, PartialEq)]
pub struct BooleanResult {
    /// Output polygon soup.
    pub output: PolygonSoup,
    /// Per-output-polygon classifications.
    pub classifications: Vec<i8>,
    /// Per-output-polygon front/back winding evidence, when produced by the
    /// general subdivision classifier.
    pub winding_pairs: Vec<Option<WindingPair>>,
    operation: Option<BooleanOp>,
}

impl BooleanResult {
    /// Constructs a result from an output soup and classifications.
    pub fn new(output: PolygonSoup, classifications: Vec<i8>) -> Self {
        let winding_pairs = vec![None; classifications.len()];
        Self {
            output,
            classifications,
            winding_pairs,
            operation: None,
        }
    }

    /// Builds a result by applying classification orientation to owned
    /// classified polygons.
    pub fn from_classified(output: PolygonSoup, classified: Vec<ClassifiedPolygon>) -> Self {
        Self::from_classified_with_operation(output, classified, None)
    }

    /// Builds a result from classified polygons and records the source operation.
    pub fn from_classified_with_operation(
        mut output: PolygonSoup,
        classified: Vec<ClassifiedPolygon>,
        operation: Option<BooleanOp>,
    ) -> Self {
        output.polygons.clear();
        let mut classifications = Vec::with_capacity(classified.len());
        let mut winding_pairs = Vec::with_capacity(classified.len());

        for classified_polygon in classified {
            let classification = classified_polygon.classification;
            let winding = classified_polygon.winding;
            let polygon = if classification == -1 {
                classified_polygon.polygon.inverted()
            } else {
                classified_polygon.polygon
            };
            output.polygons.push(polygon);
            classifications.push(classification);
            winding_pairs.push(winding);
        }

        Self {
            output,
            classifications,
            winding_pairs,
            operation,
        }
    }
}

/// Extracted output polygon with explicit vertices.
#[derive(Clone, Debug, PartialEq)]
pub struct OutputPolygon {
    /// Vertices in polygon winding order.
    pub vertices: Vec<OutputVertex>,
    /// Source mesh index.
    pub source_mesh: isize,
    /// Source polygon index.
    pub source_polygon: isize,
}

/// Indexed triangle soup using hyperreal output vertices.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TriangleSoup {
    /// Output vertices.
    pub vertices: Vec<OutputVertex>,
    /// Triangle vertex indices.
    pub triangles: Vec<[usize; 3]>,
}

/// Extracts output polygons from a boolean result.
pub fn extract_output(result: &BooleanResult) -> HypermeshResult<Vec<OutputPolygon>> {
    extract_output_polygons(&result.output.polygons)
}

/// Extracts output polygons from a borrowed polygon slice.
pub fn extract_output_polygons(polygons: &[ConvexPolygon]) -> HypermeshResult<Vec<OutputPolygon>> {
    let mut out = Vec::with_capacity(polygons.len());
    for polygon in polygons {
        let mut vertices = Vec::with_capacity(polygon.vertex_count());
        for point in polygon.vertices()? {
            vertices.push(OutputVertex {
                x: point.x,
                y: point.y,
                z: point.z,
            });
        }
        out.push(OutputPolygon {
            vertices,
            source_mesh: polygon.mesh_index,
            source_polygon: polygon.polygon_index,
        });
    }
    Ok(out)
}

/// Fan-triangulates all output polygons in a boolean result.
pub fn triangulate_output(result: &BooleanResult) -> HypermeshResult<TriangleSoup> {
    triangulate_polygons(&result.output.polygons)
}

/// Fan-triangulates and applies exact duplicate/T-junction cleanup.
pub fn triangulate_and_resolve(result: &BooleanResult) -> HypermeshResult<TriangleSoup> {
    let mut soup = resolve_tjunctions(&triangulate_output(result)?)?;
    if crate::geometry::classify_real(&signed_volume_numerator(&soup))? == Classification::On {
        soup.triangles.clear();
        soup.vertices.clear();
        return Ok(soup);
    }
    fill_boundary_loops(&mut soup);
    remove_degenerate_and_duplicate_triangles(&mut soup);
    fix_winding_by_signed_volume(&mut soup)?;
    soup = peel_open_boundary_triangles(&soup);
    if result.operation == Some(BooleanOp::Intersection)
        && peel_open_boundary_triangles(&soup).triangles.is_empty()
    {
        soup.triangles.clear();
        soup.vertices.clear();
    }
    Ok(soup)
}

/// Fan-triangulates and resolves exact duplicate/T-junction artifacts, but
/// does not cap or peel boundaries.
///
/// This is useful for tests and callers that need evidence that the classified
/// arrangement is already a closed regularized surface. Non-empty open or
/// zero-volume soups are reported as uncertified instead of being repaired.
pub fn triangulate_and_resolve_certified(result: &BooleanResult) -> HypermeshResult<TriangleSoup> {
    let mut soup = resolve_tjunctions(&triangulate_output(result)?)?;
    if soup.triangles.is_empty() {
        return Ok(soup);
    }
    if !triangle_soup_is_closed(&soup) {
        return Err(HypermeshError::UnknownClassification);
    }
    if crate::geometry::classify_real(&signed_volume_numerator(&soup))? == Classification::On {
        return Err(HypermeshError::UnknownClassification);
    }
    fix_winding_by_signed_volume(&mut soup)?;
    Ok(soup)
}

/// Fan-triangulates a borrowed polygon slice.
pub fn triangulate_polygons(polygons: &[ConvexPolygon]) -> HypermeshResult<TriangleSoup> {
    let mut soup = TriangleSoup::default();

    for polygon in polygons {
        let vertex_count = polygon.vertex_count();
        if vertex_count < 3 {
            continue;
        }

        let base = soup.vertices.len();
        for point in polygon.vertices()? {
            soup.vertices.push(OutputVertex {
                x: point.x,
                y: point.y,
                z: point.z,
            });
        }

        for index in 1..(vertex_count - 1) {
            soup.triangles.push([base, base + index, base + index + 1]);
        }
    }

    Ok(soup)
}

/// Resolves exact duplicate vertices, duplicate faces, and exact T-junctions.
///
/// This pass deliberately uses no tolerance and no primitive floating-point
/// arithmetic. It only merges or splits when exact hyperreal predicates prove
/// equality, collinearity, and segment containment.
pub fn resolve_tjunctions(input: &TriangleSoup) -> HypermeshResult<TriangleSoup> {
    let mut soup = merge_duplicate_vertices(input);
    remove_degenerate_and_duplicate_triangles(&mut soup);

    for _ in 0..256 {
        let split_tjunction = split_one_tjunction_pass(&mut soup)?;
        let split_crossing = split_one_edge_crossing_pass(&mut soup)?;
        if !split_tjunction && !split_crossing {
            break;
        }
        remove_degenerate_and_duplicate_triangles(&mut soup);
    }

    fix_winding_by_signed_volume(&mut soup)?;
    Ok(soup)
}

fn merge_duplicate_vertices(input: &TriangleSoup) -> TriangleSoup {
    let mut vertices: Vec<OutputVertex> = Vec::new();
    let mut remap = vec![0; input.vertices.len()];

    for (index, vertex) in input.vertices.iter().enumerate() {
        if let Some(existing) = vertices.iter().position(|candidate| candidate == vertex) {
            remap[index] = existing;
        } else {
            remap[index] = vertices.len();
            vertices.push(vertex.clone());
        }
    }

    let triangles = input
        .triangles
        .iter()
        .map(|triangle| [remap[triangle[0]], remap[triangle[1]], remap[triangle[2]]])
        .collect();

    TriangleSoup {
        vertices,
        triangles,
    }
}

fn remove_degenerate_and_duplicate_triangles(soup: &mut TriangleSoup) {
    let mut seen = BTreeSet::new();
    soup.triangles.retain(|triangle| {
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[0] == triangle[2] {
            return false;
        }
        let mut key = *triangle;
        key.sort();
        seen.insert(key)
    });
}

fn peel_open_boundary_triangles(input: &TriangleSoup) -> TriangleSoup {
    let mut soup = input.clone();
    loop {
        let edge_counts = triangle_edge_counts(&soup.triangles);
        let old_len = soup.triangles.len();
        soup.triangles.retain(|triangle| {
            triangle_edges(*triangle)
                .iter()
                .all(|edge| edge_counts.get(&sorted_edge(*edge)).copied().unwrap_or(0) == 2)
        });
        if soup.triangles.len() == old_len {
            return soup;
        }
    }
}

fn triangle_edge_counts(triangles: &[[usize; 3]]) -> BTreeMap<[usize; 2], usize> {
    let mut counts = BTreeMap::new();
    for triangle in triangles {
        for edge in triangle_edges(*triangle) {
            *counts.entry(sorted_edge(edge)).or_insert(0) += 1;
        }
    }
    counts
}

/// Returns true when every undirected triangle edge is used by exactly two
/// triangles.
pub fn triangle_soup_is_closed(soup: &TriangleSoup) -> bool {
    triangle_edge_counts(&soup.triangles)
        .values()
        .all(|count| *count == 2)
}

fn fill_boundary_loops(soup: &mut TriangleSoup) {
    let mut edge_counts = triangle_edge_counts(&soup.triangles);
    let mut unused = edge_counts
        .iter()
        .filter_map(|(edge, count)| if *count == 1 { Some(*edge) } else { None })
        .collect::<BTreeSet<_>>();
    if unused.is_empty() {
        return;
    }

    let mut adjacency: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for edge in &unused {
        adjacency.entry(edge[0]).or_default().push(edge[1]);
        adjacency.entry(edge[1]).or_default().push(edge[0]);
    }

    let mut caps = Vec::new();
    while let Some(start_edge) = unused.iter().next().copied() {
        let start = start_edge[0];
        let mut previous = start_edge[0];
        let mut current = start_edge[1];
        let mut loop_vertices = vec![start, current];
        unused.remove(&start_edge);

        while current != start {
            let Some(neighbors) = adjacency.get(&current) else {
                break;
            };
            let Some(next) = neighbors.iter().copied().find(|candidate| {
                *candidate != previous && unused.contains(&sorted_edge([current, *candidate]))
            }) else {
                break;
            };
            unused.remove(&sorted_edge([current, next]));
            previous = current;
            current = next;
            if current != start {
                loop_vertices.push(current);
            }
        }

        if current == start
            && loop_vertices.len() >= 3
            && let Some(loop_caps) = fan_for_boundary_loop(&loop_vertices, &edge_counts)
        {
            for triangle in &loop_caps {
                for edge in triangle_edges(*triangle) {
                    *edge_counts.entry(sorted_edge(edge)).or_insert(0) += 1;
                }
            }
            caps.extend(loop_caps);
        }
    }

    soup.triangles.extend(caps);
}

fn fan_for_boundary_loop(
    loop_vertices: &[usize],
    existing_counts: &BTreeMap<[usize; 2], usize>,
) -> Option<Vec<[usize; 3]>> {
    for root in 0..loop_vertices.len() {
        let rotated = loop_vertices
            .iter()
            .cycle()
            .skip(root)
            .take(loop_vertices.len())
            .copied()
            .collect::<Vec<_>>();
        let mut caps = Vec::new();
        let mut cap_counts = BTreeMap::new();
        let mut valid = true;

        for index in 1..(rotated.len() - 1) {
            let triangle = [rotated[0], rotated[index], rotated[index + 1]];
            for edge in triangle_edges(triangle) {
                *cap_counts.entry(sorted_edge(edge)).or_insert(0usize) += 1;
            }
            caps.push(triangle);
        }

        for (edge, cap_count) in &cap_counts {
            let existing = existing_counts.get(edge).copied().unwrap_or(0);
            let final_count = existing + cap_count;
            let is_boundary = loop_vertices
                .iter()
                .zip(loop_vertices.iter().cycle().skip(1))
                .take(loop_vertices.len())
                .any(|(a, b)| sorted_edge([*a, *b]) == *edge);
            if (is_boundary && final_count != 2) || (!is_boundary && existing != 0) {
                valid = false;
                break;
            }
        }

        if valid {
            return Some(caps);
        }
    }

    None
}

fn split_one_tjunction_pass(soup: &mut TriangleSoup) -> HypermeshResult<bool> {
    let mut edge_faces: BTreeMap<[usize; 2], Vec<usize>> = BTreeMap::new();
    for (face_index, triangle) in soup.triangles.iter().enumerate() {
        for edge in triangle_edges(*triangle) {
            edge_faces
                .entry(sorted_edge(edge))
                .or_default()
                .push(face_index);
        }
    }

    let mut to_remove = BTreeSet::new();
    let mut new_triangles = Vec::new();

    for (edge, faces) in edge_faces {
        let mut on_edge = Vec::new();
        for vertex_index in 0..soup.vertices.len() {
            if vertex_index == edge[0] || vertex_index == edge[1] {
                continue;
            }
            if point_on_segment_exact(
                &soup.vertices[vertex_index],
                &soup.vertices[edge[0]],
                &soup.vertices[edge[1]],
            )? {
                on_edge.push(vertex_index);
            }
        }
        if on_edge.is_empty() {
            continue;
        }

        for face_index in faces {
            if to_remove.contains(&face_index) {
                continue;
            }

            let triangle = soup.triangles[face_index];
            for edge_index in 0..3 {
                let ea = triangle[edge_index];
                let eb = triangle[(edge_index + 1) % 3];
                let ec = triangle[(edge_index + 2) % 3];
                if sorted_edge([ea, eb]) != edge {
                    continue;
                }

                let mut chain = Vec::with_capacity(on_edge.len() + 2);
                chain.push(ea);
                chain.extend(sort_along_segment(&on_edge, ea, eb, &soup.vertices)?);
                chain.push(eb);

                for pair in chain.windows(2) {
                    if pair[0] != pair[1] && pair[0] != ec && pair[1] != ec {
                        new_triangles.push([pair[0], pair[1], ec]);
                    }
                }
                to_remove.insert(face_index);
                break;
            }
        }
    }

    if to_remove.is_empty() {
        return Ok(false);
    }

    let mut kept = Vec::with_capacity(soup.triangles.len() + new_triangles.len());
    for (index, triangle) in soup.triangles.iter().enumerate() {
        if !to_remove.contains(&index) {
            kept.push(*triangle);
        }
    }
    kept.extend(new_triangles);
    soup.triangles = kept;
    Ok(true)
}

fn split_one_edge_crossing_pass(soup: &mut TriangleSoup) -> HypermeshResult<bool> {
    let mut edges = Vec::new();
    for triangle in &soup.triangles {
        for edge in triangle_edges(*triangle) {
            edges.push(sorted_edge(edge));
        }
    }
    edges.sort();
    edges.dedup();

    for left_index in 0..edges.len() {
        for right_index in (left_index + 1)..edges.len() {
            let left = edges[left_index];
            let right = edges[right_index];
            if left.iter().any(|vertex| right.contains(vertex)) {
                continue;
            }

            let Some(point) = proper_segment_intersection(
                &soup.vertices[left[0]],
                &soup.vertices[left[1]],
                &soup.vertices[right[0]],
                &soup.vertices[right[1]],
            )?
            else {
                continue;
            };

            let new_index = insert_or_find_vertex(soup, point);
            split_edges_at_vertex(soup, &[left, right], new_index);
            return Ok(true);
        }
    }

    Ok(false)
}

fn proper_segment_intersection(
    a: &OutputVertex,
    b: &OutputVertex,
    c: &OutputVertex,
    d: &OutputVertex,
) -> HypermeshResult<Option<OutputVertex>> {
    let ab = sub_vertex(b, a);
    let cd = sub_vertex(d, c);
    let normal = cross_arrays(&ab, &cd);
    if normal
        .iter()
        .all(|component| crate::geometry::classify_real(component) == Ok(Classification::On))
    {
        return Ok(None);
    }

    let ac = sub_vertex(c, a);
    if crate::geometry::classify_real(&dot_arrays(&ac, &normal))? != Classification::On {
        return Ok(None);
    }

    let projection_axis = dominant_component_axis(&normal)?;
    let (u_axis, v_axis) = match projection_axis {
        0 => (1, 2),
        1 => (0, 2),
        2 => (0, 1),
        _ => unreachable!("axis must be in 0..3"),
    };
    let denom = (&ab[u_axis] * &cd[v_axis]) - (&ab[v_axis] * &cd[u_axis]);
    if crate::geometry::classify_real(&denom)? == Classification::On {
        return Ok(None);
    }
    let t_num = (&ac[u_axis] * &cd[v_axis]) - (&ac[v_axis] * &cd[u_axis]);
    let t = (t_num / denom).map_err(|_| crate::error::HypermeshError::UnknownClassification)?;
    let point = OutputVertex {
        x: &a.x + &(t.clone() * &ab[0]),
        y: &a.y + &(t.clone() * &ab[1]),
        z: &a.z + &(t * &ab[2]),
    };

    if point == *a || point == *b || point == *c || point == *d {
        return Ok(None);
    }
    if point_on_segment_exact(&point, a, b)? && point_on_segment_exact(&point, c, d)? {
        Ok(Some(point))
    } else {
        Ok(None)
    }
}

fn insert_or_find_vertex(soup: &mut TriangleSoup, vertex: OutputVertex) -> usize {
    if let Some(index) = soup
        .vertices
        .iter()
        .position(|existing| existing == &vertex)
    {
        index
    } else {
        let index = soup.vertices.len();
        soup.vertices.push(vertex);
        index
    }
}

fn split_edges_at_vertex(soup: &mut TriangleSoup, edges: &[[usize; 2]], vertex: usize) {
    let mut new_triangles = Vec::new();
    let mut kept = Vec::new();
    for triangle in &soup.triangles {
        let mut split = false;
        for edge_index in 0..3 {
            let ea = triangle[edge_index];
            let eb = triangle[(edge_index + 1) % 3];
            let ec = triangle[(edge_index + 2) % 3];
            if edges.contains(&sorted_edge([ea, eb]))
                && vertex != ea
                && vertex != eb
                && vertex != ec
            {
                new_triangles.push([ea, vertex, ec]);
                new_triangles.push([vertex, eb, ec]);
                split = true;
                break;
            }
        }
        if !split {
            kept.push(*triangle);
        }
    }
    kept.extend(new_triangles);
    soup.triangles = kept;
}

fn triangle_edges(triangle: [usize; 3]) -> [[usize; 2]; 3] {
    [
        [triangle[0], triangle[1]],
        [triangle[1], triangle[2]],
        [triangle[2], triangle[0]],
    ]
}

fn sorted_edge(edge: [usize; 2]) -> [usize; 2] {
    if edge[0] <= edge[1] {
        edge
    } else {
        [edge[1], edge[0]]
    }
}

fn point_on_segment_exact(
    point: &OutputVertex,
    start: &OutputVertex,
    end: &OutputVertex,
) -> HypermeshResult<bool> {
    let ab = sub_vertex(end, start);
    let av = sub_vertex(point, start);
    let cross = cross_arrays(&ab, &av);
    if cross
        .iter()
        .any(|component| crate::geometry::classify_real(component) != Ok(Classification::On))
    {
        return Ok(false);
    }

    for axis in 0..3 {
        let p = vertex_axis(point, axis);
        let a = vertex_axis(start, axis);
        let b = vertex_axis(end, axis);
        if compare_real(p, a)?.is_lt() && compare_real(p, b)?.is_lt() {
            return Ok(false);
        }
        if compare_real(p, a)?.is_gt() && compare_real(p, b)?.is_gt() {
            return Ok(false);
        }
    }

    Ok(point != start && point != end)
}

fn sort_along_segment(
    indices: &[usize],
    start: usize,
    end: usize,
    vertices: &[OutputVertex],
) -> HypermeshResult<Vec<usize>> {
    let axis = dominant_segment_axis(&vertices[start], &vertices[end])?;
    let ascending = compare_real(
        vertex_axis(&vertices[start], axis),
        vertex_axis(&vertices[end], axis),
    )?
    .is_lt();
    let mut sorted = Vec::new();

    for index in indices {
        let mut insert_at = sorted.len();
        for (position, existing) in sorted.iter().enumerate() {
            let order = compare_real(
                vertex_axis(&vertices[*index], axis),
                vertex_axis(&vertices[*existing], axis),
            )?;
            if (ascending && order.is_lt()) || (!ascending && order.is_gt()) {
                insert_at = position;
                break;
            }
        }
        sorted.insert(insert_at, *index);
    }

    Ok(sorted)
}

fn dominant_segment_axis(start: &OutputVertex, end: &OutputVertex) -> HypermeshResult<usize> {
    let delta = sub_vertex(end, start);
    let abs = [
        delta[0].clone().abs(),
        delta[1].clone().abs(),
        delta[2].clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn dominant_component_axis(values: &[Real; 3]) -> HypermeshResult<usize> {
    let abs = [
        values[0].clone().abs(),
        values[1].clone().abs(),
        values[2].clone().abs(),
    ];
    let mut best = 0;
    for axis in 1..3 {
        if compare_real(&abs[axis], &abs[best])?.is_gt() {
            best = axis;
        }
    }
    Ok(best)
}

fn fix_winding_by_signed_volume(soup: &mut TriangleSoup) -> HypermeshResult<()> {
    let volume = signed_volume_numerator(soup);
    if crate::geometry::classify_real(&volume)? == Classification::Negative {
        for triangle in &mut soup.triangles {
            triangle.swap(0, 1);
        }
    }
    Ok(())
}

fn signed_volume_numerator(soup: &TriangleSoup) -> Real {
    let mut volume = Real::zero();
    for triangle in &soup.triangles {
        let v0 = &soup.vertices[triangle[0]];
        let v1 = &soup.vertices[triangle[1]];
        let v2 = &soup.vertices[triangle[2]];
        let term = &v0.x * &((&v1.y * &v2.z) - (&v1.z * &v2.y))
            + &v0.y * &((&v1.z * &v2.x) - (&v1.x * &v2.z))
            + &v0.z * &((&v1.x * &v2.y) - (&v1.y * &v2.x));
        volume += term;
    }
    volume
}

fn sub_vertex(left: &OutputVertex, right: &OutputVertex) -> [Real; 3] {
    [&left.x - &right.x, &left.y - &right.y, &left.z - &right.z]
}

fn cross_arrays(left: &[Real; 3], right: &[Real; 3]) -> [Real; 3] {
    [
        (&left[1] * &right[2]) - (&left[2] * &right[1]),
        (&left[2] * &right[0]) - (&left[0] * &right[2]),
        (&left[0] * &right[1]) - (&left[1] * &right[0]),
    ]
}

fn dot_arrays(left: &[Real; 3], right: &[Real; 3]) -> Real {
    (&left[0] * &right[0]) + (&left[1] * &right[1]) + (&left[2] * &right[2])
}

fn vertex_axis(vertex: &OutputVertex, axis: usize) -> &Real {
    match axis {
        0 => &vertex.x,
        1 => &vertex.y,
        2 => &vertex.z,
        _ => panic!("axis must be 0, 1, or 2"),
    }
}
