use super::*;

const STRICT: MeshContext = MeshContext::new(hyperlimit::PredicatePolicy::STRICT);

fn cube() -> TriangleMesh {
    let positions = [
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [1, 1, 1],
        [0, 1, 1],
    ]
    .map(|p| Point3::new(p[0].into(), p[1].into(), p[2].into()))
    .to_vec();
    let triangles = [
        [0, 3, 2],
        [0, 2, 1],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [3, 7, 6],
        [3, 6, 2],
        [0, 4, 7],
        [0, 7, 3],
        [1, 2, 6],
        [1, 6, 5],
    ]
    .map(|p| Triangle::new(p[0], p[1], p[2]))
    .to_vec();
    TriangleMesh::new(positions, triangles)
}

fn fresh(mesh: &TriangleMesh) -> TriangleMesh {
    TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec())
}

fn translate_positions(mesh: &mut TriangleMesh, copy_on_write: bool) {
    if copy_on_write {
        for point in Arc::make_mut(&mut mesh.positions) {
            point.x = &point.x + Real::from(10);
        }
    } else {
        mesh.positions = mesh
            .positions
            .iter()
            .cloned()
            .map(|mut point| {
                point.x = &point.x + Real::from(10);
                point
            })
            .collect::<Vec<_>>()
            .into();
    }
}

fn assert_bounds(mesh: &TriangleMesh, minimum: i32, maximum: i32) {
    let result = mesh.exact_bounds(&STRICT).unwrap();
    assert_eq!(result.certainty, MeshCertainty::Certified);
    let bounds = result.value.unwrap();
    assert_eq!(bounds.mins, Point3::new(minimum.into(), 0.into(), 0.into()));
    assert_eq!(bounds.maxs, Point3::new(maximum.into(), 1.into(), 1.into()));
}

fn warm_validity(mesh: &TriangleMesh) {
    let result = mesh.axis_aligned_box_bounds(&STRICT).unwrap();
    assert_eq!(result.certainty, MeshCertainty::Certified);
    assert!(result.value.is_some());
}

fn assert_open(mesh: &TriangleMesh) {
    assert!(matches!(
        polygon_soup(&STRICT, &[mesh.as_ref()]),
        Err(HypermeshError::OpenInput { .. })
    ));
}

#[test]
fn bounds_follow_replaced_and_copy_on_write_positions() {
    for copy_on_write in [false, true] {
        let mut mesh = cube();
        assert_bounds(&mesh, 0, 1);
        translate_positions(&mut mesh, copy_on_write);
        assert_bounds(&mesh, 10, 11);
        assert_eq!(
            mesh.exact_bounds(&STRICT),
            fresh(&mesh).exact_bounds(&STRICT)
        );
    }
}

#[test]
fn edited_clones_neither_read_nor_publish_original_bounds() {
    for warm_first in [false, true] {
        let original = cube();
        if warm_first {
            assert_bounds(&original, 0, 1);
        }
        let mut edited = original.clone();
        translate_positions(&mut edited, false);
        assert_bounds(&edited, 10, 11);
        assert_bounds(&original, 0, 1);
        assert_bounds(&original.clone(), 0, 1);
    }
}

#[test]
fn cached_empty_bounds_do_not_hide_replacement_geometry() {
    let empty = TriangleMesh::new(Vec::new(), Vec::new());
    assert_eq!(empty.exact_bounds(&STRICT).unwrap().value, None);
    let mut edited = empty.clone();
    let source = cube();
    edited.positions = source.positions;
    edited.triangles = source.triangles;
    assert_bounds(&edited, 0, 1);
    assert_eq!(empty.exact_bounds(&STRICT).unwrap().value, None);
}

#[test]
fn edited_topology_is_revalidated_through_derived_meshes() {
    let mut mesh = cube();
    warm_validity(&mesh);
    mesh.triangles = mesh.triangles[1..].to_vec().into();
    assert_open(&fresh(&mesh));
    assert_open(&mesh);
    assert_open(&mesh.reversed_winding());
    assert_open(
        &mesh
            .subdivide_triangles(NonZeroU32::new(1).unwrap())
            .unwrap(),
    );
    let matrix =
        Matrix4::from_row_major([1, 0, 0, 10, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1].map(Real::from));
    assert_open(&mesh.try_transformed(&STRICT, &matrix).unwrap().value);
    assert_open(
        &mesh
            .try_rotated_xyz_degrees(&STRICT, 0.into(), 0.into(), 90.into())
            .unwrap()
            .value,
    );
}

#[test]
fn copy_on_write_triangle_edits_recheck_winding_balance() {
    let mut mesh = cube();
    warm_validity(&mesh);
    let face = &mut Arc::make_mut(&mut mesh.triangles)[0];
    std::mem::swap(&mut face.v1, &mut face.v2);
    for value in [&mesh, &fresh(&mesh)] {
        assert!(matches!(
            polygon_soup(&STRICT, &[value.as_ref()]),
            Err(HypermeshError::NonPwnInput { .. })
        ));
    }
}

#[test]
fn source_planes_follow_replaced_positions_and_reindexed_triangles() {
    for reindex in [false, true] {
        let mut mesh = cube();
        polygon_soup(&STRICT, &[mesh.as_ref()]).unwrap();
        if reindex {
            Arc::make_mut(&mut mesh.triangles).reverse();
        } else {
            translate_positions(&mut mesh, false);
        }
        let actual = polygon_soup(&STRICT, &[mesh.as_ref()]).unwrap();
        let expected = polygon_soup(&STRICT, &[fresh(&mesh).as_ref()]).unwrap();
        assert_eq!(actual.certainty, MeshCertainty::Certified);
        assert_eq!(actual, expected);
    }
}

#[test]
fn borrowed_view_slices_neither_reuse_nor_publish_original_geometry_facts() {
    for warm_first in [false, true] {
        for reindex in [false, true] {
            let original = cube();
            if warm_first {
                polygon_soup(&STRICT, &[original.as_ref()]).unwrap();
            }
            let mut edited = fresh(&original);
            let mut view = original.as_ref();
            if reindex {
                Arc::make_mut(&mut edited.triangles).reverse();
                view.triangles = &edited.triangles;
            } else {
                translate_positions(&mut edited, false);
                view.positions = &edited.positions;
            }
            let expected = polygon_soup(&STRICT, &[edited.as_ref()]).unwrap();
            assert_eq!(polygon_soup(&STRICT, &[view]).unwrap(), expected);
            assert_eq!(
                polygon_soup(&STRICT, &[original.as_ref()]).unwrap(),
                polygon_soup(&STRICT, &[fresh(&original).as_ref()]).unwrap()
            );
        }
    }
}

#[test]
fn borrowed_view_prefix_length_is_part_of_cache_identity() {
    let mesh = cube();
    warm_validity(&mesh);
    let mut view = mesh.as_ref();
    view.triangles = &mesh.triangles[..mesh.triangles.len() - 1];
    assert!(matches!(
        polygon_soup(&STRICT, &[view]),
        Err(HypermeshError::OpenInput { .. })
    ));
}

#[test]
fn borrowed_view_can_use_a_larger_position_buffer_than_its_origin() {
    let mesh = cube();
    warm_validity(&mesh);
    let larger = mesh
        .subdivide_triangles(NonZeroU32::new(1).unwrap())
        .unwrap();
    assert!(larger.positions.len() > mesh.positions.len());
    let mut view = mesh.as_ref();
    view.positions = &larger.positions;
    view.triangles = &larger.triangles;
    assert_eq!(
        polygon_soup(&STRICT, &[view]).unwrap(),
        polygon_soup(&STRICT, &[fresh(&larger).as_ref()]).unwrap()
    );
}

#[test]
fn changed_vertices_invalidate_box_facts_without_polluting_original() {
    let original = cube();
    warm_validity(&original);
    let mut edited = original.clone();
    Arc::make_mut(&mut edited.positions)[0].x = Real::from(-1);
    let result = edited.axis_aligned_box_bounds(&STRICT).unwrap();
    assert_eq!(result.certainty, MeshCertainty::Certified);
    assert_eq!(result.value, None);
    assert_bounds(&edited, -1, 1);
    warm_validity(&original);
    assert_bounds(&original, 0, 1);
}

#[test]
fn wide_plane_schedule_is_recomputed_for_changed_coordinates() {
    use hyperreal::Rational;
    let mut mesh = cube();
    polygon_soup(&STRICT, &[mesh.as_ref()]).unwrap();
    let denominator = Rational::new(2).powi(2048_i64.into()).unwrap();
    let scale = Real::from((&denominator + Rational::one()) / &denominator);
    for point in Arc::make_mut(&mut mesh.positions) {
        point.x = &point.x * &scale;
        point.y = &point.y * &scale;
        point.z = &point.z * &scale;
    }
    assert!(points_require_wide_dyadic_plane_normalization(
        mesh.positions.iter()
    ));
    let actual = polygon_soup(&STRICT, &[mesh.as_ref()]).unwrap();
    let expected = polygon_soup(&STRICT, &[fresh(&mesh).as_ref()]).unwrap();
    assert_eq!(actual.certainty, MeshCertainty::Certified);
    assert_eq!(actual, expected);
}

#[test]
fn cache_guards_preserve_unchanged_facts_without_owning_source_buffers() {
    let mesh = cube();
    warm_validity(&mesh);
    let clone = mesh.clone();
    let cached_bounds = Arc::clone(mesh.facts.exact_bounds.get().unwrap().as_ref().unwrap());
    let mut edited = mesh.clone();
    edited.triangles = edited.triangles[1..].to_vec().into();
    assert_bounds(&edited, 0, 1);
    assert!(Arc::ptr_eq(
        edited.facts.exact_bounds.get().unwrap().as_ref().unwrap(),
        &cached_bounds
    ));
    assert_open(&edited);
    warm_validity(&clone);
    let source = Arc::downgrade(&mesh.positions);
    let facts = Arc::clone(&mesh.facts);
    drop(edited);
    drop(clone);
    drop(mesh);
    assert!(source.upgrade().is_none());
    // A weak identity keeps the allocation/control block until its last weak
    // owner drops, but does not hold a strong reference to the source slice.
    assert!(facts.source_positions.upgrade().is_none());
    println!(
        "mesh-fact-layout mesh={} facts={} weak_positions={} weak_triangles={}",
        std::mem::size_of::<TriangleMesh>(),
        std::mem::size_of::<TriangleMeshFacts>(),
        std::mem::size_of::<Weak<[Point3]>>(),
        std::mem::size_of::<Weak<[Triangle]>>()
    );
}
