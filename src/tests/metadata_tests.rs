//! Tests for metadata preservation and provenance.

use super::support::*;

#[test]
fn test_polygon_metadata_string() {
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let mut poly = Polygon::new(verts, "triangle".to_string());

    assert_eq!(poly.metadata(), &"triangle".to_string());

    poly.set_metadata("updated".to_string());
    assert_eq!(poly.metadata(), &"updated".to_string());

    poly.metadata_mut().push_str("_appended");
    assert_eq!(poly.metadata(), &"updated_appended".to_string());
}

#[test]
fn test_polygon_metadata_integer() {
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(verts, 42u32);

    assert_eq!(poly.metadata(), &42);
}

#[test]
fn test_polygon_metadata_custom_struct() {
    let my_data = MyMetaData {
        id: 999,
        label: "MyLabel".into(),
    };
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(verts, my_data.clone());

    assert_eq!(poly.metadata(), &my_data);
}

#[test]
fn test_csg_construction_with_metadata() {
    let poly_a = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(1.0, 1.0, 0.0), Vector3::z()),
        ],
        "PolyA".to_string(),
    );
    let poly_b = Polygon::new(
        vec![
            Vertex::new(p3(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(3.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(3.0, 1.0, 0.0), Vector3::z()),
        ],
        "PolyB".to_string(),
    );
    let csg = Mesh::from_polygons(vec![poly_a.clone(), poly_b.clone()]);

    assert_eq!(csg.polygons.len(), 2);
    assert_eq!(csg.polygons[0].metadata(), &"PolyA".to_string());
    assert_eq!(csg.polygons[1].metadata(), &"PolyB".to_string());
}

#[test]
fn test_union_metadata() {
    let cube1 = Mesh::cube(r(1.0), "Cube1".to_string());
    let cube2 = Mesh::cube(r(1.0), "Cube2".to_string()).translate(r(0.5), r(0.0), r(0.0));

    let union_csg = cube1.union(&cube2);

    let metadata = union_csg
        .polygons
        .iter()
        .map(|polygon| polygon.metadata().as_str())
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(metadata, std::collections::BTreeSet::from(["Cube1", "Cube2"]));
}

#[test]
fn test_difference_metadata() {
    let cube1 = Mesh::cube(r(2.0), "Cube1".to_string());
    let cube2 = Mesh::cube(r(2.0), "Cube2".to_string()).translate(r(0.5), r(0.5), r(0.5));

    let result = cube1.difference(&cube2);

    let metadata = result
        .polygons
        .iter()
        .map(|polygon| polygon.metadata().as_str())
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(metadata, std::collections::BTreeSet::from(["Cube1", "Cube2"]));
}

#[test]
fn test_intersect_metadata() {
    let cube1 = Mesh::cube(r(2.0), "Cube1".to_string());
    let cube2 = Mesh::cube(r(2.0), "Cube2".to_string()).translate(r(0.5), r(0.5), r(0.5));

    let result = cube1.intersection(&cube2);

    let metadata = result
        .polygons
        .iter()
        .map(|polygon| polygon.metadata().as_str())
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(metadata, std::collections::BTreeSet::from(["Cube1", "Cube2"]));
}

#[test]
fn test_empty_union_does_not_relabel_faces() {
    let empty = Mesh::<String>::empty();
    let cube = Mesh::cube(r(1.0), "right".to_string());

    let result = empty.union(&cube);

    assert!(
        result
            .polygons
            .iter()
            .all(|polygon| polygon.metadata() == "right")
    );
}

#[test]
fn test_flip_invert_metadata() {
    let csg = Mesh::cube(r(2.0), "MyCube".to_string());

    let inverted = csg.inverse();
    for poly in &inverted.polygons {
        assert_eq!(poly.metadata(), &"MyCube".to_string());
    }
}

#[test]
fn test_subdivide_metadata() {
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(2.0, 2.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 2.0, 0.0), Vector3::z()),
        ],
        "LargeQuad".to_string(),
    );
    let csg = Mesh::from_polygons(vec![poly]);
    let subdivided = csg.subdivide_triangles(1.try_into().expect("not 0"));

    assert!(subdivided.polygons.len() > 1);
    for spoly in &subdivided.polygons {
        assert_eq!(spoly.metadata(), &"LargeQuad".to_string());
    }
}

#[test]
fn test_transform_metadata() {
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
        ],
        "Tri".to_string(),
    );
    let csg = Mesh::from_polygons(vec![poly]);
    let csg_trans = csg.translate(r(10.0), r(5.0), r(0.0));
    let csg_scale = csg_trans.scale(r(2.0), r(2.0), r(1.0));
    let csg_rot = csg_scale.rotate(r(0.0), r(0.0), r(45.0));

    for poly in &csg_rot.polygons {
        assert_eq!(poly.metadata(), &"Tri".to_string());
    }
}

#[test]
fn test_complex_metadata_struct_in_boolean_ops() {
    #[derive(Debug, Clone, PartialEq)]
    struct Color(u8, u8, u8);

    let csg1 = Mesh::cube(r(2.0), Color(255, 0, 0));
    let csg2 = Mesh::cube(r(2.0), Color(0, 255, 0)).translate(r(0.5), r(0.5), r(0.5));

    let unioned = csg1.union(&csg2);
    assert!(
        unioned
            .polygons
            .iter()
            .any(|polygon| polygon.metadata() == &Color(255, 0, 0))
    );
    assert!(
        unioned
            .polygons
            .iter()
            .any(|polygon| polygon.metadata() == &Color(0, 255, 0))
    );
}
