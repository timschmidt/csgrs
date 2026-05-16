//! Tests for metadata preservation and provenance.

use super::support::*;

#[test]
fn test_polygon_metadata_string() {
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
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
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
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
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(verts, my_data.clone());

    assert_eq!(poly.metadata(), &my_data);
}

#[test]
fn test_csg_construction_with_metadata() {
    let poly_a = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
        ],
        "PolyA".to_string(),
    );
    let poly_b = Polygon::new(
        vec![
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(3.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(3.0, 1.0, 0.0), Vector3::z()),
        ],
        "PolyB".to_string(),
    );
    let csg = Mesh::from_polygons(&[poly_a.clone(), poly_b.clone()], "Mesh".to_string());

    assert_eq!(csg.polygons.len(), 2);
    assert_eq!(csg.polygons[0].metadata(), &"PolyA".to_string());
    assert_eq!(csg.polygons[1].metadata(), &"PolyB".to_string());
}

#[test]
fn test_union_metadata() {
    let cube1 = Mesh::cube(1.0, "Cube1".to_string());
    let cube2 = Mesh::cube(1.0, "Cube2".to_string()).translate(0.5, 0.0, 0.0);

    let union_csg = cube1.union(&cube2);

    for poly in &union_csg.polygons {
        let data = poly.metadata();
        assert!(
            data == "Cube1" || data == "Cube2",
            "Union polygon has unexpected shared data = {:?}",
            data
        );
    }
}

#[test]
fn test_difference_metadata() {
    let cube1 = Mesh::cube(2.0, "Cube1".to_string());
    let cube2 = Mesh::cube(2.0, "Cube2".to_string()).translate(0.5, 0.5, 0.5);

    let result = cube1.difference(&cube2);

    let mut saw_cube1 = false;
    let mut saw_cube2 = false;
    for poly in &result.polygons {
        let metadata = poly.metadata();
        saw_cube1 |= metadata == "Cube1";
        saw_cube2 |= metadata == "Cube2";
        assert!(
            metadata == "Cube1" || metadata == "Cube2",
            "Difference polygon has unexpected metadata = {:?}",
            metadata
        );
    }

    assert!(
        saw_cube1 && saw_cube2,
        "Difference should retain source metadata from both operands for clipped boundary faces"
    );
}

#[test]
fn test_intersect_metadata() {
    let cube1 = Mesh::cube(2.0, "Cube1".to_string());
    let cube2 = Mesh::cube(2.0, "Cube2".to_string()).translate(0.5, 0.5, 0.5);

    let result = cube1.intersection(&cube2);

    for poly in &result.polygons {
        let data = poly.metadata();
        assert!(
            data == "Cube1" || data == "Cube2",
            "Intersection polygon has unexpected shared data = {:?}",
            data
        );
    }
}

#[test]
fn test_flip_invert_metadata() {
    let csg = Mesh::cube(2.0, "MyCube".to_string());

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
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
        ],
        "LargeQuad".to_string(),
    );
    let csg = Mesh::from_polygons(&[poly], "Mesh".to_string());
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
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        "Tri".to_string(),
    );
    let csg = Mesh::from_polygons(&[poly], "Mesh".to_string());
    let csg_trans = csg.translate(10.0, 5.0, 0.0);
    let csg_scale = csg_trans.scale(2.0, 2.0, 1.0);
    let csg_rot = csg_scale.rotate(0.0, 0.0, 45.0);

    for poly in &csg_rot.polygons {
        assert_eq!(poly.metadata(), &"Tri".to_string());
    }
}

#[test]
#[cfg(feature = "nurbs")]
fn test_nurbs_metadata_preserved_by_backend_ops() {
    let rect = crate::nurbs::Nurbs::rectangle(2.0, 2.0, "rect");
    let moved = rect.translate(1.0, 0.0, 0.0);
    assert_eq!(moved.metadata, "rect");

    let remapped = moved.map_metadata(|metadata| format!("{metadata}-mapped"));
    assert_eq!(remapped.metadata, "rect-mapped");

    let replaced = remapped.with_metadata("replacement");
    assert_eq!(replaced.metadata, "replacement");

    let cutter = crate::nurbs::Nurbs::rectangle(1.0, 1.0, "cutter");
    let difference = replaced.try_difference(&cutter).unwrap();
    assert_eq!(difference.metadata, "replacement");
}

#[test]
#[cfg(all(feature = "nurbs", feature = "sketch", feature = "mesh"))]
fn test_nurbs_metadata_preserved_by_conversions() {
    let rect = crate::nurbs::Nurbs::rectangle(2.0, 2.0, "profile");

    let sketch = rect.to_sketch();
    assert_eq!(sketch.metadata, "profile");

    let sketch_precise = rect.to_sketch_with_tolerance(Some(1e-3));
    assert_eq!(sketch_precise.metadata, "profile");

    let mesh = rect.extrude_vector(Vector3::new(0.0, 0.0, 1.0));
    assert_eq!(mesh.metadata, "profile");
    assert!(
        mesh.polygons
            .iter()
            .all(|polygon| polygon.metadata() == &"profile")
    );

    let mesh_precise =
        rect.extrude_vector_with_tolerance(Vector3::new(0.0, 0.0, 1.0), Some(1e-3));
    assert_eq!(mesh_precise.metadata, "profile");
    assert!(
        mesh_precise
            .polygons
            .iter()
            .all(|polygon| polygon.metadata() == &"profile")
    );
}

#[test]
fn test_complex_metadata_struct_in_boolean_ops() {
    #[derive(Debug, Clone, PartialEq)]
    struct Color(u8, u8, u8);

    let csg1 = Mesh::cube(2.0, Color(255, 0, 0));
    let csg2 = Mesh::cube(2.0, Color(0, 255, 0)).translate(0.5, 0.5, 0.5);

    let unioned = csg1.union(&csg2);
    for poly in &unioned.polygons {
        let col = poly.metadata();
        assert!(
            *col == Color(255, 0, 0) || *col == Color(0, 255, 0),
            "Unexpected color in union: {:?}",
            col
        );
    }
}
