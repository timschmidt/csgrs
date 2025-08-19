use csgrs::{
    mesh::{Mesh, polygon::Polygon, vertex::Vertex},
    traits::CSG,
};
use nalgebra::{Point3, Vector3};

/// A small, custom metadata type to demonstrate usage.
/// We derive PartialEq so we can assert equality in tests.
#[derive(Debug, Clone, PartialEq)]
struct MyMetaData {
    id: u32,
    label: String,
}

#[test]
fn string_metadata() {
    // Create a simple triangle polygon with shared data = Some("triangle".to_string()).
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let mut poly = Polygon::new(verts, Some("triangle".to_string()));

    // Check getter
    assert_eq!(poly.metadata(), Some(&"triangle".to_string()));

    // Check setter
    poly.set_metadata("updated".to_string());
    assert_eq!(poly.metadata(), Some(&"updated".to_string()));

    // Check mutable getter
    if let Some(data) = poly.metadata_mut() {
        data.push_str("_appended");
    }
    assert_eq!(poly.metadata(), Some(&"updated_appended".to_string()));
}

#[test]
fn integer_metadata() {
    // Create a polygon with integer shared data
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(verts, Some(42u32));

    // Confirm data
    assert_eq!(poly.metadata(), Some(&42));
}

#[test]
fn struct_metadata() {
    // Create a polygon with our custom struct
    let my_data = MyMetaData {
        id: 999,
        label: "MyLabel".into(),
    };
    let verts = vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ];
    let poly = Polygon::new(verts, Some(my_data.clone()));

    assert_eq!(poly.metadata(), Some(&my_data));
}

#[test]
fn construction_with_metadata() {
    // Build a Mesh of two polygons, each with distinct shared data.
    let poly_a = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(1.0, 1.0, 0.0), Vector3::z()),
        ],
        Some("PolyA".to_string()),
    );
    let poly_b = Polygon::new(
        vec![
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(3.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(3.0, 1.0, 0.0), Vector3::z()),
        ],
        Some("PolyB".to_string()),
    );
    let csg = Mesh::from_polygons(&[poly_a.clone(), poly_b.clone()], None);

    // We expect two polygons with the same shared data as the originals.
    assert_eq!(csg.polygons.len(), 2);
    assert_eq!(csg.polygons[0].metadata(), Some(&"PolyA".to_string()));
    assert_eq!(csg.polygons[1].metadata(), Some(&"PolyB".to_string()));
}

#[test]
fn union() {
    // Let's union two squares in the XY plane, each with different shared data.
    // So after union, we typically get polygons from each original shape.
    // If there's any overlap, new polygons might be formed, but in Sketch
    // each new polygon inherits the shared data from whichever polygon it came from.

    // Cube1 from (0,0) to (1,1) => label "Cube1"
    let cube1 = Mesh::cube(1.0, None); // bottom-left at (0,0), top-right at (1,1)
    let mut cube1 = cube1; // now let us set shared data for each polygon
    for p in &mut cube1.polygons {
        p.set_metadata("Cube1".to_string());
    }

    // Translate Cube2 so it partially overlaps. => label "Cube2"
    let cube2 = Mesh::cube(1.0, None).translate(0.5, 0.0, 0.0);
    let mut cube2 = cube2;
    for p in &mut cube2.polygons {
        p.set_metadata("Cube2".to_string());
    }

    // Union
    let union_csg = cube1.union(&cube2);

    // Depending on the library's polygon splitting, we often end up with multiple polygons.
    // We can at least confirm that each polygon's shared data is EITHER "Square1" or "Square2",
    // and never mixed or lost.
    for poly in &union_csg.polygons {
        let data = poly.metadata().unwrap();
        assert!(
            data == "Cube1" || data == "Cube2",
            "Union polygon has unexpected shared data = {:?}",
            data
        );
    }
}

#[test]
fn difference() {
    // Difference two cubes, each with different shared data. The resulting polygons
    // come from the *minuend* (the first shape) with *some* portion clipped out.
    // So the differenced portion from the second shape won't appear in the final.

    let mut cube1 = Mesh::cube(2.0, Some("Cube1".to_string()));
    for p in &mut cube1.polygons {
        p.set_metadata("Cube1".to_string());
    }

    let mut cube2 = Mesh::cube(2.0, Some("Cube2".to_string())).translate(0.5, 0.5, 0.5);
    for p in &mut cube2.polygons {
        p.set_metadata("Cube2".to_string());
    }

    let result = cube1.difference(&cube2);

    println!("{:#?}", cube1);
    println!("{:#?}", cube2);
    println!("{:#?}", result);

    // All polygons in the result should come from "Cube1" only.
    for poly in &result.polygons {
        assert_eq!(poly.metadata(), Some(&"Cube1".to_string()));
    }
}

#[test]
fn intersect() {
    // Intersection: the resulting polygons should come from polygons that are inside both.
    // Typically, the library picks polygons from the first shape, then clips them
    // against the second. Depending on exact implementation, the polygons that remain
    // carry the first shape's shared data. In many CSG implementations, the final polygons
    // keep the "side" from whichever shape is relevant. That might be shape A or B or both.
    // We'll check that we only see "Cube1" or "Cube2" but not random data.

    let mut cube1 = Mesh::cube(2.0, None);
    for p in &mut cube1.polygons {
        p.set_metadata("Cube1".to_string());
    }

    let mut cube2 = Mesh::cube(2.0, None).translate(0.5, 0.5, 0.5);
    for p in &mut cube2.polygons {
        p.set_metadata("Cube2".to_string());
    }

    let result = cube1.intersection(&cube2);

    // Depending on the implementation, it's common that intersection polygons are
    // actually from both shapes or from shape A. Let's check that if they do have shared data,
    // it must be from either "Cube1" or "Cube2".
    for poly in &result.polygons {
        let data = poly.metadata().unwrap();
        assert!(
            data == "Cube1" || data == "Cube2",
            "Intersection polygon has unexpected shared data = {:?}",
            data
        );
    }
}

#[test]
fn flip_invert() {
    // Flipping or inverting a shape should NOT change the shared data;
    // it only flips normals/polygons.

    let mut csg = Mesh::cube(2.0, None);
    for p in &mut csg.polygons {
        p.set_metadata("MyCube".to_string());
    }

    // Invert
    let inverted = csg.inverse();
    for poly in &inverted.polygons {
        assert_eq!(poly.metadata(), Some(&"MyCube".to_string()));
    }
}

#[test]
fn subdivide() {
    // Subdivide a polygon with shared data, ensure all new subdivided polygons
    // preserve that data.

    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(2.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(2.0, 2.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 2.0, 0.0), Vector3::z()),
        ],
        Some("LargeQuad".to_string()),
    );
    let csg = Mesh::from_polygons(&[poly], None);
    let subdivided = csg.subdivide_triangles(1.try_into().expect("not 0")); // one level of subdivision

    // Now it's split into multiple triangles. Each should keep "LargeQuad" as metadata.
    assert!(subdivided.polygons.len() > 1);
    for spoly in &subdivided.polygons {
        assert_eq!(spoly.metadata(), Some(&"LargeQuad".to_string()));
    }
}

#[test]
fn transform() {
    // Make sure that transform does not lose or change shared data.
    let poly = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
        ],
        Some("Tri".to_string()),
    );
    let csg = Mesh::from_polygons(&[poly], None);
    let csg_trans = csg.translate(10.0, 5.0, 0.0);
    let csg_scale = csg_trans.scale(2.0, 2.0, 1.0);
    let csg_rot = csg_scale.rotate(0.0, 0.0, 45.0);

    for poly in &csg_rot.polygons {
        assert_eq!(poly.metadata(), Some(&"Tri".to_string()));
    }
}

#[test]
fn struct_in_boolean_ops() {
    // We'll do an operation using a custom struct to verify it remains intact.
    // We'll do a union for instance.

    #[derive(Debug, Clone, PartialEq)]
    struct Color(u8, u8, u8);

    let mut csg1 = Mesh::cube(2.0, None);
    for p in &mut csg1.polygons {
        p.set_metadata(Color(255, 0, 0));
    }
    let mut csg2 = Mesh::cube(2.0, None).translate(0.5, 0.5, 0.5);
    for p in &mut csg2.polygons {
        p.set_metadata(Color(0, 255, 0));
    }

    let unioned = csg1.union(&csg2);
    // Now polygons are either from csg1 (red) or csg2 (green).
    for poly in &unioned.polygons {
        let col = poly.metadata().unwrap();
        assert!(
            *col == Color(255, 0, 0) || *col == Color(0, 255, 0),
            "Unexpected color in union: {:?}",
            col
        );
    }
}
