use csgrs::mesh::{plane::Plane, polygon::Polygon, vertex::Vertex};
use nalgebra::{Point3, Vector3};

#[test]
fn degenerate_polygon_after_clipping() {
    let vertices = vec![
        Vertex::new(Point3::origin(), Vector3::y()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::y()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices.clone(), None);
    let plane = Plane::from_normal(Vector3::new(0.0, 0.0, 1.0), 0.0);

    eprintln!("Original polygon: {:?}", polygon);
    eprintln!("Clipping plane: {:?}", plane);

    let (_coplanar_front, _coplanar_back, front, back) = plane.split_polygon(&polygon);

    eprintln!("Front polygons: {:?}", front);
    eprintln!("Back polygons: {:?}", back);

    assert!(front.is_empty(), "Front should be empty for this test");
    assert!(back.is_empty(), "Back should be empty for this test");
}

#[test]
fn valid_polygon_clipping() {
    let vertices = vec![
        Vertex::new(Point3::origin(), Vector3::y()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::y()),
        Vertex::new(Point3::new(0.5, 1.0, 0.0), Vector3::y()),
    ];

    let polygon: Polygon<()> = Polygon::new(vertices, None);

    let plane = Plane::from_normal(-Vector3::y(), -0.5);

    eprintln!("Polygon before clipping: {:?}", polygon);
    eprintln!("Clipping plane: {:?}", plane);

    let (_coplanar_front, _coplanar_back, front, back) = plane.split_polygon(&polygon);

    eprintln!("Front polygons: {:?}", front);
    eprintln!("Back polygons: {:?}", back);

    assert!(!front.is_empty(), "Front should not be empty");
    assert!(!back.is_empty(), "Back should not be empty");
}
