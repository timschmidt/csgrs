//! Verify a fixture expressible through all primitive-scalar bindings.
use csgrs::{
    adapter::{AdapterError, TriangleMeshF64},
    errors::{ValidationError, ValidationErrorKind},
    solid::SolidExt,
};

fn main() {
    let cube = TriangleMeshF64::cube(1.0).unwrap();
    let rotated = cube.rotate(0.0, 0.0, 13.0).unwrap();
    let restored = rotated.rotate(0.0, 0.0, -13.0).unwrap();
    for operation in ["union", "difference", "intersection", "xor"] {
        let native: ValidationError = match operation {
            "union" => cube.native().try_union(restored.native()),
            "difference" => cube.native().try_difference(restored.native()),
            "intersection" => cube.native().try_intersection(restored.native()),
            "xor" => cube.native().try_xor(restored.native()),
            _ => unreachable!(),
        }
        .err()
        .expect("fixture must remain natively undecided")
        .into();
        assert_eq!(native.kind(), ValidationErrorKind::Undecided);
        let adapter = match operation {
            "union" => cube.union(&restored),
            "difference" => cube.difference(&restored),
            "intersection" => cube.intersection(&restored),
            "xor" => cube.xor(&restored),
            _ => unreachable!(),
        }
        .err()
        .expect("failed Boolean cannot publish geometry");
        assert_eq!(adapter.to_string(), native.to_string());
        let structured = matches!(adapter, AdapterError::Construction(_));
        println!("{operation}: native=Undecided structured={structured} diagnostic={native}");
    }
    // The failure does not poison a subsequent independently provable query.
    assert!(cube.difference(&cube).unwrap().native().triangles.is_empty());
}
