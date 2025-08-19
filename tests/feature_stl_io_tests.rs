#![cfg(feature = "stl-io")]

use csgrs::mesh::Mesh;

#[test]
fn mesh_to_stl_ascii() {
    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let stl_str = cube.to_stl_ascii("test_cube");
    // Basic checks
    assert!(stl_str.contains("solid test_cube"));
    assert!(stl_str.contains("endsolid test_cube"));

    // Should contain some facet normals
    assert!(stl_str.contains("facet normal"));
    // Should contain some vertex lines
    assert!(stl_str.contains("vertex"));
}

#[test]
fn to_stl_and_from_stl_file() -> Result<(), Box<dyn std::error::Error>> {
    // We'll create a small shape, write to an STL, read it back.
    // You can redirect to a temp file or do an in-memory test.
    let tmp_path = "output.stl";

    let cube: Mesh<()> = Mesh::cube(2.0, None);
    let res = cube.to_stl_binary("A cube");
    let _ = std::fs::write(tmp_path, res.as_ref().unwrap());
    assert!(res.is_ok());

    let stl_data: Vec<u8> = std::fs::read(tmp_path)?;
    let csg_in: Mesh<()> = Mesh::from_stl(&stl_data, None)?;
    // We expect to read the same number of triangular faces as the cube originally had
    // (though the orientation/normals might differ).
    // The default cube -> 6 polygons x 1 polygon each with 4 vertices => 12 triangles in STL.
    // So from_stl_file => we get 12 triangles as 12 polygons (each is a tri).
    assert_eq!(csg_in.polygons.len(), 12);

    // Cleanup the temp file if desired
    let _ = std::fs::remove_file(tmp_path);
    Ok(())
}
