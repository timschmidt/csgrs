//! Example: Multi-Format Export Demo  
//!
//! This example demonstrates exporting Mesh objects to multiple 3D file formats:
//! OBJ (universal format), PLY (research/scanning), and AMF (3D printing format).
//! These formats can be opened in most 3D modeling software, CAD programs, and 3D viewers.
use csgrs::mesh::Mesh;
use csgrs::csg::CSG;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Multi-Format Export Demo");
    println!("========================");
    println!();

    // Create various Mesh objects to demonstrate OBJ export

    // 1. Simple cube
    let cube: Mesh<()> = Mesh::cube(20.0, None).center();
    export_to_obj(&cube, "cube", "Simple 20x20x20mm cube")?;

    // 2. Sphere
    let sphere: Mesh<()> = Mesh::sphere(15.0, 32, 16, None);
    export_to_obj(&sphere, "sphere", "Sphere with 15mm radius")?;

    // 3. Cylinder
    let cylinder: Mesh<()> = Mesh::cylinder(8.0, 25.0, 24, None);
    export_to_obj(&cylinder, "cylinder", "Cylinder: 8mm radius, 25mm height")?;

    // 4. Complex boolean operation: cube with spherical cavity
    let cube_large: Mesh<()> = Mesh::cube(30.0, None).center();
    let sphere_cavity: Mesh<()> = Mesh::sphere(12.0, 24, 12, None).translate(5.0, 5.0, 0.0);
    let cube_with_cavity = cube_large.difference(&sphere_cavity);
    export_to_obj(
        &cube_with_cavity,
        "cube_with_cavity",
        "30mm cube with 12mm spherical cavity",
    )?;

    // 5. Union operation: cube + sphere
    let cube_small: Mesh<()> = Mesh::cube(16.0, None).center();
    let sphere_union: Mesh<()> = Mesh::sphere(10.0, 20, 10, None).translate(8.0, 8.0, 8.0);
    let union_object = cube_small.union(&sphere_union);
    export_to_obj(
        &union_object,
        "cube_sphere_union",
        "Union of 16mm cube and 10mm sphere",
    )?;

    // 6. Intersection operation
    let cube_intersect: Mesh<()> = Mesh::cube(25.0, None).center();
    let sphere_intersect: Mesh<()> = Mesh::sphere(15.0, 24, 12, None).translate(5.0, 5.0, 0.0);
    let intersection_object = cube_intersect.intersection(&sphere_intersect);
    export_to_obj(
        &intersection_object,
        "cube_sphere_intersection",
        "Intersection of cube and sphere",
    )?;

    // 7. More complex shape: cube with cylindrical hole
    let cube_base: Mesh<()> = Mesh::cube(40.0, None).center();
    let hole_cylinder: Mesh<()> = Mesh::cylinder(6.0, 50.0, 16, None)
        .rotate(90.0, 0.0, 0.0) // Rotate to align with X-axis
        .translate(0.0, 0.0, 0.0);
    let cube_with_hole = cube_base.difference(&hole_cylinder);
    export_to_obj(
        &cube_with_hole,
        "cube_with_hole",
        "40mm cube with 12mm diameter hole",
    )?;

    println!();
    println!("Export Summary");
    println!("==============");
    println!("Created files in multiple 3D formats:");
    println!();

    println!("OBJ Format (Universal 3D):");
    println!("  • cube.obj - Basic primitive");
    println!("  • sphere.obj - Spherical primitive");
    println!("  • cylinder.obj - Cylindrical primitive");
    println!("  • cube_with_cavity.obj - Difference operation");
    println!("  • cube_sphere_union.obj - Union operation");
    println!("  • cube_sphere_intersection.obj - Intersection operation");
    println!("  • cube_with_hole.obj - Complex drilling operation");
    println!();

    println!("PLY Format (Research/Scanning):");
    println!("  • cube.ply - Basic primitive with normals");
    println!("  • sphere.ply - High-detail spherical mesh");
    println!("  • cylinder.ply - Cylindrical primitive");
    println!("  • cube_with_cavity.ply - Boolean difference");
    println!("  • cube_sphere_union.ply - Union operation");
    println!("  • cube_sphere_intersection.ply - Intersection operation");
    println!("  • cube_with_hole.ply - Complex drilling operation");
    println!();

    println!("AMF Format (3D Printing):");
    println!("  • cube.amf - Basic primitive (XML format)");
    println!("  • sphere.amf - High-detail spherical mesh");
    println!("  • cylinder.amf - Cylindrical primitive");
    println!("  • cube_with_cavity.amf - Boolean difference");
    println!("  • cube_sphere_union.amf - Union operation");
    println!("  • cube_sphere_intersection.amf - Intersection operation");
    println!("  • cube_with_hole.amf - Complex drilling operation");
    println!();

    println!("Compatible Software:");
    println!("  • 3D Modeling: Blender, Maya, 3ds Max, Cinema 4D");
    println!("  • CAD: AutoCAD, SolidWorks, Fusion 360, FreeCAD");
    println!("  • Analysis: MeshLab, CloudCompare, ParaView");
    println!("  • Research: Open3D, PCL, VTK-based tools");
    println!("  • 3D Printing: PrusaSlicer, Cura, Simplify3D, Netfabb");
    println!("  • Online: Various web-based 3D viewers");

    Ok(())
}

fn export_to_obj(
    csg: &Mesh<()>,
    name: &str,
    description: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    // Export OBJ format
    #[cfg(feature = "obj-io")]
    {
        use std::fs::File;

        let filename = format!("{}.obj", name);
        let mut file = File::create(&filename)?;
        csg.write_obj(&mut file, name)?;

        println!("✓ Exported {}: {}", filename, description);

        // Print some statistics
        let obj_content = csg.to_obj(name);
        let vertex_count = obj_content
            .lines()
            .filter(|line| line.starts_with("v "))
            .count();
        let face_count = obj_content
            .lines()
            .filter(|line| line.starts_with("f "))
            .count();
        let normal_count = obj_content
            .lines()
            .filter(|line| line.starts_with("vn "))
            .count();

        println!(
            "  OBJ Stats: {} vertices, {} faces, {} normals",
            vertex_count, face_count, normal_count
        );
    }

    #[cfg(not(feature = "obj-io"))]
    {
        println!("⚠ OBJ export not available - 'obj-io' feature not enabled");
    }

    // Export PLY format
    #[cfg(feature = "ply-io")]
    {
        use std::fs::File;

        let filename = format!("{}.ply", name);
        let mut file = File::create(&filename)?;
        csg.write_ply(&mut file, description)?;

        println!("✓ Exported {}: {}", filename, description);

        // Print some statistics
        let ply_content = csg.to_ply(description);
        let vertex_count = ply_content
            .lines()
            .filter(|line| {
                !line.starts_with("ply")
                    && !line.starts_with("format")
                    && !line.starts_with("comment")
                    && !line.starts_with("element")
                    && !line.starts_with("property")
                    && !line.starts_with("end_header")
                    && !line.starts_with("3 ")
                    && !line.trim().is_empty()
            })
            .count();
        let face_count = ply_content
            .lines()
            .filter(|line| line.starts_with("3 "))
            .count();

        println!(
            "  PLY Stats: {} vertices, {} triangles with normals",
            vertex_count, face_count
        );
    }

    #[cfg(not(feature = "ply-io"))]
    {
        println!("⚠ PLY export not available - 'ply-io' feature not enabled");
    }

    // Export AMF format
    #[cfg(feature = "amf-io")]
    {
        use std::fs::File;

        let filename = format!("{}.amf", name);
        let mut file = File::create(&filename)?;
        csg.write_amf(&mut file, name, "millimeter")?;

        println!("✓ Exported {}: {}", filename, description);

        // Print some statistics
        let amf_content = csg.to_amf(name, "millimeter");
        let vertex_count = amf_content.matches("<vertex id=").count();
        let triangle_count = amf_content.matches("<triangle id=").count();

        println!(
            "  AMF Stats: {} vertices, {} triangles (XML format)",
            vertex_count, triangle_count
        );
    }

    #[cfg(not(feature = "amf-io"))]
    {
        println!("⚠ AMF export not available - 'amf-io' feature not enabled");
    }

    println!("  Description: {}", description);
    println!();

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_obj_export() {
        // Test basic OBJ export functionality
        let cube: Mesh<()> = Mesh::cube(10.0, None);

        #[cfg(feature = "obj-io")]
        {
            let obj_content = cube.to_obj("test_cube");

            // Check that OBJ content contains expected elements
            assert!(obj_content.contains("o test_cube"));
            assert!(obj_content.contains("v ")); // Should have vertices
            assert!(obj_content.contains("vn ")); // Should have normals
            assert!(obj_content.contains("f ")); // Should have faces

            // Check header
            assert!(obj_content.contains("# Generated by csgrs library"));
            assert!(obj_content.contains("# Object: test_cube"));
        }
    }

    #[test]
    fn test_obj_content_format() {
        let sphere: Mesh<()> = Mesh::sphere(5.0, 8, 4, None); // Low res for testing

        #[cfg(feature = "obj-io")]
        {
            let obj_content = sphere.to_obj("test_sphere");

            // Verify OBJ format structure
            let lines: Vec<&str> = obj_content.lines().collect();

            // Should have object declaration
            assert!(lines.iter().any(|line| line.starts_with("o test_sphere")));

            // Should have vertices (format: v x y z)
            let vertex_lines: Vec<_> =
                lines.iter().filter(|line| line.starts_with("v ")).collect();
            assert!(!vertex_lines.is_empty());

            // Check vertex format
            for vertex_line in vertex_lines.iter().take(3) {
                let parts: Vec<&str> = vertex_line.split_whitespace().collect();
                assert_eq!(parts[0], "v");
                assert!(parts.len() >= 4); // v x y z

                // Should be parseable as floats
                assert!(parts[1].parse::<f64>().is_ok());
                assert!(parts[2].parse::<f64>().is_ok());
                assert!(parts[3].parse::<f64>().is_ok());
            }

            // Should have normals (format: vn x y z)
            let normal_lines: Vec<_> =
                lines.iter().filter(|line| line.starts_with("vn ")).collect();
            assert!(!normal_lines.is_empty());

            // Should have faces (format: f v1//n1 v2//n2 v3//n3)
            let face_lines: Vec<_> =
                lines.iter().filter(|line| line.starts_with("f ")).collect();
            assert!(!face_lines.is_empty());

            // Check face format
            for face_line in face_lines.iter().take(3) {
                let parts: Vec<&str> = face_line.split_whitespace().collect();
                assert_eq!(parts[0], "f");
                assert!(parts.len() >= 4); // f v1//n1 v2//n2 v3//n3

                // Check face vertex format (should be number//number)
                for vertex_ref in &parts[1..] {
                    assert!(vertex_ref.contains("//"));
                }
            }
        }
    }

    #[test]
    fn test_boolean_operations_obj_export() {
        // Test that boolean operations export correctly
        let cube: Mesh<()> = Mesh::cube(10.0, None);
        let sphere: Mesh<()> = Mesh::sphere(6.0, 8, 4, None);

        #[cfg(feature = "obj-io")]
        {
            // Test union
            let union_result = cube.union(&sphere);
            let union_obj = union_result.to_obj("union_test");
            assert!(union_obj.contains("o union_test"));
            assert!(union_obj.contains("v "));
            assert!(union_obj.contains("f "));

            // Test difference
            let diff_result = cube.difference(&sphere);
            let diff_obj = diff_result.to_obj("diff_test");
            assert!(diff_obj.contains("o diff_test"));
            assert!(diff_obj.contains("v "));
            assert!(diff_obj.contains("f "));

            // Test intersection
            let intersect_result = cube.intersection(&sphere);
            let intersect_obj = intersect_result.to_obj("intersect_test");
            assert!(intersect_obj.contains("o intersect_test"));
            assert!(intersect_obj.contains("v "));
            assert!(intersect_obj.contains("f "));
        }
    }

    #[test]
    fn test_ply_export() {
        // Test basic PLY export functionality
        let cube: Mesh<()> = Mesh::cube(10.0, None);

        #[cfg(feature = "ply-io")]
        {
            let ply_content = cube.to_ply("Test cube for PLY export");

            // Check that PLY content contains expected elements
            assert!(ply_content.contains("ply"));
            assert!(ply_content.contains("format ascii 1.0"));
            assert!(ply_content.contains("comment Test cube for PLY export"));
            assert!(ply_content.contains("comment Generated by csgrs library"));
            assert!(ply_content.contains("element vertex"));
            assert!(ply_content.contains("element face"));
            assert!(ply_content.contains("property float x"));
            assert!(ply_content.contains("property float y"));
            assert!(ply_content.contains("property float z"));
            assert!(ply_content.contains("property float nx"));
            assert!(ply_content.contains("property float ny"));
            assert!(ply_content.contains("property float nz"));
            assert!(ply_content.contains("end_header"));

            // Check data content
            let lines: Vec<&str> = ply_content.lines().collect();
            let data_lines: Vec<_> = lines
                .iter()
                .skip_while(|line| **line != "end_header")
                .skip(1) // Skip the "end_header" line itself
                .collect();

            // Should have vertex data and face data
            assert!(!data_lines.is_empty());

            // Check that we have triangular faces (should start with "3 ")
            let face_lines: Vec<_> = data_lines
                .iter()
                .filter(|line| line.starts_with("3 "))
                .collect();
            assert!(!face_lines.is_empty());
        }
    }

    #[test]
    fn test_ply_format_structure() {
        let sphere: Mesh<()> = Mesh::sphere(5.0, 8, 4, None); // Low res for testing

        #[cfg(feature = "ply-io")]
        {
            let ply_content = sphere.to_ply("Test sphere");

            // Verify PLY format structure
            let lines: Vec<&str> = ply_content.lines().collect();

            // Check header structure
            assert_eq!(lines[0], "ply");
            assert_eq!(lines[1], "format ascii 1.0");
            assert!(lines[2].starts_with("comment Test sphere"));
            assert_eq!(lines[3], "comment Generated by csgrs library");

            // Find vertex and face counts
            let vertex_line = lines
                .iter()
                .find(|line| line.starts_with("element vertex"))
                .unwrap();
            let face_line = lines
                .iter()
                .find(|line| line.starts_with("element face"))
                .unwrap();

            let vertex_count: usize = vertex_line
                .split_whitespace()
                .nth(2)
                .unwrap()
                .parse()
                .unwrap();
            let face_count: usize =
                face_line.split_whitespace().nth(2).unwrap().parse().unwrap();

            assert!(vertex_count > 0);
            assert!(face_count > 0);

            // Check that data section contains the right number of items
            let header_end = lines.iter().position(|line| *line == "end_header").unwrap();
            let data_lines = &lines[header_end + 1..];

            // Count vertex data lines (lines with 6 float values: x y z nx ny nz)
            let vertex_data_lines: Vec<_> = data_lines
                .iter()
                .filter(|line| {
                    let parts: Vec<_> = line.split_whitespace().collect();
                    parts.len() == 6 && parts.iter().all(|p| p.parse::<f64>().is_ok())
                })
                .collect();

            // Count face data lines (lines starting with "3 ")
            let face_data_lines: Vec<_> = data_lines
                .iter()
                .filter(|line| line.starts_with("3 "))
                .collect();

            assert_eq!(vertex_data_lines.len(), vertex_count);
            assert_eq!(face_data_lines.len(), face_count);
        }
    }

    #[test]
    fn test_amf_export() {
        // Test basic AMF export functionality
        let cube: Mesh<()> = Mesh::cube(10.0, None);

        #[cfg(feature = "amf-io")]
        {
            let amf_content = cube.to_amf("test_cube", "millimeter");

            // Check that AMF content contains expected XML elements
            assert!(amf_content.contains("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
            assert!(amf_content.contains("<amf unit=\"millimeter\" version=\"1.1\">"));
            assert!(amf_content.contains("<object id=\"test_cube\">"));
            assert!(amf_content.contains("<mesh>"));
            assert!(amf_content.contains("<vertices>"));
            assert!(amf_content.contains("<volume>"));
            assert!(amf_content.contains("</amf>"));

            // Check metadata
            assert!(
                amf_content.contains("<metadata type=\"producer\">csgrs library</metadata>")
            );
            assert!(
                amf_content
                    .contains("<metadata type=\"cad\">Constructive Solid Geometry</metadata>")
            );

            // Check that vertices and triangles are present
            assert!(amf_content.contains("<vertex id="));
            assert!(amf_content.contains("<coordinates>"));
            assert!(amf_content.contains("<triangle id="));
            assert!(amf_content.contains("<v1>"));
            assert!(amf_content.contains("<v2>"));
            assert!(amf_content.contains("<v3>"));
        }
    }

    #[test]
    fn test_amf_with_color() {
        let sphere: Mesh<()> = Mesh::sphere(5.0, 8, 4, None); // Low res for testing

        #[cfg(feature = "amf-io")]
        {
            let amf_content =
                sphere.to_amf_with_color("red_sphere", "millimeter", (1.0, 0.0, 0.0));

            // Check that color/material information is present
            assert!(amf_content.contains("<material id=\"material1\">"));
            assert!(amf_content.contains("<color>"));
            assert!(amf_content.contains("<r>1.000</r>"));
            assert!(amf_content.contains("<g>0.000</g>"));
            assert!(amf_content.contains("<b>0.000</b>"));
            assert!(amf_content.contains("<a>1.0</a>"));
            assert!(amf_content.contains("</color>"));
            assert!(amf_content.contains("</material>"));

            // Check that volume references the material
            assert!(amf_content.contains("<volume materialid=\"material1\">"));

            // Verify overall structure
            assert!(amf_content.contains("<object id=\"red_sphere\">"));
            assert!(amf_content.contains("</object>"));
        }
    }

    #[test]
    fn test_amf_xml_structure() {
        let cube: Mesh<()> = Mesh::cube(8.0, None);

        #[cfg(feature = "amf-io")]
        {
            let amf_content = cube.to_amf("test_structure", "inch");

            // Verify proper XML structure and hierarchy
            assert!(amf_content.contains("<amf unit=\"inch\""));

            // Count vertices and triangles
            let vertex_count = amf_content.matches("<vertex id=").count();
            let triangle_count = amf_content.matches("<triangle id=").count();

            assert!(vertex_count > 0);
            assert!(triangle_count > 0);

            // Basic cube should have 8 vertices and 12 triangles (2 per face * 6 faces)
            assert_eq!(vertex_count, 8);
            assert_eq!(triangle_count, 12);

            // Verify XML closing tags match opening tags
            assert!(amf_content.contains("</vertices>"));
            assert!(amf_content.contains("</volume>"));
            assert!(amf_content.contains("</mesh>"));
            assert!(amf_content.contains("</object>"));
            assert!(amf_content.contains("</amf>"));
        }
    }
}
