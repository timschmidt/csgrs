//! The shipped user path of docs/admission.md, run against packaged crates:
//! construct, transform, compose, inspect certainty, validate topology,
//! export, reimport and check each format's contract.

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh, curve,
    io::{obj, ply, stl},
    solid::{self, SolidExt},
};
use std::io::Cursor;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn check(condition: bool, message: &str) -> Result<()> {
    if condition {
        Ok(())
    } else {
        Err(message.into())
    }
}

/// Six times the signed volume, in exact arithmetic. Boolean results may
/// contain rational Steiner vertices, so this is not an integer sum.
fn six_volume(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].clone().to_vector());
        sum + a.dot(&b.cross(&c))
    })
}

fn exactly(value: &Real, expected: i64) -> bool {
    let expected = Real::from(expected).exact_rational();
    expected.is_some() && value.exact_rational() == expected
}

/// The vertices of each binary STL facet, as written.
fn stl_facets(bytes: &[u8]) -> Vec<[[f32; 3]; 3]> {
    bytes[84..]
        .as_chunks::<50>()
        .0
        .iter()
        .map(|facet| {
            let value = |index: usize| {
                let at = 12 + 4 * index;
                f32::from_le_bytes(facet[at..at + 4].try_into().expect("four bytes"))
            };
            std::array::from_fn(|vertex| std::array::from_fn(|axis| value(3 * vertex + axis)))
        })
        .collect()
}

fn closed(mesh: &TriangleMesh, context: &GeometryContext) -> Result<bool> {
    Ok(mesh
        .is_closed_manifold_geometry(&context.mesh_context())?
        .into_value())
}

fn main() -> Result<()> {
    let context = GeometryContext::STRICT;

    // Construct.
    let body = solid::cube(Real::from(12))?;
    let bar = curve::try_extrude(&curve::square(Real::from(4))?, Real::from(16), &context)?
        .into_value();

    // Transform: a translation and an exact quarter turn.
    let drill = bar.translated(Real::from(4), Real::from(4), Real::from(-2))?;
    let cross = solid::try_rotate(&bar, Real::from(90), Real::zero(), Real::zero())?
        .translated(Real::from(4), Real::from(14), Real::from(4))?;

    // Compose, keeping the aggregate certainty of every consumed decision.
    let drilled = body.try_difference_with_context(&drill, &context)?;
    check(
        drilled.certainty == GeometryCertainty::Certified,
        "strict difference is not certified",
    )?;
    let part = drilled.value.try_difference_with_context(&cross, &context)?;
    check(
        part.certainty == GeometryCertainty::Certified,
        "second strict difference is not certified",
    )?;
    let part = part.into_value();

    // Validate topology and geometry: a closed manifold with the exact
    // volume 12^3 - 4*4*12 - (4*4*12 - 4*4*4).
    check(closed(&part, &context)?, "result is not a closed manifold")?;
    let expected = 6 * (1728 - 192 - (192 - 64));
    check(
        exactly(&six_volume(&part), expected),
        "result volume is not exact",
    )?;

    // Export and reimport through each triangle format.
    let binary = stl::to_stl_binary(&part, "part")?;
    check(
        binary.len() == 84 + 50 * part.triangles.len(),
        "binary STL size does not match its triangle count",
    )?;
    // Binary STL stores f32 coordinates. Reimport validates a closed surface
    // and must reproduce every stored value exactly, facet by facet.
    let from_stl = stl::from_stl(&binary)?;
    let facets = stl_facets(&binary);
    check(
        from_stl.triangles.len() == part.triangles.len()
            && facets.len() == part.triangles.len(),
        "STL reimport changed the triangle count",
    )?;
    for (triangle, facet) in from_stl.triangles.iter().zip(&facets) {
        for (index, stored) in triangle.indices().into_iter().zip(facet) {
            let point = &from_stl.positions[index];
            for (coordinate, stored) in [&point.x, &point.y, &point.z].into_iter().zip(stored)
            {
                check(
                    coordinate.exact_rational() == Real::try_from(*stored)?.exact_rational(),
                    "STL reimport does not reproduce the stored f32 coordinates",
                )?;
            }
        }
    }

    let text = obj::to_obj(&part, "part")?;
    let vertices = text.lines().filter(|line| line.starts_with("v ")).count();
    let faces = text.lines().filter(|line| line.starts_with("f ")).count();
    check(
        vertices == part.positions.len() && faces == part.triangles.len(),
        "OBJ text does not list every vertex and face",
    )?;
    // OBJ writes each coordinate's binary64 projection as an exact decimal, so
    // reimport preserves the projection: exporting it again is lossless.
    let from_obj = obj::from_obj(Cursor::new(text.as_bytes()))?;
    check(
        from_obj.triangles == part.triangles,
        "OBJ reimport changed the triangle indices",
    )?;
    check(
        obj::to_obj(&from_obj, "part")? == text,
        "OBJ reimport does not preserve the binary64 projection",
    )?;
    check(closed(&from_obj, &context)?, "OBJ reimport is not closed")?;

    // PLY writes per-corner vertices with generated normals. Its header must
    // declare exactly the rows that follow, with one face per triangle.
    let ply_text = ply::to_ply(&part, "part")?;
    let (header, body) = ply_text
        .split_once("end_header\n")
        .ok_or("PLY output has no end_header")?;
    let declared = |element: &str| -> Result<usize> {
        let line = header
            .lines()
            .find_map(|line| line.strip_prefix(&format!("element {element} ")))
            .ok_or_else(|| format!("PLY header declares no {element} element"))?;
        Ok(line.trim().parse()?)
    };
    let (vertex_count, face_count) = (declared("vertex")?, declared("face")?);
    let rows: Vec<&str> = body.lines().collect();
    check(
        face_count == part.triangles.len() && rows.len() == vertex_count + face_count,
        "PLY header counts do not match its body",
    )?;
    check(
        rows[..vertex_count]
            .iter()
            .all(|row| row.split_whitespace().count() == 6),
        "PLY vertex rows are not position and normal",
    )?;
    for row in &rows[vertex_count..] {
        let fields: Vec<usize> = row
            .split_whitespace()
            .map(str::parse)
            .collect::<std::result::Result<_, _>>()?;
        check(
            fields.len() == 4
                && fields[0] == 3
                && fields[1..].iter().all(|&index| index < vertex_count),
            "PLY face rows are not triangles over declared vertices",
        )?;
    }

    println!(
        "user path: {} vertices, {} triangles, certified, closed, exact volume; \
         STL and OBJ round trips and the PLY layout checked",
        part.positions.len(),
        part.triangles.len()
    );
    Ok(())
}
