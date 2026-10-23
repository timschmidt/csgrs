//! The AMF decimal spelling preserves the selected binary64 projection.
#![cfg(feature = "amf-io")]

use csgrs::{
    GeometryContext, Real, TriangleMesh,
    io::{IoError, amf},
    solid,
};
use hyperlattice::Point3;
use hypermesh::{MeshCertainty, Triangle};

fn real(value: f64) -> Real {
    Real::try_from(value).unwrap()
}

/// Text between each `<tag>` and `</tag>`, in document order.
fn elements<'a>(text: &'a str, tag: &str) -> Vec<&'a str> {
    let open = format!("<{tag}>");
    let close = format!("</{tag}>");
    text.split(&open)
        .skip(1)
        .map(|rest| rest.split_once(&close).unwrap().0)
        .collect()
}

/// Reads vertices and triangles back as a consumer would, without CSGRS.
fn parsed(text: &str) -> (Vec<[&str; 3]>, Vec<[usize; 3]>) {
    let [x, y, z] = ["x", "y", "z"].map(|tag| elements(text, tag));
    let vertices = (0..x.len()).map(|index| [x[index], y[index], z[index]]);
    let [a, b, c] = ["v1", "v2", "v3"].map(|tag| elements(text, tag));
    let triangles = (0..a.len()).map(|index| {
        [a[index], b[index], c[index]].map(|field| field.parse::<usize>().unwrap())
    });
    (vertices.collect(), triangles.collect())
}

/// Requires every field to spell its binary64 value exactly.
fn assert_fields(text: &str, expected: &[[f64; 3]]) {
    let (vertices, _) = parsed(text);
    assert_eq!(vertices.len(), expected.len());
    for (fields, expected) in vertices.iter().zip(expected) {
        for axis in 0..3 {
            assert_eq!(
                fields[axis].parse::<f64>().unwrap().to_bits(),
                expected[axis].to_bits()
            );
            assert_eq!(fields[axis].parse::<Real>().unwrap(), real(expected[axis]));
            assert!(fields[axis].len() <= 1077, "binary64 decimal field length");
        }
    }
}

fn triangle(value: f64, axis: usize) -> (TriangleMesh, [[f64; 3]; 3]) {
    let value = if value == 0.0 { 0.0 } else { value }; // Real has unsigned zero.
    let mut rows = [[0.0; 3]; 3];
    for row in &mut rows {
        row[axis] = value;
    }
    rows[1][(axis + 1) % 3] = 1.0;
    rows[2][(axis + 2) % 3] = 1.0;
    let mesh = TriangleMesh::new(
        rows.map(|row| Point3::new(real(row[0]), real(row[1]), real(row[2])))
            .to_vec(),
        vec![Triangle::new(0, 1, 2)],
    );
    (mesh, rows)
}

#[test]
fn every_finite_exponent_family_and_boundary_values_preserve_exact_fields() {
    let mut state = 0x2718_2818_2845_9045_u64;
    let mut sample = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    for exponent in 0..2047_u64 {
        for sign in [0, 1_u64 << 63] {
            let value =
                f64::from_bits(sign | (exponent << 52) | (sample() & ((1_u64 << 52) - 1)));
            let (mesh, rows) = triangle(value, exponent as usize % 3);
            assert_fields(&amf::to_amf(&mesh, "family", "meter").unwrap(), &rows);
        }
    }
    for value in [
        f64::from_bits(1),
        f64::MIN_POSITIVE,
        2_f64.powi(-70),
        1e-18,
        0.1,
        1.0 / 3.0,
        f64::MAX,
    ] {
        for sign in [1.0, -1.0] {
            let (mesh, rows) = triangle(sign * value, 2);
            assert_fields(&amf::to_amf(&mesh, "boundary", "inch").unwrap(), &rows);
        }
    }
}

#[test]
fn already_exact_decimal_artifacts_stay_byte_identical() {
    for value in [0.0, -1.5, 2_f64.powi(-17), 2_f64.powi(60), f64::MAX] {
        let (mesh, rows) = triangle(value, 0);
        let mut expected = String::from(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<amf unit=\"millimeter\" version=\"1.1\">\n<metadata type=\"producer\">csgrs</metadata>\n<metadata type=\"description\">stable</metadata>\n<material id=\"1\"><color><r>0.50000000000000000</r><g>0.25000000000000000</g><b>1.00000000000000000</b><a>1</a></color></material>\n<object id=\"0\"><mesh><vertices>\n",
        );
        for [x, y, z] in rows {
            expected.push_str(&format!(
                "<vertex><coordinates><x>{x:.17}</x><y>{y:.17}</y><z>{z:.17}</z></coordinates></vertex>\n"
            ));
        }
        expected.push_str("</vertices><volume materialid=\"1\">\n<triangle><v1>0</v1><v2>1</v2><v3>2</v3></triangle>\n</volume></mesh></object></amf>\n");
        let color = (real(0.5), real(0.25), Real::one());
        assert_eq!(
            amf::to_amf_with_color(&mesh, "stable", "millimeter", color).unwrap(),
            expected
        );
    }
}

#[test]
fn color_components_spell_their_projection() {
    let (mesh, _) = triangle(1.0, 0);
    let tenth = (Real::one() / Real::from(10)).unwrap();
    let tiny = real(2_f64.powi(-70));
    let text =
        amf::to_amf_with_color(&mesh, "color", "meter", (tenth, tiny, Real::zero())).unwrap();
    let [red, green] = ["r", "g"].map(|tag| elements(&text, tag)[0]);
    assert_eq!(red.parse::<f64>().unwrap().to_bits(), 0.1_f64.to_bits());
    assert_eq!(green.parse::<Real>().unwrap(), real(2_f64.powi(-70)));
}

#[test]
fn tiny_closed_body_stays_closed_after_an_external_read() {
    // With 17 fixed fractional digits every coordinate of this cube was
    // written as zero, and the reread surface had no nondegenerate face.
    let side = real(2_f64.powi(-70));
    let source = solid::cube(side.clone()).unwrap();
    let text = amf::to_amf(&source, "tiny cube", "meter").unwrap();
    let (vertices, triangles) = parsed(&text);
    let restored = TriangleMesh::new(
        vertices
            .iter()
            .map(|fields| {
                let [x, y, z] = fields.map(|field| field.parse::<Real>().unwrap());
                Point3::new(x, y, z)
            })
            .collect(),
        triangles
            .iter()
            .map(|&[a, b, c]| Triangle::new(a, b, c))
            .collect(),
    );
    assert_eq!(restored.positions, source.positions);
    let context = GeometryContext::STRICT.mesh_context();
    assert_eq!(
        restored.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    let report = solid::exact_mass_properties(&restored, Real::one()).unwrap();
    assert_eq!(report.volume, &side * &side * &side);
}

#[test]
fn failures_remain_explicit_before_any_bytes_are_written() {
    let (mut source, _) = triangle(1.0, 0);
    let huge = format!("1{}", "0".repeat(1000)).parse::<Real>().unwrap();
    for point in std::sync::Arc::make_mut(&mut source.positions) {
        point.x = huge.clone();
    }
    let mut sink = b"unchanged".to_vec();
    assert!(matches!(
        amf::write_amf(&source, &mut sink, "overflow", "meter"),
        Err(IoError::UnrepresentableCoordinate {
            format: "AMF",
            field: "vertex x",
            target: "f64"
        })
    ));
    assert!(matches!(
        amf::write_amf(&triangle(1.0, 0).0, &mut sink, "units", "furlong"),
        Err(IoError::Unsupported { format: "AMF", .. })
    ));
    assert_eq!(sink, b"unchanged");
}
