//! Structural quotas precede exact scalar and geometry reconstruction.

use hyperbrep::{
    ArchiveLimitExceeded, ArchiveLimits, ArchiveResource, Curve3, ModelBuilder, PersistenceError,
    Point3, RawModel, Real, builder,
};
use serde_json::{Value, json};

fn empty() -> Value {
    serde_json::from_str(&ModelBuilder::new().finish().unwrap().to_json().unwrap()).unwrap()
}

fn bad_array(n: usize) -> Value {
    json!(vec!["not an exact scalar"; n])
}

fn rejected(raw: &Value, limits: &ArchiveLimits, resource: ArchiveResource, observed: usize) {
    let allowance = match resource {
        ArchiveResource::Records => limits.records,
        ArchiveResource::Degree => limits.degree,
        ArchiveResource::GeometryElements => limits.geometry_elements,
        ArchiveResource::TopologyReferences => limits.topology_references,
        _ => unreachable!("structural quota test"),
    };
    assert_eq!(
        RawModel::from_json(&raw.to_string(), limits).unwrap_err(),
        PersistenceError::ResourceLimit(ArchiveLimitExceeded {
            resource,
            limit: allowance,
            observed,
        })
    );
}

#[test]
fn cuboid_exact_record_and_reference_boundaries_preserve_bytes_and_volume() {
    // 8 vertices, 12 curves, 24 pcurves, 6 surfaces, 12 edges, 24 uses,
    // 6 wires, 6 faces, 1 shell, 1 solid. Variable lists: 24 uses + 6 faces.
    let model =
        builder::cuboid(Point3::origin(), Point3::new(2.into(), 3.into(), 4.into())).unwrap();
    let json = model.model().to_json().unwrap();
    let raw: Value = serde_json::from_str(&json).unwrap();
    let limits = ArchiveLimits {
        records: 100,
        degree: 0,
        geometry_elements: 0,
        topology_references: 30,
        ..ArchiveLimits::default()
    };
    rejected(
        &raw,
        &ArchiveLimits {
            records: 99,
            ..limits
        },
        ArchiveResource::Records,
        100,
    );
    rejected(
        &raw,
        &ArchiveLimits {
            topology_references: 29,
            ..limits
        },
        ArchiveResource::TopologyReferences,
        30,
    );
    let loaded = RawModel::from_json(&json, &limits)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(loaded.to_json().unwrap(), json);
    assert_eq!(
        loaded.solids().next().unwrap().volume().unwrap(),
        Real::from(24)
    );
}

#[test]
fn a_valid_nurbs_retries_after_degree_and_geometry_exhaustion() {
    let mut builder = ModelBuilder::new();
    builder
        .curve(
            Curve3::nurbs(
                2,
                vec![
                    Point3::origin(),
                    Point3::new(1.into(), 2.into(), 0.into()),
                    Point3::new(2.into(), 0.into(), 0.into()),
                ],
                [1, 2, 1].map(Real::from).to_vec(),
                [2, 2, 2, 5, 5, 5].map(Real::from).to_vec(),
            )
            .unwrap(),
        )
        .unwrap();
    let json = builder.finish().unwrap().to_json().unwrap();
    let raw: Value = serde_json::from_str(&json).unwrap();
    let limits = ArchiveLimits {
        records: 1,
        degree: 2,
        geometry_elements: 12,
        ..ArchiveLimits::default()
    };
    rejected(
        &raw,
        &ArchiveLimits {
            degree: 1,
            ..limits
        },
        ArchiveResource::Degree,
        2,
    );
    rejected(
        &raw,
        &ArchiveLimits {
            geometry_elements: 11,
            ..limits
        },
        ArchiveResource::GeometryElements,
        12,
    );
    let replay = RawModel::from_json(&json, &limits)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replay.to_json().unwrap(), json);
}

#[test]
fn every_spline_storage_family_is_admitted_before_its_invalid_scalars() {
    let lattice = json!([bad_array(4), bad_array(4), bad_array(4)]);
    for (arena, geometry, degree, elements) in [
        (
            "curves",
            json!({"RationalBezier":{"control_points":bad_array(5),"weights":bad_array(5)}}),
            4,
            10,
        ),
        (
            "curves",
            json!({"Nurbs":{"degree":11,"control_points":bad_array(3),"weights":bad_array(3),"knots":bad_array(7)}}),
            11,
            13,
        ),
        (
            "pcurves",
            json!({"RationalBezier":{"homogeneous_controls":bad_array(4)}}),
            3,
            4,
        ),
        (
            "pcurves",
            json!({"RationalQuadraticBezier":{"control_points":bad_array(3),"weights":bad_array(3)}}),
            2,
            6,
        ),
        (
            "pcurves",
            json!({"Nurbs":{"degree":5,"homogeneous_controls":bad_array(6),"knots":bad_array(12)}}),
            5,
            18,
        ),
        (
            "surfaces",
            json!({"RationalBezier":{"control_points":lattice,"weights":lattice}}),
            3,
            30,
        ),
        (
            "surfaces",
            json!({"Nurbs":{"u_degree":3,"v_degree":9,"control_points":lattice,"weights":lattice,"u_knots":bad_array(8),"v_knots":bad_array(9)}}),
            9,
            47,
        ),
    ] {
        let mut raw = empty();
        raw[arena] = json!([geometry]);
        let limits = ArchiveLimits {
            degree,
            geometry_elements: elements,
            ..ArchiveLimits::default()
        };
        rejected(
            &raw,
            &ArchiveLimits {
                degree: degree - 1,
                ..limits
            },
            ArchiveResource::Degree,
            degree,
        );
        rejected(
            &raw,
            &ArchiveLimits {
                geometry_elements: elements - 1,
                ..limits
            },
            ArchiveResource::GeometryElements,
            elements,
        );
        // The admitted shape still has deliberately invalid scalar payloads:
        // full schema/scalar validation must run, never publish a trusted model.
        assert!(matches!(
            RawModel::from_json(&raw.to_string(), &limits),
            Err(PersistenceError::Json(_))
        ));
    }
}

#[test]
fn topology_lists_are_charged_before_index_decoding() {
    for (arena, record) in [
        ("wires", json!({"edge_uses":bad_array(3)})),
        ("faces", json!({"inner":bad_array(3)})),
        ("shells", bad_array(3)),
        ("solids", json!({"voids":bad_array(3)})),
    ] {
        let mut raw = empty();
        raw[arena] = json!([record]);
        rejected(
            &raw,
            &ArchiveLimits {
                topology_references: 2,
                ..ArchiveLimits::default()
            },
            ArchiveResource::TopologyReferences,
            3,
        );
        assert!(matches!(
            RawModel::from_json(&raw.to_string(), &ArchiveLimits::default()),
            Err(PersistenceError::Json(_))
        ));
    }
}

#[test]
fn detached_sources_roots_and_inline_profiles_share_one_total() {
    for surface in ["Extrusion", "Revolution"] {
        let bezier =
            json!({"RationalBezier":{"control_points":bad_array(2),"weights":bad_array(2)}});
        let mut raw = empty();
        raw["curves"] = json!([bezier]);
        raw["selection"]["point_sources"] = json!([{"Nurbs":{"degree":1,"control_points":bad_array(2),"weights":bad_array(2),"knots":bad_array(4)}}]);
        raw["selection"]["pcurve_sources"] =
            json!([{"RationalBezier":{"homogeneous_controls":bad_array(2)}}]);
        raw["selection"]["parameters"] = json!([{"Root":{"polynomial":bad_array(3)}}]);
        raw["surfaces"] = json!([{surface:{"profile":bezier}}]);
        // 4 + 8 + 2 + 3 + 4 entries; six records include the inline profile.
        rejected(
            &raw,
            &ArchiveLimits {
                records: 5,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Records,
            6,
        );
        rejected(
            &raw,
            &ArchiveLimits {
                geometry_elements: 20,
                ..ArchiveLimits::default()
            },
            ArchiveResource::GeometryElements,
            21,
        );
        rejected(
            &raw,
            &ArchiveLimits {
                degree: 1,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Degree,
            2,
        );
    }
}

#[test]
fn empty_tensor_rows_and_declared_extreme_degree_cannot_bypass_admission() {
    let mut raw = empty();
    raw["surfaces"] = json!([{"RationalBezier":{"control_points":[[],[],[],[]],"weights":[]}}]);
    rejected(
        &raw,
        &ArchiveLimits {
            geometry_elements: 3,
            ..ArchiveLimits::default()
        },
        ArchiveResource::GeometryElements,
        4,
    );
    raw["surfaces"] = json!([]);
    raw["curves"] =
        json!([{"Nurbs":{"degree":usize::MAX,"control_points":[],"weights":[],"knots":[]}}]);
    rejected(
        &raw,
        &ArchiveLimits::default(),
        ArchiveResource::Degree,
        usize::MAX,
    );
}

#[test]
fn every_arena_record_is_charged_before_payload_decoding() {
    for arena in [
        "vertices",
        "curves",
        "pcurves",
        "surfaces",
        "edges",
        "edge_uses",
        "wires",
        "faces",
        "shells",
        "solids",
    ] {
        let mut raw = empty();
        raw[arena] = json!(["invalid record"]);
        rejected(
            &raw,
            &ArchiveLimits {
                records: 0,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Records,
            1,
        );
    }
    for arena in ["parameters", "point_sources", "pcurve_sources"] {
        let mut raw = empty();
        raw["selection"][arena] = json!(["invalid record"]);
        rejected(
            &raw,
            &ArchiveLimits {
                records: 0,
                ..ArchiveLimits::default()
            },
            ArchiveResource::Records,
            1,
        );
    }
}

#[test]
fn version_admission_is_independent_of_field_order_and_rejects_ambiguity() {
    let limits = ArchiveLimits::default();
    for raw in [
        r#"{"version":8,"curves":["obsolete record"]}"#,
        r#"{"curves":["obsolete record"],"version":8}"#,
    ] {
        assert_eq!(
            RawModel::from_json(raw, &limits).unwrap_err(),
            PersistenceError::UnsupportedVersion(8)
        );
    }
    for raw in [
        r#"{"version":9,"version":9}"#,
        r#"{"version":9,"version":8}"#,
        r#"{"version":8,"version":9}"#,
        r#"{"curves":[]}"#,
    ] {
        assert!(matches!(
            RawModel::from_json(raw, &limits),
            Err(PersistenceError::Json(_))
        ));
    }
    let raw = empty();
    let canonical = raw.to_string();
    // The independent Value serializer orders keys lexically (version late).
    // Our encoder writes version first; both must admit the same exact model.
    let first = format!("{{\"version\":9,{}", &canonical[1..]);
    let first = first.replacen(",\"version\":9", "", 1);
    for input in [&canonical, &first] {
        let model = RawModel::from_json(input, &limits)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            model.to_json().unwrap(),
            RawModel::from_json(&canonical, &limits)
                .unwrap()
                .validate()
                .unwrap()
                .to_json()
                .unwrap()
        );
    }
}
