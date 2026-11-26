//! Format-valid empty scenes, hostile input, and continued exact modeling.
#![cfg(feature = "gltf-io")]

#[path = "support/uncertainty.rs"]
/// Shared fixture that keeps a known zero unresolved at the finite decision terminal.
mod uncertainty;

use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh,
    io::{IoError, gltf::*},
    solid::{self, SolidExt},
};
use hyperlattice::Point3;
use hypermesh::{BooleanOp, MeshCertainty, Triangle};
use serde_json::{Value, json};

/// Serializes a JSON fixture and sends it through the public glTF importer.
fn import(value: &Value) -> Result<GltfMeshImport, IoError> {
    from_gltf(&serde_json::to_vec(value).unwrap())
}

/// Checks canonical empty storage and an outside classification at the origin.
fn assert_empty(mesh: &TriangleMesh) {
    assert!(mesh.positions.is_empty() && mesh.triangles.is_empty());
    assert!(!solid::contains_point(mesh, &Point3::origin()).unwrap());
}

/// Checks exact boundary planes and signed volume independently of serialization agreement.
/// Closed-manifold and PWN validation establish separate claims. Integer fixture
/// bounds must keep the expected-volume expression within i32 arithmetic range.
fn assert_box(mesh: &TriangleMesh, min: [i32; 3], max: [i32; 3]) {
    let lower = min.map(Real::from);
    let upper = max.map(Real::from);
    let mut six_volume = Real::zero();
    for triangle in mesh.triangles.iter() {
        let points = triangle.indices().map(|index| &mesh.positions[index]);
        let coordinates = points.map(|point| [&point.x, &point.y, &point.z]);
        for point in coordinates {
            for axis in 0..3 {
                assert!(point[axis] >= &lower[axis] && point[axis] <= &upper[axis]);
            }
        }
        assert!((0..3).any(|axis| {
            coordinates.iter().all(|p| p[axis] == &lower[axis])
                || coordinates.iter().all(|p| p[axis] == &upper[axis])
        }));
        let [a, b, c] = points.map(|point| point.to_vector());
        six_volume += a.dot(&b.cross(&c));
    }
    assert_eq!(
        six_volume,
        Real::from(6 * (max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2]))
    );
    let context = GeometryContext::STRICT.mesh_context();
    let manifold = mesh.is_closed_manifold_geometry(&context).unwrap();
    assert!(manifold.value);
    assert_eq!(manifold.certainty, MeshCertainty::Certified);
    assert_eq!(
        mesh.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
}

/// Independently assembles a version-2 GLB header and four-byte-padded chunks.
/// If `binary`, moves the fixture's first embedded buffer into BIN and removes
/// its URI. Fixture lengths must fit u32; this is not a general glTF packager.
fn glb(mut value: Value, binary: bool) -> Vec<u8> {
    let bin = binary.then(|| {
        let uri = value["buffers"][0]
            .as_object_mut()
            .unwrap()
            .remove("uri")
            .unwrap();
        BASE64
            .decode(uri.as_str().unwrap().split_once(',').unwrap().1)
            .unwrap()
    });
    let mut json = serde_json::to_vec(&value).unwrap();
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let mut chunks = Vec::new();
    chunks.extend_from_slice(&u32::try_from(json.len()).unwrap().to_le_bytes());
    chunks.extend_from_slice(b"JSON");
    chunks.extend_from_slice(&json);
    if let Some(mut bytes) = bin {
        while !bytes.len().is_multiple_of(4) {
            bytes.push(0);
        }
        chunks.extend_from_slice(&u32::try_from(bytes.len()).unwrap().to_le_bytes());
        chunks.extend_from_slice(b"BIN\0");
        chunks.extend_from_slice(&bytes);
    }
    let mut bytes = b"glTF".to_vec();
    bytes.extend_from_slice(&2u32.to_le_bytes());
    bytes.extend_from_slice(&u32::try_from(12 + chunks.len()).unwrap().to_le_bytes());
    bytes.extend_from_slice(&chunks);
    bytes
}

#[test]
/// Imports schema-valid empty JSON/JSON-only GLB with exact counts and missing-history evidence.
/// Empty scenes do not falsely claim coordinate projection or predicate certainty.
fn schema_valid_empty_json_and_json_only_glb_import_as_canonical_empty() {
    for value in [
        json!({"asset":{"version":"2.0"},"scenes":[{}],"scene":0}),
        json!({"asset":{"version":"2.0"},"scenes":[{"nodes":[0]}],
            "nodes":[{"name":"empty","translation":[2,3,4]}]}),
    ] {
        for bytes in [
            serde_json::to_vec(&value).unwrap(),
            glb(value.clone(), false),
        ] {
            let result = from_gltf(&bytes).unwrap();
            assert_empty(&result.mesh);
            assert_eq!(
                (
                    result.scene_index,
                    result.mesh_node_count,
                    result.primitive_count
                ),
                (0, 0, 0)
            );
            let evidence = result.mesh.construction_evidence();
            assert!(evidence.source_history_missing());
            assert!(!evidence.coordinates_projected());
            assert_eq!(evidence.predicate_certainty(), None);
        }
    }
}

#[test]
/// Checks omission of absent geometry arrays while retaining empty node/scene names and replay.
fn empty_scene_and_named_empty_objects_omit_geometry_without_losing_names() {
    let empty = solid::empty();
    let scene: Value = serde_json::from_str(&to_gltf_scene("nothing", &[]).unwrap()).unwrap();
    assert_eq!(scene["scenes"][0], json!({"name":"nothing"}));
    assert!(scene.get("nodes").is_none());
    assert_empty(&import(&scene).unwrap().mesh);
    for text in [
        to_gltf(&empty, "empty").unwrap(),
        to_gltf_scene(
            "empties",
            &[
                GltfSceneObject::new("first", &empty).unwrap(),
                GltfSceneObject::new("second", &empty).unwrap(),
            ],
        )
        .unwrap(),
    ] {
        let value: Value = serde_json::from_str(&text).unwrap();
        for field in ["meshes", "buffers", "bufferViews", "accessors"] {
            assert!(value.get(field).is_none(), "{field}");
        }
        let names: Vec<_> = value["nodes"]
            .as_array()
            .unwrap()
            .iter()
            .map(|node| {
                assert!(node.get("mesh").is_none());
                node["name"].as_str().unwrap()
            })
            .collect();
        assert!(names == ["empty"] || names == ["first", "second"]);
        assert_empty(&from_gltf(text.as_bytes()).unwrap().mesh);
    }
}

#[test]
/// Checks independent node/mesh indices in mixed empty/two-box scenes across JSON and GLB.
/// Verifies each box geometrically, then subtracts to empty and reuses that result.
fn mixed_scene_preserves_node_and_mesh_indices_and_continues_modeling() {
    let cube = solid::cube(1).unwrap();
    let moved = cube.translated(3, 2, 1).unwrap();
    let empty = solid::empty();
    let text = to_gltf_scene(
        "mixed",
        &[
            GltfSceneObject::new("before", &empty).unwrap(),
            GltfSceneObject::new("cube", &cube).unwrap(),
            GltfSceneObject::new("between", &empty).unwrap(),
            GltfSceneObject::new("moved", &moved).unwrap(),
            GltfSceneObject::new("after", &empty).unwrap(),
        ],
    )
    .unwrap();
    let value: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(
        value["nodes"],
        json!([
            {"name":"before"},{"name":"cube","mesh":0},{"name":"between"},
            {"name":"moved","mesh":1},{"name":"after"}
        ])
    );
    assert_eq!(value["scenes"][0]["nodes"], json!([0, 1, 2, 3, 4]));
    for bytes in [text.into_bytes(), glb(value, true)] {
        let imported = from_gltf(&bytes).unwrap();
        assert_eq!((imported.mesh_node_count, imported.primitive_count), (2, 2));
        imported
            .mesh
            .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
            .unwrap();
        let mut assigned = 0;
        for (min, max) in [([0; 3], [1; 3]), ([3, 2, 1], [4, 3, 2])] {
            let triangles: Vec<_> = imported
                .mesh
                .triangles
                .iter()
                .copied()
                .filter(|triangle| {
                    triangle.indices().iter().all(|&index| {
                        let point = &imported.mesh.positions[index];
                        [&point.x, &point.y, &point.z].iter().enumerate().all(
                            |(axis, value)| {
                                **value >= Real::from(min[axis])
                                    && **value <= Real::from(max[axis])
                            },
                        )
                    })
                })
                .collect();
            assigned += triangles.len();
            assert_box(
                &TriangleMesh::new(imported.mesh.positions.to_vec(), triangles),
                min,
                max,
            );
        }
        assert_eq!(assigned, imported.mesh.triangles.len());
        let remainder = imported.mesh.try_difference(&cube).unwrap();
        assert_box(&remainder, [3, 2, 1], [4, 3, 2]);
        let removed = remainder.try_difference(&moved).unwrap();
        assert_empty(&removed);
        assert_box(&removed.try_union(&cube).unwrap(), [0; 3], [1; 3]);
    }
}

#[test]
/// Reimports a terminal-qualified empty result as explicitly missing history.
/// Strict transform/Boolean identities and repeated replay must retain that
/// qualification; the original approximation receipt is not serialized by glTF.
fn empty_roundtrip_preserves_qualified_history_through_strict_identities() {
    let cube = solid::cube(1).unwrap();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let mut points = cube.positions.to_vec();
    points.push(Point3::new(
        uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one()),
        0.into(),
        0.into(),
    ));
    let approximate =
        hypermesh::convex_hull(&GeometryContext::APPROXIMATE_512.mesh_context(), &points)
            .unwrap();
    assert_eq!(approximate.certainty, MeshCertainty::Approximate512Consumed);
    let removed = approximate.value.try_difference(&cube).unwrap();
    assert_empty(&removed);
    assert!(removed.construction_evidence().approximate_512_consumed());
    let serialized = to_gltf(&removed, "removed").unwrap();
    let imported = from_gltf(serialized.as_bytes()).unwrap().mesh;
    // glTF has no source receipt. Import explicitly reports missing history;
    // later local proofs cannot certify the original construction's intent.
    assert!(imported.construction_evidence().source_history_missing());
    assert_eq!(imported.construction_evidence().predicate_certainty(), None);
    let transformed = solid::scale(&imported, 2, 3, 4).unwrap();
    for (left, right, operation, nonempty) in [
        (&cube, &transformed, BooleanOp::Union, true),
        (&transformed, &cube, BooleanOp::Union, true),
        (&cube, &transformed, BooleanOp::Intersection, false),
        (&cube, &transformed, BooleanOp::Difference, true),
        (&transformed, &cube, BooleanOp::Difference, false),
    ] {
        let result =
            solid::boolean_with_context(left, right, operation, GeometryContext::STRICT)
                .unwrap();
        assert_eq!(result.certainty, GeometryCertainty::Certified);
        assert!(result.value.construction_evidence().source_history_missing());
        if nonempty {
            assert_box(&result.value, [0; 3], [1; 3]);
        } else {
            assert_empty(&result.value);
        }
        let next = from_gltf(to_gltf(&result.value, "next").unwrap().as_bytes())
            .unwrap()
            .mesh;
        assert!(next.construction_evidence().source_history_missing());
        if nonempty {
            assert_box(&next, [0; 3], [1; 3]);
        } else {
            assert_empty(&next);
        }
    }
}

/// Assembles a unit right-triangle document with independent attribute/index bytes.
/// `component` selects implicit indices or unsigned 5121/5123/5125 encoding.
/// Sparse positions override two zero-base rows; otherwise `interleaved` adds
/// a sentinel fourth float. Other component codes are invalid fixture requests.
fn triangle_document(component: Option<u32>, interleaved: bool, sparse: bool) -> Value {
    let positions = [[0f32, 0., 0.], [1., 0., 0.], [0., 1., 0.]];
    let mut bytes = Vec::new();
    let mut views = Vec::new();
    let position_accessor = if sparse {
        bytes.extend_from_slice(&[1, 2, 0, 0]);
        for point in &positions[1..] {
            for value in point {
                bytes.extend_from_slice(&value.to_le_bytes());
            }
        }
        views.push(json!({"buffer":0,"byteOffset":0,"byteLength":2}));
        views.push(json!({"buffer":0,"byteOffset":4,"byteLength":24}));
        json!({"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0],
            "sparse":{"count":2,"indices":{"bufferView":0,"componentType":5121},"values":{"bufferView":1}}})
    } else {
        for point in positions {
            for value in point {
                bytes.extend_from_slice(&value.to_le_bytes());
            }
            if interleaved {
                bytes.extend_from_slice(&999f32.to_le_bytes());
            }
        }
        let mut view = json!({"buffer":0,"byteLength":bytes.len(),"target":34962});
        if interleaved {
            view["byteStride"] = json!(16);
        }
        views.push(view);
        json!({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]})
    };
    let mut accessors = vec![position_accessor];
    let mut primitive = json!({"attributes":{"POSITION":0}});
    if let Some(component) = component {
        let offset = bytes.len();
        for index in [2u32, 0, 1] {
            let encoded = index.to_le_bytes();
            bytes.extend_from_slice(
                &encoded[..match component {
                    5121 => 1,
                    5123 => 2,
                    5125 => 4,
                    _ => unreachable!(),
                }],
            );
        }
        accessors.push(json!({"bufferView":views.len(),"componentType":component,"count":3,"type":"SCALAR"}));
        views.push(json!({"buffer":0,"byteOffset":offset,"byteLength":bytes.len()-offset,"target":34963}));
        primitive["indices"] = json!(1);
    }
    json!({"asset":{"version":"2.0"},"scenes":[{"nodes":[0]}],"nodes":[{"mesh":0}],
        "meshes":[{"primitives":[primitive]}],"accessors":accessors,"bufferViews":views,
        "buffers":[{"byteLength":bytes.len(),"uri":format!("data:application/octet-stream;base64,{}",BASE64.encode(bytes))}]})
}

#[test]
/// Checks exact position/index rows for implicit/u8/u16/u32 and packed/interleaved/sparse layouts.
/// Exercises JSON and GLB, instance counts and explicit projection qualification.
fn valid_unsigned_implicit_interleaved_and_sparse_accessors_keep_exact_rows() {
    for component in [None, Some(5121), Some(5123), Some(5125)] {
        for (interleaved, sparse) in [(false, false), (true, false), (false, true)] {
            let value = triangle_document(component, interleaved, sparse);
            for bytes in [serde_json::to_vec(&value).unwrap(), glb(value, true)] {
                let result = from_gltf(&bytes).unwrap();
                assert_eq!(
                    &*result.mesh.positions,
                    &[
                        Point3::origin(),
                        Point3::new(1.into(), 0.into(), 0.into()),
                        Point3::new(0.into(), 1.into(), 0.into())
                    ]
                );
                let expected = if component.is_some() {
                    [2, 0, 1]
                } else {
                    [0, 1, 2]
                };
                assert_eq!(result.mesh.triangles[0].indices(), expected);
                assert_eq!((result.mesh_node_count, result.primitive_count), (1, 1));
                assert!(result.mesh.construction_evidence().coordinates_projected());
            }
        }
    }
}

#[test]
/// Rejects malformed accessor shape/count/reference/alignment/range and checked size overflow.
/// A later valid import must still succeed after the failed attempts.
fn malformed_accessor_counts_types_references_and_ranges_return_errors() {
    let fixture = triangle_document(Some(5125), false, false);
    for (pointer, replacement) in [
        ("/meshes/0/primitives", json!([])),
        ("/meshes/0/primitives/0/attributes/POSITION", json!(999)),
        ("/accessors/0/count", json!(0)),
        ("/accessors/1/count", json!(0)),
        ("/accessors/1/componentType", json!(5126)),
        ("/accessors/1/type", json!("VEC2")),
        ("/accessors/0/type", json!("VEC2")),
        ("/bufferViews/1/byteLength", json!(1)),
        ("/bufferViews/1/byteOffset", json!(1_000_000)),
        ("/buffers/0/byteLength", json!(1)),
    ] {
        let mut value = fixture.clone();
        *value.pointer_mut(pointer).unwrap() = replacement;
        assert!(
            matches!(import(&value), Err(IoError::MalformedInput(_))),
            "{pointer}"
        );
    }
    for (accessor, field, value) in [
        (0, "normalized", json!(true)),
        (1, "normalized", json!(true)),
        (1, "byteOffset", json!(1_000_000)),
        (0, "byteOffset", json!(1)),
    ] {
        let mut document = fixture.clone();
        document["accessors"][accessor][field] = value;
        assert!(matches!(import(&document), Err(IoError::MalformedInput(_))));
    }
    let mut overflow = fixture.clone();
    overflow["bufferViews"][1]["byteOffset"] = json!(u64::MAX);
    assert!(matches!(
        import(&overflow),
        Err(IoError::SizeOverflow { format: "glTF", .. })
    ));
    assert!(
        import(&fixture).is_ok(),
        "invalid reads must not poison a later import"
    );
}

#[test]
/// Rejects invalid sparse counts/ranges and decreasing, duplicate or out-of-range indices.
fn sparse_counts_order_and_ranges_are_checked_before_reading() {
    let fixture = triangle_document(None, false, true);
    for (pointer, value) in [
        ("/accessors/0/sparse/count", json!(0)),
        ("/accessors/0/sparse/count", json!(4)),
        ("/bufferViews/0/byteLength", json!(1)),
        ("/bufferViews/1/byteLength", json!(12)),
    ] {
        let mut document = fixture.clone();
        *document.pointer_mut(pointer).unwrap() = value;
        assert!(matches!(import(&document), Err(IoError::MalformedInput(_))));
    }
    for indices in [[2, 1], [1, 1], [1, 3]] {
        let mut document = fixture.clone();
        let uri = document["buffers"][0]["uri"].as_str().unwrap();
        let mut bytes = BASE64.decode(uri.split_once(',').unwrap().1).unwrap();
        bytes[..2].copy_from_slice(&indices);
        document["buffers"][0]["uri"] = json!(format!(
            "data:application/octet-stream;base64,{}",
            BASE64.encode(bytes)
        ));
        assert!(matches!(import(&document), Err(IoError::MalformedInput(_))));
    }
}

#[test]
/// Distinguishes malformed graphs/empty arrays from valid empty geometry and missing-scene support.
/// Includes cycles, shared children, bad roots, non-affine transforms and truncated GLB.
fn malformed_graphs_and_present_empty_arrays_cannot_masquerade_as_empty() {
    for (nodes, roots) in [
        (json!([{"children":[0]}]), json!([0])),
        (json!([{"children":[1]},{"children":[0]}]), json!([0])),
        (json!([{"children":[2]},{"children":[2]},{}]), json!([0, 1])),
        (json!([{"children":[1,1]},{}]), json!([0])),
        (json!([{"children":[1]},{}]), json!([1])),
        (json!([{}]), json!([0, 0])),
        (json!([{"children":[9]}]), json!([0])),
    ] {
        let value =
            json!({"asset":{"version":"2.0"},"nodes":nodes,"scenes":[{"nodes":roots}]});
        assert!(matches!(import(&value), Err(IoError::MalformedInput(_))));
    }
    for field in [
        "scenes",
        "nodes",
        "meshes",
        "accessors",
        "buffers",
        "bufferViews",
    ] {
        let mut value = json!({"asset":{"version":"2.0"},"scenes":[{}]});
        value[field] = json!([]);
        assert!(matches!(import(&value), Err(IoError::MalformedInput(_))));
    }
    assert!(matches!(
        import(&json!({"asset":{"version":"2.0"},"scenes":[{"nodes":[]}]})),
        Err(IoError::MalformedInput(_))
    ));
    assert!(matches!(
        import(&json!({"asset":{"version":"2.0"}})),
        Err(IoError::Unsupported { format: "glTF", .. })
    ));
    let invalid_matrix = json!({"asset":{"version":"2.0"},"scenes":[{"nodes":[0]}],
        "nodes":[{"matrix":[1,0,0,1,0,1,0,0,0,0,1,0,0,0,0,1]}]});
    assert!(matches!(
        import(&invalid_matrix),
        Err(IoError::Unsupported { format: "glTF", .. })
    ));
    assert!(from_gltf(b"glTF").is_err());
}

#[test]
/// Exercises a full 10,000-node meshless chain through the iterative importer.
/// Checks completed canonical emptiness, without asserting a time or memory bound.
fn full_ten_thousand_node_chain_completes_without_recursive_traversal() {
    let count = 10_000;
    let nodes: Vec<_> = (0..count)
        .map(|index| {
            if index + 1 == count {
                json!({})
            } else {
                json!({"children":[index+1]})
            }
        })
        .collect();
    let document = json!({"asset":{"version":"2.0"},"scenes":[{"nodes":[0]}],"nodes":nodes});
    assert_empty(&import(&document).unwrap().mesh);
}

#[test]
/// Checks first/default scene selection and parent-scale-after-child-translation coordinates.
fn default_scene_and_nested_transform_order_remain_explicit() {
    let mut value = triangle_document(None, false, false);
    value["nodes"] = json!([
        {"children":[1],"scale":[2,3,4]}, {"mesh":0,"translation":[5,6,7]},
        {"name":"empty"}
    ]);
    value["scenes"] = json!([{"nodes":[2]},{"nodes":[0]},{"nodes":[0]}]);
    assert_empty(&import(&value).unwrap().mesh);
    value["scene"] = json!(2);
    let result = import(&value).unwrap();
    assert_eq!(result.scene_index, 2);
    assert_eq!(
        &*result.mesh.positions,
        &[[10, 18, 28], [12, 18, 28], [10, 21, 28]].map(|[x, y, z]| Point3::new(
            x.into(),
            y.into(),
            z.into()
        ))
    );
}

#[test]
/// Checks independent returned text, eviction, invalid source mutation and concurrent empty replay.
fn empty_cache_eviction_mutation_and_concurrent_exports_agree() {
    let empty = solid::empty();
    let expected = to_gltf(&empty, "empty").unwrap();
    let mut owned = to_gltf(&empty, "empty").unwrap();
    owned.clear();
    assert_eq!(to_gltf(&empty, "empty").unwrap(), expected);
    for index in 0..5 {
        to_gltf(&empty, &format!("evict-{index}")).unwrap();
    }
    assert_eq!(to_gltf(&empty, "empty").unwrap(), expected);
    let mut changed = empty.clone();
    changed.positions = vec![Point3::origin()].into();
    assert!(matches!(
        to_gltf(&changed, "empty"),
        Err(IoError::Geometry { format: "glTF", .. })
    ));
    let mut changed = empty.clone();
    changed.triangles = vec![Triangle::new(0, 1, 2)].into();
    assert!(matches!(
        to_gltf(&changed, "empty"),
        Err(IoError::Construction { format: "glTF", .. })
    ));
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                assert_eq!(to_gltf(&empty, "empty").unwrap(), expected);
                assert_empty(&from_gltf(expected.as_bytes()).unwrap().mesh);
            });
        }
    });
}

#[test]
/// Rejects duplicate empty names and malformed geometry before touching existing writer bytes.
/// Also checks successful empty output from the stream API.
fn invalid_geometry_and_duplicate_empty_names_leave_writers_untouched() {
    let empty = solid::empty();
    let duplicate = [
        GltfSceneObject::new("same", &empty).unwrap(),
        GltfSceneObject::new("same", &empty).unwrap(),
    ];
    let mut sink = b"unchanged".to_vec();
    assert!(matches!(
        write_gltf_scene(&mut sink, "scene", &duplicate),
        Err(IoError::Geometry { format: "glTF", .. })
    ));
    assert_eq!(sink, b"unchanged");
    for invalid in [
        TriangleMesh::new(vec![Point3::origin()], vec![]),
        TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]),
    ] {
        assert!(GltfSceneObject::new("bad", &invalid).is_err());
        assert!(write_gltf(&invalid, &mut sink, "bad").is_err());
        assert_eq!(sink, b"unchanged");
    }
    let mut output = Vec::new();
    write_gltf(&empty, &mut output, "empty").unwrap();
    assert_empty(&from_gltf(&output).unwrap().mesh);
}
