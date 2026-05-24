#![no_main]

use std::collections::BTreeSet;

use hypermesh::exact::{
    ExactMesh, ValidationPolicy, polygon_patch_candidate_face_sets_for_internal_fuzz,
    polygon_patch_pairs_for_internal_fuzz,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 4 {
        return;
    }

    let prefix = 1 + i64::from(data[0] % 31);
    let width = 2 + i64::from(data[1] % 31);
    let height = 2 + i64::from(data[2] % 31);
    exercise_subpatch_not_rooted_at_component_minimum(prefix, width, height);

    let oversized_faces = 33 + usize::from(data[3] % 16);
    exercise_oversized_component_remains_bounded(oversized_faces);
});

fn exercise_subpatch_not_rooted_at_component_minimum(prefix: i64, width: i64, height: i64) {
    let left = open_mesh(
        &[
            [0, 0, 0],
            [prefix, 0, 0],
            [prefix + width, 0, 0],
            [prefix + width, height, 0],
            [prefix, height, 0],
        ],
        &[
            0, 4, 1, //
            1, 4, 2, 2, 4, 3,
        ],
    );
    let right = open_mesh(
        &[
            [prefix, 0, 0],
            [prefix + width, 0, 0],
            [prefix + width, height, 0],
            [prefix, height, 0],
        ],
        &[0, 1, 2, 0, 2, 3],
    );

    let pairs =
        polygon_patch_pairs_for_internal_fuzz(&left, &BTreeSet::new(), &right, &BTreeSet::new())
            .expect("source-disk candidate discovery should complete");
    assert_eq!(pairs, vec![(vec![1, 2], vec![0, 1])]);
}

fn exercise_oversized_component_remains_bounded(face_count: usize) {
    let mesh = oversized_component_fan(face_count);
    let candidate_faces =
        polygon_patch_candidate_face_sets_for_internal_fuzz(&mesh, &BTreeSet::new())
            .expect("bounded source-disk candidate discovery should complete");

    assert!(candidate_faces.iter().all(|faces| faces.len() <= 9));
}

fn oversized_component_fan(face_count: usize) -> ExactMesh {
    let mut points = Vec::new();
    points.push([0, 0, 0]);
    for index in 0..=face_count {
        points.push([index as i64, 1, 0]);
    }
    let mut triangles = Vec::new();
    for index in 1..points.len() - 1 {
        triangles.extend([0, index, index + 1]);
    }
    open_mesh(&points, &triangles)
}

fn open_mesh(points: &[[i64; 3]], triangles: &[usize]) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(points.len() * 3);
    for point in points {
        coordinates.extend_from_slice(point);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        triangles,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("fuzz fixture should import as an exact open mesh")
}
