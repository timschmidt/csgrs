#![no_main]

mod support;

use hypermesh::{
    BooleanExpression, BooleanOp, BooleanProgram, TriangleMeshRef, boolean,
};
use libfuzzer_sys::fuzz_target;
use support::{
    Bytes, CONTEXT, box_mesh, operation, r, validate_batch, value, volume_numerator,
};

fn oracle_volume(boxes: &[([i64; 3], [i64; 3])], operation: BooleanOp) -> i64 {
    let mut axes = [Vec::new(), Vec::new(), Vec::new()];
    for (min, max) in boxes {
        for axis in 0..3 {
            axes[axis].extend([min[axis], max[axis]]);
        }
    }
    for coordinates in &mut axes {
        coordinates.sort_unstable();
        coordinates.dedup();
    }

    let mut volume = 0;
    for x in axes[0].windows(2) {
        for y in axes[1].windows(2) {
            for z in axes[2].windows(2) {
                let doubled_sample = [x[0] + x[1], y[0] + y[1], z[0] + z[1]];
                let winding = boxes
                    .iter()
                    .map(|(min, max)| {
                        i32::from((0..3).all(|axis| {
                            2 * min[axis] < doubled_sample[axis]
                                && doubled_sample[axis] < 2 * max[axis]
                        }))
                    })
                    .collect::<Vec<_>>();
                if operation.contains(&winding) {
                    volume += (x[1] - x[0]) * (y[1] - y[0]) * (z[1] - z[0]);
                }
            }
        }
    }
    volume
}

fuzz_target!(|data: [u8; 32]| {
    let mut bytes = Bytes::new(&data);
    let mesh_count = 1 + usize::from(bytes.next() % 4);
    let op = operation(bytes.next());
    let api = bytes.next() % 3;
    let mut bounds = Vec::with_capacity(mesh_count);
    for _ in 0..mesh_count {
        let min = [
            bytes.bounded_i64(6),
            bytes.bounded_i64(6),
            bytes.bounded_i64(6),
        ];
        let max = [
            min[0] + bytes.positive_i64(5),
            min[1] + bytes.positive_i64(5),
            min[2] + bytes.positive_i64(5),
        ];
        bounds.push((min, max));
    }
    let meshes = bounds
        .iter()
        .map(|(min, max)| box_mesh(*min, *max))
        .collect::<Vec<_>>();
    let refs = meshes.iter().map(|mesh| mesh.as_ref()).collect::<Vec<_>>();
    let raw_refs = meshes
        .iter()
        .map(|mesh| TriangleMeshRef::new(&mesh.positions, &mesh.triangles))
        .collect::<Vec<_>>();

    let all_nodes = [
        BooleanExpression::Operation(BooleanOp::Union),
        BooleanExpression::Operation(BooleanOp::Intersection),
        BooleanExpression::Operation(BooleanOp::Difference),
        BooleanExpression::Operation(BooleanOp::SymmetricDifference),
    ];
    let all_roots = [0, 1, 2, 3];
    let (views, program, output) = match api {
        0 => (
            raw_refs.as_slice(),
            BooleanProgram::Operation(op),
            0,
        ),
        1 => (refs.as_slice(), BooleanProgram::Operation(op), 0),
        _ => (
            refs.as_slice(),
            BooleanProgram::Expressions {
                nodes: &all_nodes,
                roots: &all_roots,
            },
            match op {
                BooleanOp::Union => 0,
                BooleanOp::Intersection => 1,
                BooleanOp::Difference => 2,
                BooleanOp::SymmetricDifference => 3,
            },
        ),
    };
    let batch = value(boolean(&CONTEXT, views, program))
        .unwrap_or_else(|error| panic!("integer-box Boolean failed: {error:?}"));
    validate_batch(&batch);

    assert_eq!(
        volume_numerator(&batch.vertices, &batch.results[output]),
        r(6 * oracle_volume(&bounds, op)),
        "exact Boolean volume disagrees with the box-cell oracle",
    );
});
