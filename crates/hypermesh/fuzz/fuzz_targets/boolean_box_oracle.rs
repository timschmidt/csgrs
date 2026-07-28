#![no_main]

mod support;

use hypermesh::{
    BooleanOp, EmberConfig, boolean_operation, boolean_triangle_soup,
    boolean_triangle_soup_with_certified_convex_inputs,
};
use libfuzzer_sys::fuzz_target;
use support::{Bytes, box_mesh, operation, r, validate_result, validate_soup, volume_numerator};

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

    let soup = match api {
        0 => {
            let result = boolean_operation(&refs, op, EmberConfig::default())
                .unwrap_or_else(|error| panic!("integer-box Boolean failed: {error:?}"));
            validate_result(&result, op, refs.len())
        }
        1 => {
            let soup = boolean_triangle_soup(&refs, op, EmberConfig::default())
                .unwrap_or_else(|error| panic!("integer-box immediate Boolean failed: {error:?}"));
            validate_soup(&soup);
            soup
        }
        _ => {
            let certified = vec![true; refs.len()];
            let soup = boolean_triangle_soup_with_certified_convex_inputs(
                &refs,
                op,
                &certified,
                EmberConfig::default(),
            )
            .unwrap_or_else(|error| {
                panic!("certified integer-box immediate Boolean failed: {error:?}")
            });
            validate_soup(&soup);
            soup
        }
    };

    assert_eq!(
        volume_numerator(&soup),
        r(6 * oracle_volume(&bounds, op)),
        "exact Boolean volume disagrees with the box-cell oracle",
    );
});
