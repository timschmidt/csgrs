//! Search a fixed signed-zero corpus without changing the exact input points.
use csgrs::GeometryContext;
use hypermesh::{MeshCertainty, Point3, Real, convex_hull};

const POINTS: [[i8; 3]; 12] = [
    [0, -1, -1],
    [0, -1, 1],
    [0, 1, -1],
    [0, 1, 1],
    [-1, 0, -1],
    [-1, 0, 1],
    [1, 0, -1],
    [1, 0, 1],
    [-1, -1, 0],
    [-1, 1, 0],
    [1, -1, 0],
    [1, 1, 0],
];

fn negative_zero_count(points: &[Point3]) -> usize {
    points
        .iter()
        .flat_map(|point| [&point.x, &point.y, &point.z])
        .filter(|coordinate| {
            coordinate.to_f64_lossy().unwrap().to_bits() == (-0.0_f64).to_bits()
        })
        .count()
}

fn main() {
    let context = GeometryContext::STRICT.mesh_context();
    println!("exact_original_points={POINTS:?}");
    for mask in 0_u16..4096 {
        let points: Vec<_> = POINTS
            .iter()
            .enumerate()
            .map(|(index, coordinates)| {
                let [x, y, z] = coordinates.map(|value| {
                    let value = if value == 0 && mask & (1 << index) != 0 {
                        -0.0_f64
                    } else {
                        f64::from(value)
                    };
                    Real::try_from(value).unwrap()
                });
                Point3::new(x, y, z)
            })
            .collect();
        let before = negative_zero_count(&points);
        assert_eq!(before, mask.count_ones() as usize);
        let cold = convex_hull(&context, &points);
        let mut display_negative_zeros = 0;
        for point in &points {
            for coordinate in [&point.x, &point.y, &point.z] {
                let value = coordinate.to_f32_lossy().unwrap();
                display_negative_zeros += usize::from(value.to_bits() == (-0.0_f32).to_bits());
            }
        }
        let warm = convex_hull(&context, &points);
        let replay = convex_hull(&context, &points);
        let after = negative_zero_count(&points);
        let outcomes = [cold.unwrap(), warm.unwrap(), replay.unwrap()];
        for outcome in &outcomes {
            assert_eq!(outcome.certainty, MeshCertainty::Certified);
            assert_eq!(outcome.value.positions.len(), 12);
            assert_eq!(outcome.value.triangles.len(), 20);
            assert!(outcome.value.is_closed_manifold());
        }
        let changed = outcomes[1..].iter().any(|outcome| {
            outcome.value.positions != outcomes[0].value.positions
                || outcome.value.triangles != outcomes[0].value.triangles
        });
        if changed {
            println!(
                "counterexample mask={mask} masks_checked={} negative_zero_before={before} display_f32_negative_zeros={display_negative_zeros} negative_zero_after={after}",
                u32::from(mask) + 1
            );
            for (phase, outcome) in ["cold", "warm", "replay"].into_iter().zip(&outcomes) {
                let order: Vec<_> = outcome
                    .value
                    .positions
                    .iter()
                    .map(|point| points.iter().position(|source| source == point).unwrap())
                    .collect();
                println!("phase={phase} position_order={order:?}");
                println!("triangle_rows={:?}", outcome.value.triangles);
            }
            return;
        }
        if mask % 256 == 255 {
            println!("progress masks_checked={}", u32::from(mask) + 1);
        }
    }
    println!("no_counterexample masks_checked=4096");
}
