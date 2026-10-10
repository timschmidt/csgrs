use super::f64_box_certainly_misses_forward_ray;

#[test]
fn finite_ray_filter_keeps_independently_known_intersections() {
    for scale in [1e-200_f64, 1.0, 1e150, 1e154, 1e155, 1e200, 1e300] {
        for sx in [-1.0_f64, 1.0] {
            for sy in [-1.0_f64, 1.0] {
                let x = sx * scale;
                let y = sy * scale;
                let bounds = [
                    x.min(2.0 * x),
                    x.max(2.0 * x),
                    y.min(2.0 * y),
                    y.max(2.0 * y),
                ];
                assert!(bounds.iter().all(|value| value.is_finite()));
                // At parameter one the ray equals a stored box corner.
                assert!(!f64_box_certainly_misses_forward_ray(
                    bounds,
                    [[0.0; 2]; 2],
                    [[x; 2], [y; 2]],
                ));
            }
        }
    }
    for sx in [-1.0_f64, 1.0] {
        for sy in [-1.0_f64, 1.0] {
            let x = sx * 1e308;
            let y = sy * 1e308;
            let bounds = [
                (x * 0.9).min(x * 1.1),
                (x * 0.9).max(x * 1.1),
                (y * 0.9).min(y * 1.1),
                (y * 0.9).max(y * 1.1),
            ];
            assert!(bounds.iter().all(|value| value.is_finite()));
            // The origin is strictly inside both intervals. Finite enclosure
            // endpoints can nevertheless overflow during midpoint formation.
            assert!(!f64_box_certainly_misses_forward_ray(
                bounds,
                [[x; 2], [y; 2]],
                [[1.0; 2], [2.0; 2]],
            ));
        }
    }
}

#[test]
fn finite_ray_filter_preserves_certified_side_and_behind_rejections() {
    for bounds in [[-2.0, -1.0, 1.0, 2.0], [-2.0, -1.0, -2.0, -1.0]] {
        assert!(f64_box_certainly_misses_forward_ray(
            bounds,
            [[0.0; 2]; 2],
            [[1.0; 2]; 2],
        ));
    }
}
