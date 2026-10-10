fn f64_box_certainly_misses_forward_ray(
    bounds: [f64; 4],
    origin: [[f64; 2]; 2],
    direction: [[f64; 2]; 2],
) -> bool {
    const MARGIN: f64 = 1.0e-9;
    // Midpoints and radii of the certified origin and direction intervals.
    let mid_radius = |[low, high]: [f64; 2]| ((low + high) * 0.5, (high - low) * 0.5);
    let (origin_x, origin_x_radius) = mid_radius(origin[0]);
    let (origin_y, origin_y_radius) = mid_radius(origin[1]);
    let (dx, dx_radius) = mid_radius(direction[0]);
    let (dy, dy_radius) = mid_radius(direction[1]);
    // |ab - a'b'| <= |a'| r_b + |b'| r_a + r_a r_b for a in a' +- r_a.
    let product_error = |a: f64, a_radius: f64, b: f64, b_radius: f64| {
        a.abs() * b_radius + b.abs() * a_radius + a_radius * b_radius
    };
    let mut side_signs = [true, true];
    let mut all_behind = true;
    for (x, y) in [
        (bounds[0], bounds[2]),
        (bounds[0], bounds[3]),
        (bounds[1], bounds[2]),
        (bounds[1], bounds[3]),
    ] {
        let delta_x = x - origin_x;
        let delta_y = y - origin_y;
        let scale = (x.abs() + origin_x.abs()) * (dx.abs() + dy.abs())
            + (y.abs() + origin_y.abs()) * (dx.abs() + dy.abs());
        let input_error = product_error(dy, dy_radius, delta_x, origin_x_radius)
            + product_error(dx, dx_radius, delta_y, origin_y_radius)
            + product_error(dx, dx_radius, delta_x, origin_x_radius)
            + product_error(dy, dy_radius, delta_y, origin_y_radius);
        let error = MARGIN * (scale + input_error) + input_error + 1.0e-300;
        let side = -dy * delta_x + dx * delta_y;
        let forward = dx * delta_x + dy * delta_y;
        if side >= -error {
            side_signs[0] = false;
        }
        if side <= error {
            side_signs[1] = false;
        }
        if forward >= -error {
            all_behind = false;
        }
    }
    side_signs[0] || side_signs[1] || all_behind
}

fn main() {
    let mut failures = 0;
    let mut cases = 0;
    let mut check = |name: String, bounds: [f64; 4], origin: [[f64; 2]; 2], direction: [[f64; 2]; 2], expected_miss: bool, witness: &str| {
        assert!(bounds.into_iter().chain(origin.into_iter().flatten()).chain(direction.into_iter().flatten()).all(f64::is_finite));
        let miss = f64_box_certainly_misses_forward_ray(bounds, origin, direction);
        cases += 1;
        failures += usize::from(miss != expected_miss);
        println!("{{\"case\":\"{name}\",\"bounds\":{bounds:?},\"origin\":{origin:?},\"direction\":{direction:?},\"expected_miss\":{expected_miss},\"actual_miss\":{miss},\"witness\":\"{witness}\",\"all_inputs_finite\":true}}");
    };
    for scale in [1e-200_f64, 1.0, 1e150, 1e154, 1e155, 1e200, 1e300] {
        for sx in [-1.0_f64, 1.0] {
            for sy in [-1.0_f64, 1.0] {
                let x = sx * scale;
                let y = sy * scale;
                check(format!("ray_hit_{scale:e}_{sx}_{sy}"), [x.min(2.0*x), x.max(2.0*x), y.min(2.0*y), y.max(2.0*y)], [[0.0; 2]; 2], [[x; 2], [y; 2]], false, "At exact parameter one the ray equals a stored box corner.");
            }
        }
    }
    for sx in [-1.0_f64, 1.0] {
        for sy in [-1.0_f64, 1.0] {
            let x = sx * 1e308;
            let y = sy * 1e308;
            check(format!("inside_large_translation_{sx}_{sy}"), [(x*0.9).min(x*1.1), (x*0.9).max(x*1.1), (y*0.9).min(y*1.1), (y*0.9).max(y*1.1)], [[x; 2], [y; 2]], [[1.0; 2], [2.0; 2]], false, "The ray origin lies strictly inside both stored box intervals.");
        }
    }
    check("ordinary_side_miss".into(), [-2.0,-1.0,1.0,2.0], [[0.0; 2]; 2], [[1.0; 2]; 2], true, "Opposite-sign coordinates put every corner strictly left of y=x.");
    check("ordinary_behind_miss".into(), [-2.0,-1.0,-2.0,-1.0], [[0.0; 2]; 2], [[1.0; 2]; 2], true, "Every corner has a negative dot product with (1,1).");
    assert_eq!(failures, 0, "{failures} of {cases} exact box/ray witnesses were contradicted");
}
