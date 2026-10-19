// Independent exact set oracle; no mesh Boolean or containment calls.
fn q(n: i64) -> Q {
    Q::new(n)
}
fn sub(a: &[Q; 3], b: &[Q; 3]) -> [Q; 3] {
    std::array::from_fn(|i| &a[i] - &b[i])
}
fn cross(a: &[Q; 3], b: &[Q; 3]) -> [Q; 3] {
    [
        &a[1] * &b[2] - &a[2] * &b[1],
        &a[2] * &b[0] - &a[0] * &b[2],
        &a[0] * &b[1] - &a[1] * &b[0],
    ]
}
fn dot(a: &[Q; 3], b: &[Q; 3]) -> Q {
    (0..3).fold(q(0), |s, i| s + &a[i] * &b[i])
}
fn orient(a: &[Q; 3], b: &[Q; 3], p: &[Q; 3]) -> Q {
    (&b[1] - &a[1]) * (&p[2] - &a[2]) - (&b[2] - &a[2]) * (&p[1] - &a[1])
}

#[derive(Default, Debug)]
struct Counts {
    outputs: usize,
    cells: usize,
    ray_trials: usize,
}

// All triangles must stay on the source coordinate planes in the inverse
// affine chart. The open grid cells therefore contain no output boundary.
// Exact signed ray crossings at one nondegenerate point per cell establish
// its winding, and hence the entire regularized set. This oracle uses rational
// arithmetic directly; it never calls Hypermesh containment or Boolean code.
fn audit(
    mesh: &TriangleMesh,
    frame: &Frame,
    a: &Box3,
    b: &Box3,
    expected: fn(bool, bool) -> bool,
) -> Result<Counts, String> {
    let planes: [Vec<Q>; 3] = std::array::from_fn(|i| {
        let mut v = vec![
            a.lo[i].clone(),
            a.hi[i].clone(),
            b.lo[i].clone(),
            b.hi[i].clone(),
        ];
        v.sort_by(|x, y| x.partial_cmp(y).unwrap());
        v.dedup();
        v
    });
    let grid: [Vec<Q>; 3] = std::array::from_fn(|i| {
        let mut v = planes[i].clone();
        v.insert(0, &v[0] - q(1));
        v.push(v.last().unwrap() + q(1));
        v
    });
    let points: Vec<_> = mesh
        .positions
        .iter()
        .map(|p| frame.chart(p))
        .collect::<Result<_, _>>()?;
    for p in &points {
        if (0..3).any(|i| p[i] < planes[i][0] || p[i] > *planes[i].last().unwrap()) {
            return Err("incorrect-result: output extends beyond source bounds".into());
        }
    }
    let mut triangles = Vec::new();
    let mut six_volume = q(0);
    for triangle in mesh.triangles.iter() {
        let mut ids = triangle.indices();
        if ids.iter().any(|&i| i >= points.len()) {
            return Err("incorrect-result: invalid output index".into());
        }
        if frame.name == "reflection" {
            ids.swap(0, 2);
        }
        let t = ids.map(|i| points[i].clone());
        if !(0..3)
            .any(|i| t[0][i] == t[1][i] && t[0][i] == t[2][i] && planes[i].contains(&t[0][i]))
        {
            return Err("incorrect-result: output facet is not on a source plane".into());
        }
        if cross(&sub(&t[1], &t[0]), &sub(&t[2], &t[0])) == [q(0), q(0), q(0)] {
            return Err("incorrect-result: degenerate output triangle".into());
        }
        six_volume = six_volume + dot(&t[0], &cross(&t[1], &t[2]));
        triangles.push(t);
    }
    let mut count = Counts {
        outputs: 1,
        ..Counts::default()
    };
    let mut volume = q(0);
    for xs in grid[0].windows(2) {
        for ys in grid[1].windows(2) {
            for zs in grid[2].windows(2) {
                let size = [&xs[1] - &xs[0], &ys[1] - &ys[0], &zs[1] - &zs[0]];
                let mut sampled = None;
                // This rational curve intersects any projected edge line at most
                // twice. 6*T+1 samples suffice for the at most 3*T edge lines.
                for k in 0..=6 * triangles.len() {
                    count.ray_trials += 1;
                    let p = [
                        (&xs[0] + &xs[1]) / q(2),
                        &ys[0] + &size[1] / q(k as i64 + 2),
                        &zs[0] + &size[2] / q(k as i64 + 3),
                    ];
                    let mut winding = 0_i64;
                    let mut ambiguous = false;
                    for t in &triangles {
                        let area = orient(&t[0], &t[1], &t[2]);
                        if area == q(0) || t[0][0] <= p[0] {
                            continue;
                        }
                        let side = [
                            orient(&t[0], &t[1], &p),
                            orient(&t[1], &t[2], &p),
                            orient(&t[2], &t[0], &p),
                        ];
                        if side.contains(&q(0)) {
                            ambiguous = true;
                            break;
                        }
                        if side.iter().all(|s| (*s > q(0)) == (area > q(0))) {
                            winding += if area > q(0) { 1 } else { -1 };
                        }
                    }
                    if !ambiguous {
                        sampled = Some((p, winding));
                        break;
                    }
                }
                let (p, winding) =
                    sampled.ok_or("oracle-error: nondegenerate ray bound exhausted")?;
                let inside = expected(a.contains(&p), b.contains(&p));
                if (winding != 0) != inside {
                    return Err(format!(
                        "incorrect-result: cell winding {winding}, expected {inside}, point {p:?}"
                    ));
                }
                count.cells += 1;
                if inside {
                    volume = volume + &size[0] * &size[1] * &size[2];
                }
            }
        }
    }
    if six_volume != q(6) * &volume {
        return Err(
            "incorrect-result: oriented volume differs from independent occupied-cell volume"
                .into(),
        );
    }
    if volume == q(0) && (!mesh.positions.is_empty() || !mesh.triangles.is_empty()) {
        return Err("incorrect-result: empty set is not canonical empty geometry".into());
    }
    Ok(count)
}
