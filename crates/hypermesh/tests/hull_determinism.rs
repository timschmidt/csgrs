//! Exact set admission is independent of the fixed-input serialization check.
use hypermesh::{MeshCertainty, MeshContext, Point3, PredicatePolicy, Real, TriangleMesh};
use hyperreal::Rational;

fn r(n: i64) -> Real {
    Real::from(n)
}

fn rational_points(points: &[Point3], shift: &Real) -> Vec<[Rational; 3]> {
    points
        .iter()
        .map(|p| {
            [&p.x, &p.y, &p.z].map(|c| {
                (c - shift)
                    .exact_rational_normal_form()
                    .expect("rational source chart")
            })
        })
        .collect()
}

// Independent rational determinant, without hull predicates, caches or BVH.
fn determinant(
    a: &[Rational; 3],
    b: &[Rational; 3],
    c: &[Rational; 3],
    p: &[Rational; 3],
) -> Rational {
    let ab: [_; 3] = std::array::from_fn(|i| &b[i] - &a[i]);
    let ac: [_; 3] = std::array::from_fn(|i| &c[i] - &a[i]);
    let ap: [_; 3] = std::array::from_fn(|i| &p[i] - &a[i]);
    (&ab[1] * &ac[2] - &ab[2] * &ac[1]) * &ap[0]
        + (&ab[2] * &ac[0] - &ab[0] * &ac[2]) * &ap[1]
        + (&ab[0] * &ac[1] - &ab[1] * &ac[0]) * &ap[2]
}

fn assert_set(
    input: &[Point3],
    hull: &TriangleMesh,
    corners: &[Point3],
    shift: &Real,
    context: &MeshContext,
) {
    assert_eq!(
        hull.validate_closed_pwn(context).unwrap().certainty,
        MeshCertainty::Certified
    );
    assert!(
        hull.positions.iter().all(|p| input.contains(p)),
        "no rounded or invented hull coordinates"
    );
    assert!(
        corners.iter().all(|p| hull.positions.contains(p)),
        "every independently known extreme point remains"
    );
    let source = rational_points(input, shift);
    let output = rational_points(&hull.positions, shift);
    for face in hull.triangles.iter() {
        let [a, b, c] = face.indices().map(|i| &output[i]);
        assert!(
            source
                .iter()
                .all(|p| !determinant(a, b, c, p).is_positive()),
            "every source point lies in every outward halfspace"
        );
    }
}

fn cloud(sphere: bool, shift: &Real, permutation: usize) -> (Vec<Point3>, Vec<Point3>) {
    let mut points = Vec::new();
    if sphere {
        // All 30 points have norm one; their own tangent plane proves each
        // extreme independently of the hull algorithm.
        for zero in 0..3 {
            for swap in [false, true] {
                for a in [-3, 3] {
                    for b in [-4, 4] {
                        let mut p = [r(0), r(0), r(0)];
                        p[(zero + 1) % 3] = (r(if swap { b } else { a }) / r(5)).unwrap();
                        p[(zero + 2) % 3] = (r(if swap { a } else { b }) / r(5)).unwrap();
                        points.push(Point3::new(p[0].clone(), p[1].clone(), p[2].clone()));
                    }
                }
            }
        }
        for axis in 0..3 {
            for sign in [-1, 1] {
                let mut p = [r(0), r(0), r(0)];
                p[axis] = r(sign);
                points.push(Point3::new(p[0].clone(), p[1].clone(), p[2].clone()));
            }
        }
    } else {
        for x in [-1, 1] {
            for y in [-1, 1] {
                for z in [-1, 1] {
                    points.push(Point3::new(r(x), r(y), r(z)));
                }
            }
        }
    }
    for p in &mut points {
        p.x += shift;
        p.y += shift;
        p.z += shift;
    }
    let corners = points.clone();
    points.push(Point3::new(shift.clone(), shift.clone(), shift.clone()));
    if !sphere {
        for axis in 0..3 {
            for sign in [-1, 1] {
                let mut p = [shift.clone(), shift.clone(), shift.clone()];
                p[axis] += r(sign);
                points.push(Point3::new(p[0].clone(), p[1].clone(), p[2].clone()));
            }
        }
    }
    points.push(points[2].clone());
    match permutation {
        1 => points.reverse(),
        2 => points.rotate_left(3),
        _ => {}
    }
    (points, corners)
}

#[test]
fn cospherical_and_coplanar_hulls_replay_in_each_fixed_input_order() {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        for sphere in [false, true] {
            for shift in [r(0), r(2).sqrt().unwrap()] {
                for permutation in 0..3 {
                    let (input, corners) = cloud(sphere, &shift, permutation);
                    let first = hypermesh::convex_hull(&context, &input).unwrap();
                    assert_eq!(first.certainty, MeshCertainty::Certified);
                    assert_set(&input, &first.value, &corners, &shift, &context);
                    let ids = (0..input.len())
                        .map(|i| {
                            let n = (i as u64) * 5;
                            [n, n + 1, n + 2, n + 3, n + 4]
                        })
                        .collect::<Vec<_>>();
                    for next in [
                        hypermesh::convex_hull_with_coplanar_groups(&context, &input, &[]).unwrap(),
                        hypermesh::convex_hull_with_retained_facts(&context, &input, &[], &ids)
                            .unwrap(),
                    ] {
                        assert_eq!(next.certainty, first.certainty);
                        assert!(next.value == first.value, "public hull entry points differ");
                    }
                    for points in [&input, input.clone().as_slice()] {
                        for p in points {
                            for c in [&p.x, &p.y, &p.z] {
                                let _ = c.to_f64_lossy();
                            }
                        }
                        let next = hypermesh::convex_hull(&context, points).unwrap();
                        assert_eq!(next.certainty, first.certainty);
                        assert!(
                            next.value == first.value,
                            "fixed-input rows changed after warming/cloning"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn hull_output_continues_through_a_boolean_and_another_hull() {
    let context = MeshContext::new(PredicatePolicy::STRICT);
    let (input, corners) = cloud(true, &r(0), 0);
    let hull = hypermesh::convex_hull(&context, &input).unwrap().value;
    let (box_points, _) = cloud(false, &r(0), 0);
    let enclosing = hypermesh::convex_hull(&context, &box_points).unwrap().value;
    let result = hypermesh::boolean(
        &context,
        &[hull.as_ref(), enclosing.as_ref()],
        hypermesh::BooleanProgram::Operation(hypermesh::BooleanOp::Intersection),
    )
    .unwrap();
    assert_eq!(result.certainty, MeshCertainty::Certified);
    let intersection = result.value.into_triangle_meshes().unwrap().pop().unwrap();
    // Unit-sphere points lie in [-1,1]^3. The independent halfspace oracle
    // and every known extreme point prove the intended convex intersection.
    assert_set(&input, &intersection, &corners, &r(0), &context);
    let next = hypermesh::convex_hull(&context, &intersection.positions).unwrap();
    assert_eq!(next.certainty, MeshCertainty::Certified);
    assert_set(&input, &next.value, &corners, &r(0), &context);
}

#[test]
fn rounded_or_unavailable_scheduling_keys_do_not_merge_exact_vertices() {
    let context = MeshContext::new(PredicatePolicy::STRICT);
    for exponent in [-1100, -600, 60, 1100] {
        let scale = Real::from(2).powi(exponent.into()).unwrap();
        let input = vec![
            Point3::new(r(0), r(0), r(0)),
            Point3::new(scale.clone(), r(0), r(0)),
            Point3::new(r(0), scale.clone(), r(0)),
            Point3::new(r(0), r(0), scale.clone()),
            Point3::new(scale.clone(), scale.clone(), scale.clone()),
            Point3::new(scale.clone(), -scale, r(0)),
        ];
        let first = hypermesh::convex_hull(&context, &input).unwrap();
        assert_eq!(first.certainty, MeshCertainty::Certified);
        assert_set(&input, &first.value, &input, &r(0), &context);
        assert!(hypermesh::convex_hull(&context, &input).unwrap().value == first.value);
    }
}
