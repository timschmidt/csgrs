//! Fixed-frame curve sweeps whose plane progress is monotone but not affine.
//!
//! Expected sets come from closed forms that do not use the sweep kernel: a
//! straight prism, or the inverse of the progress polynomial together with the
//! lateral path offset at that parameter.

use hyperbrep::*;
use std::cmp::Ordering;

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn bezier(controls: &[(Real, Real)]) -> Curve3 {
    // Controls are (x, z); every path stays in the plane y = 0.
    Curve3::rational_bezier(
        controls
            .iter()
            .map(|(x, z)| point(x.clone(), Real::zero(), z.clone()))
            .collect(),
        vec![Real::one(); controls.len()],
    )
    .unwrap()
}

fn vertical(levels: &[Real]) -> Curve3 {
    bezier(
        &levels
            .iter()
            .map(|level| (Real::zero(), level.clone()))
            .collect::<Vec<_>>(),
    )
}

fn polygon(points: &[(i64, i64)]) -> Vec<Point2> {
    points
        .iter()
        .map(|(x, y)| Point2::new((*x).into(), (*y).into()))
        .collect()
}

fn order(left: &Real, right: &Real) -> Ordering {
    hyperlimit::compare_reals(left, right, hyperlimit::PredicatePolicy::STRICT)
        .value()
        .unwrap()
}

fn assert_exact(actual: &Real, expected: &Real) {
    assert_eq!(order(actual, expected), Ordering::Equal);
}

fn body(set: &BodySet) -> SolidRef<'_> {
    assert_eq!(set.len(), 1);
    set.bodies().next().unwrap()
}

fn volume(set: &BodySet) -> Real {
    set.bodies()
        .fold(Real::zero(), |sum, solid| sum + solid.volume().unwrap())
}

fn replay(set: &BodySet) -> Model {
    let json = set.model().to_json().unwrap();
    let model = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), json);
    model
}

/// Location in the open interval `(0, width)` of a coordinate.
fn interval_location(value: &Real, width: &Real) -> SolidPointLocation {
    match (order(value, &Real::zero()), order(value, width)) {
        (Ordering::Less, _) | (_, Ordering::Greater) => SolidPointLocation::Outside,
        (Ordering::Equal, _) | (_, Ordering::Equal) => SolidPointLocation::Boundary,
        _ => SolidPointLocation::Inside,
    }
}

fn combine(locations: [SolidPointLocation; 3]) -> SolidPointLocation {
    if locations.contains(&SolidPointLocation::Outside) {
        SolidPointLocation::Outside
    } else if locations.contains(&SolidPointLocation::Boundary) {
        SolidPointLocation::Boundary
    } else {
        SolidPointLocation::Inside
    }
}

/// The triangle `x, y > 0, x + y < 2` swept straight through `0 < z < 1`.
fn triangle_prism_location(sample: &Point3) -> SolidPointLocation {
    let diagonal = Real::from(2) - &sample.x - &sample.y;
    let triangle = combine([
        interval_location(&sample.x, &Real::from(4)),
        interval_location(&sample.y, &Real::from(4)),
        interval_location(&diagonal, &Real::from(4)),
    ]);
    combine([
        triangle,
        interval_location(&sample.z, &Real::one()),
        SolidPointLocation::Inside,
    ])
}

/// Unit square sections at the lateral `offset` of a path in the plane y = 0.
fn shifted_square_location(sample: &Point3, offset: &Real) -> SolidPointLocation {
    combine([
        interval_location(&(&sample.x - offset), &Real::one()),
        interval_location(&sample.y, &Real::one()),
        interval_location(&sample.z, &Real::one()),
    ])
}

/// x = t², z = (t + t²)/2: the parameter of level z is (√(1 + 8z) - 1)/2.
fn quadratic_lateral_offset(level: &Real) -> Real {
    let root =
        ((Real::one() + Real::from(8) * level).sqrt().unwrap() - Real::one()) / Real::from(2);
    let root = root.unwrap();
    &root * &root
}

fn quadratic_lateral_path() -> Curve3 {
    bezier(&[
        (Real::zero(), Real::zero()),
        (Real::zero(), q(1, 4)),
        (Real::one(), Real::one()),
    ])
}

fn quadratic_lateral_samples() -> Vec<Point3> {
    let half = q(1, 2);
    let irrational_offset = quadratic_lateral_offset(&half);
    let mut samples = Vec::new();
    for x in [q(3, 4), q(1, 4), q(1, 8), q(5, 4), q(51, 40)] {
        samples.push(point(x, half.clone(), q(3, 8)));
    }
    // At z = 1/2 the section starts at (3 - √5)/2, an irrational offset.
    for x in [
        q(38, 100),
        q(39, 100),
        irrational_offset.clone(),
        irrational_offset.clone() + Real::one(),
        q(138, 100),
        q(139, 100),
    ] {
        samples.push(point(x, half.clone(), half.clone()));
    }
    samples.push(point(Real::zero(), half.clone(), Real::zero()));
    samples.push(point(q(1, 2), half.clone(), Real::zero()));
    samples.push(point(q(3, 2), half.clone(), Real::one()));
    samples.push(point(q(3, 2), half.clone(), q(101, 100)));
    samples.push(point(q(1, 2), half.clone(), -q(1, 100)));
    samples
}

fn quadratic_lateral_expected(sample: &Point3) -> SolidPointLocation {
    let level = &sample.z;
    if order(level, &Real::zero()) == Ordering::Less
        || order(level, &Real::one()) == Ordering::Greater
    {
        return SolidPointLocation::Outside;
    }
    shifted_square_location(sample, &quadratic_lateral_offset(level))
}

#[test]
fn equivalent_progress_parameterizations_sweep_one_prism() {
    let profile = polygon(&[(0, 0), (2, 0), (0, 2)]);
    let affine = vertical(&[Real::zero(), Real::one()]);
    let paths = [
        affine.clone(),
        // z = t written with an elevated degree.
        vertical(&[Real::zero(), q(1, 2), Real::one()]),
        // z = (t + t²)/2 and its exact degree elevation.
        vertical(&[Real::zero(), q(1, 4), Real::one()]),
        vertical(&[Real::zero(), q(1, 6), q(1, 2), Real::one()]),
        // z = (2t - t²)/(1 + 2t - 2t²), a rational monotone progress.
        Curve3::rational_bezier(
            vec![
                point(Real::zero(), Real::zero(), Real::zero()),
                point(Real::zero(), Real::zero(), q(1, 2)),
                point(Real::zero(), Real::zero(), Real::one()),
            ],
            vec![Real::one(), Real::from(2), Real::one()],
        )
        .unwrap(),
    ];
    let reference = builder::sweep_curve(&profile, Vector3::x(), Vector3::y(), affine).unwrap();
    let reference_bounds = reference.model().bounds().unwrap().unwrap();
    let mut samples = Vec::new();
    for z in [
        -q(1, 10),
        Real::zero(),
        q(1, 3),
        q(1, 2),
        Real::one(),
        q(11, 10),
    ] {
        for (x, y) in [
            (q(1, 2), q(1, 2)),
            (Real::zero(), q(1, 2)),
            (Real::one(), Real::one()),
            (q(3, 2), q(1, 4)),
            (q(3, 2), q(1, 2)),
            (-q(1, 10), q(1, 2)),
        ] {
            samples.push(point(x, y, z.clone()));
        }
    }
    for path in paths {
        let swept = builder::sweep_curve(&profile, Vector3::x(), Vector3::y(), path).unwrap();
        let solid = body(&swept);
        assert_exact(&solid.volume().unwrap(), &Real::from(2));
        let bounds = swept.model().bounds().unwrap().unwrap();
        for (actual, expected) in [
            (&bounds.mins.x, &reference_bounds.mins.x),
            (&bounds.mins.y, &reference_bounds.mins.y),
            (&bounds.mins.z, &reference_bounds.mins.z),
            (&bounds.maxs.x, &reference_bounds.maxs.x),
            (&bounds.maxs.y, &reference_bounds.maxs.y),
            (&bounds.maxs.z, &reference_bounds.maxs.z),
        ] {
            assert_exact(actual, expected);
        }
        let replayed = replay(&swept);
        let replayed_solid = replayed.solids().next().unwrap();
        assert_exact(&replayed_solid.volume().unwrap(), &Real::from(2));
        for sample in &samples {
            let expected = triangle_prism_location(sample);
            assert_eq!(
                solid.classify_point(sample).unwrap(),
                expected,
                "{sample:?}"
            );
            assert_eq!(replayed_solid.classify_point(sample).unwrap(), expected);
            assert_eq!(body(&reference).classify_point(sample).unwrap(), expected);
        }
    }
}

#[test]
fn monotone_progress_keeps_lateral_sections() {
    let square = polygon(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
    let elevated = bezier(&[
        (Real::zero(), Real::zero()),
        (Real::zero(), q(1, 6)),
        (q(1, 3), q(1, 2)),
        (Real::one(), Real::one()),
    ]);
    for path in [quadratic_lateral_path(), elevated] {
        let swept = builder::sweep_curve(&square, Vector3::x(), Vector3::y(), path).unwrap();
        let solid = body(&swept);
        assert_exact(&solid.volume().unwrap(), &Real::one());
        let replayed = replay(&swept);
        let replayed_solid = replayed.solids().next().unwrap();
        for sample in quadratic_lateral_samples() {
            let expected = quadratic_lateral_expected(&sample);
            assert_eq!(
                solid.classify_point(&sample).unwrap(),
                expected,
                "{sample:?}"
            );
            assert_eq!(replayed_solid.classify_point(&sample).unwrap(), expected);
        }
    }
}

#[test]
fn selected_progress_roots_classify_sections() {
    // x = t², z = (t + t³)/2. At z = 1/2 the parameter is the real root of
    // t³ + t - 1, so the section offset t² has no stored scalar.
    let path = bezier(&[
        (Real::zero(), Real::zero()),
        (Real::zero(), q(1, 6)),
        (q(1, 3), q(1, 3)),
        (Real::one(), Real::one()),
    ]);
    let square = polygon(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
    let swept = builder::sweep_curve(&square, Vector3::x(), Vector3::y(), path).unwrap();
    let replayed = replay(&swept);
    let level = q(1, 2);
    // For 0 < s, s < t* exactly when (s + s³)/2 < 1/2, so x < t*² exactly
    // when g(√x) < 1/2. This oracle never forms t*.
    let below_offset = |x: &Real| {
        let root = x.clone().sqrt().unwrap();
        let progress = ((&root + &root * &root * &root) / Real::from(2)).unwrap();
        order(&progress, &level) == Ordering::Less
    };
    assert_exact(&body(&swept).volume().unwrap(), &Real::one());
    for x in [
        q(46, 100),
        q(47, 100),
        q(146, 100),
        q(147, 100),
        q(1, 10),
        q(19, 10),
    ] {
        let beyond_far_side =
            order(&x, &Real::one()) == Ordering::Greater && !below_offset(&(&x - Real::one()));
        let expected = if below_offset(&x) || beyond_far_side {
            SolidPointLocation::Outside
        } else {
            SolidPointLocation::Inside
        };
        let sample = point(x, q(1, 2), level.clone());
        assert_eq!(
            body(&swept).classify_point(&sample).unwrap(),
            expected,
            "{sample:?}"
        );
        assert_eq!(
            replayed
                .solids()
                .next()
                .unwrap()
                .classify_point(&sample)
                .unwrap(),
            expected
        );
    }
    // Profile edges parallel to the path's lateral motion still decide.
    let boundary = point(q(1, 2), Real::zero(), level.clone());
    assert_eq!(
        body(&swept).classify_point(&boundary).unwrap(),
        SolidPointLocation::Boundary
    );
}

#[test]
fn bernstein_inconclusive_progress_is_decided_by_sturm() {
    let square = polygon(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
    // z = (t - 1/2)³ + 3t/100 + 1/8: z' = 3(t - 1/2)² + 3/100 > 0, but the
    // control polygon dips and the degree-5 derivative form has a negative
    // interior coefficient, so only the exact root count proves it.
    let dipping = vertical(&[Real::zero(), q(13, 50), q(1, 50), q(7, 25)]);
    let swept = builder::sweep_curve(&square, Vector3::x(), Vector3::y(), dipping).unwrap();
    let solid = body(&swept);
    assert_exact(&solid.volume().unwrap(), &q(7, 25));
    for (sample, expected) in [
        (
            point(q(1, 2), q(1, 2), q(7, 50)),
            SolidPointLocation::Inside,
        ),
        (
            point(q(1, 2), q(1, 2), q(7, 25)),
            SolidPointLocation::Boundary,
        ),
        (
            point(q(1, 2), q(1, 2), q(3, 10)),
            SolidPointLocation::Outside,
        ),
        (
            point(Real::one(), q(1, 2), q(1, 10)),
            SolidPointLocation::Boundary,
        ),
    ] {
        assert_eq!(solid.classify_point(&sample).unwrap(), expected);
    }

    for (levels, reason) in [
        // z' = 3((t - 1/2)² - 1/100) is negative near t = 1/2 although the
        // endpoints rise: the sweep would retrace its own sections.
        (
            vec![Real::zero(), q(6, 25), -q(1, 50), q(11, 50)],
            "interior backtracking",
        ),
        // z = 2t - 3t²/2 descends after t = 2/3.
        (vec![Real::zero(), Real::one(), q(1, 2)], "backtracking end"),
        // z = t² and z = (t - 1/2)³ + 1/8 rise strictly but stop at one
        // parameter; this certificate requires a nonvanishing rate.
        (
            vec![Real::zero(), Real::zero(), Real::one()],
            "stationary start",
        ),
        (
            vec![Real::zero(), q(1, 4), Real::zero(), q(1, 4)],
            "stationary interior",
        ),
    ] {
        assert_eq!(
            builder::sweep_curve(&square, Vector3::x(), Vector3::y(), vertical(&levels))
                .unwrap_err(),
            ConstructionError::NonMonotoneSweepPath,
            "{reason}"
        );
    }
}

#[test]
fn monotone_progress_requires_a_fixed_frame() {
    let square = polygon(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
    let origins = vec![
        point(Real::zero(), Real::zero(), Real::zero()),
        point(Real::zero(), Real::zero(), q(1, 4)),
        point(Real::zero(), Real::zero(), Real::one()),
    ];
    let fixed = RationalBezierSweepFrame::try_new(
        origins.clone(),
        vec![Vector3::x(); 3],
        vec![Vector3::y(); 3],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert_exact(
        &volume(&builder::sweep_moving_frame(&square, fixed).unwrap()),
        &Real::one(),
    );
    let tapered = RationalBezierSweepFrame::try_new(
        origins,
        vec![
            Vector3::x(),
            Vector3::x(),
            Vector3::from_xyz(Real::from(2), Real::zero(), Real::zero()),
        ],
        vec![Vector3::y(); 3],
        vec![Real::one(); 3],
    )
    .unwrap();
    assert_eq!(
        builder::sweep_moving_frame(&square, tapered).unwrap_err(),
        ConstructionError::NonMonotoneSweepPath
    );
}

#[test]
fn monotone_region_sweeps_keep_holes() {
    let outer = polygon(&[(0, 0), (4, 0), (4, 4), (0, 4)]);
    let hole = polygon(&[(1, 1), (3, 1), (3, 3), (1, 3)]);
    let swept = builder::sweep_curve_region(
        &outer,
        &[hole],
        Vector3::x(),
        Vector3::y(),
        quadratic_lateral_path(),
    )
    .unwrap();
    let solid = body(&swept);
    assert_exact(&solid.volume().unwrap(), &Real::from(12));
    let level = q(3, 8);
    // The section at z = 3/8 is shifted by 1/4.
    for (x, y, expected) in [
        (q(9, 4), Real::from(2), SolidPointLocation::Outside),
        (q(3, 4), Real::from(2), SolidPointLocation::Inside),
        (q(5, 4), Real::from(2), SolidPointLocation::Boundary),
        (q(13, 4), Real::from(2), SolidPointLocation::Boundary),
        (q(15, 4), Real::from(2), SolidPointLocation::Inside),
        (q(17, 4), Real::from(2), SolidPointLocation::Boundary),
        (q(1, 8), Real::from(2), SolidPointLocation::Outside),
    ] {
        let sample = point(x, y, level.clone());
        assert_eq!(
            solid.classify_point(&sample).unwrap(),
            expected,
            "{sample:?}"
        );
    }
    let replayed = replay(&swept);
    assert_exact(
        &replayed.solids().next().unwrap().volume().unwrap(),
        &Real::from(12),
    );
}

#[test]
fn reversed_traversal_and_affine_images_preserve_monotone_sweeps() {
    let square = polygon(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
    let forward = builder::sweep_curve(
        &square,
        Vector3::x(),
        Vector3::y(),
        quadratic_lateral_path(),
    )
    .unwrap();
    // The same set traversed from the top: the swapped frame keeps the
    // section orientation and the symmetric profile.
    let reversed = builder::sweep_curve(
        &square,
        Vector3::y(),
        Vector3::x(),
        bezier(&[
            (Real::one(), Real::one()),
            (Real::zero(), q(1, 4)),
            (Real::zero(), Real::zero()),
        ]),
    )
    .unwrap();
    assert_exact(&volume(&reversed), &Real::one());
    for sample in quadratic_lateral_samples() {
        let expected = quadratic_lateral_expected(&sample);
        assert_eq!(body(&reversed).classify_point(&sample).unwrap(), expected);
    }

    // Shear and nonuniform scale, then a reflection, applied to the model
    // and to the authored inputs give the same set: T(sweep) = sweep(T).
    let shear = Matrix4::from_row_major([
        Real::from(2),
        Real::zero(),
        q(1, 2),
        Real::one(),
        Real::zero(),
        Real::from(3),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        q(1, 3),
        Real::from(4),
        -Real::one(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ]);
    let reflection = Matrix4::affine_nonuniform_scale([-Real::one(), Real::one(), Real::one()]);
    let reflected_shear = Matrix4::from_row_major([
        -Real::from(2),
        Real::zero(),
        -q(1, 2),
        -Real::one(),
        Real::zero(),
        Real::from(3),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        q(1, 3),
        Real::from(4),
        -Real::one(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ]);
    for transform in [shear, reflection, reflected_shear] {
        let moved = forward.transformed(&transform).unwrap();
        let direction = |vector: Vector3| transform.transform_direction3(&vector);
        let image_path = || {
            Curve3::rational_bezier(
                [
                    point(Real::zero(), Real::zero(), Real::zero()),
                    point(Real::zero(), Real::zero(), q(1, 4)),
                    point(Real::one(), Real::zero(), Real::one()),
                ]
                .iter()
                .map(|control| transform.transform_point3(control).unwrap())
                .collect(),
                vec![Real::one(); 3],
            )
            .unwrap()
        };
        let authored = builder::sweep_curve(
            &square,
            direction(Vector3::x()),
            direction(Vector3::y()),
            image_path(),
        );
        let determinant = transform.determinant().abs();
        assert_exact(&volume(&moved), &determinant);
        let authored = match authored {
            Ok(authored) => Some(authored),
            // A reflected frame reverses section orientation; the swapped
            // frame authors the same set.
            Err(ConstructionError::NonMonotoneSweepPath) => Some(
                builder::sweep_curve(
                    &square,
                    direction(Vector3::y()),
                    direction(Vector3::x()),
                    image_path(),
                )
                .unwrap(),
            ),
            Err(error) => panic!("{error:?}"),
        };
        let authored = authored.unwrap();
        assert_exact(&volume(&authored), &determinant);
        for sample in quadratic_lateral_samples() {
            let expected = quadratic_lateral_expected(&sample);
            let image = transform.transform_point3(&sample).unwrap();
            assert_eq!(body(&moved).classify_point(&image).unwrap(), expected);
            assert_eq!(body(&authored).classify_point(&image).unwrap(), expected);
        }
    }
}

#[test]
fn monotone_sweep_cap_cuts_compose_with_booleans() {
    let square = polygon(&[(0, 0), (1, 0), (1, 1), (0, 1)]);
    let swept = builder::sweep_curve(
        &square,
        Vector3::x(),
        Vector3::y(),
        quadratic_lateral_path(),
    )
    .unwrap();
    let slab = builder::cuboid(
        point(-Real::from(2), -Real::from(2), q(1, 4)),
        point(Real::from(3), Real::from(3), q(3, 4)),
    )
    .unwrap();
    let middle = boolean::intersection(&swept, &slab).unwrap();
    let ends = boolean::difference(&swept, &slab).unwrap();
    assert_eq!(middle.len(), 1);
    assert_eq!(ends.len(), 2);
    assert_exact(&volume(&middle), &q(1, 2));
    assert_exact(&volume(&ends), &q(1, 2));
    let replayed_middle = replay(&middle);
    let replayed_ends = replay(&ends);
    let locate = |model: &Model, sample: &Point3| {
        let mut location = SolidPointLocation::Outside;
        for solid in model.solids() {
            match solid.classify_point(sample)? {
                SolidPointLocation::Inside => return Ok(SolidPointLocation::Inside),
                SolidPointLocation::Boundary => location = SolidPointLocation::Boundary,
                SolidPointLocation::Outside => {}
            }
        }
        Ok::<_, QueryError>(location)
    };
    for sample in quadratic_lateral_samples() {
        let expected = quadratic_lateral_expected(&sample);
        let in_slab = interval_location(&(&sample.z - q(1, 4)), &q(1, 2));
        let middle_expected = combine([expected, in_slab, SolidPointLocation::Inside]);
        assert_eq!(
            locate(middle.model(), &sample).unwrap(),
            middle_expected,
            "{sample:?}"
        );
        match locate(&replayed_middle, &sample) {
            Ok(location) => assert_eq!(location, middle_expected, "{sample:?}"),
            // The archive stores the cut path by its subdivided controls in
            // Q(√3, √7). An exact boundary point at the offset (3 - √5)/2
            // then needs a nested-radical zero proof; it stays undecided and
            // is never guessed. Points off the boundary still decide.
            Err(QueryError::Geometry(GeometryError::PlanarClassificationUnresolved(_)))
                if middle_expected == SolidPointLocation::Boundary => {}
            Err(error) => panic!("{sample:?}: {error:?}"),
        }
        if in_slab == SolidPointLocation::Outside {
            assert_eq!(locate(ends.model(), &sample).unwrap(), expected);
            assert_eq!(locate(&replayed_ends, &sample).unwrap(), expected);
        }
    }
}
