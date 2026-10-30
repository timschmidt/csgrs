//! Repeated defining factors retain source evidence through geometric consumers.
use hypercurve::{
    BooleanOp, Curve2, CurveParameter2, CurveParameterRange2, CurvePath2, CurveRegion2, FillRule,
    LineSeg2, Point2, PolynomialCurveSet2, Real, RegionPointLocation,
};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::bivariate_multiply;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|r| r.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn power(p: &BivariatePolynomial, n: usize) -> BivariatePolynomial {
    (0..n).fold(polynomial(&[&[1]]), |a, _| bivariate_multiply(&a, p))
}
fn curves(equation: &BivariatePolynomial) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        Default::default(),
    )
    .unwrap();
    assert_eq!(topology.equation(), equation);
    PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap()
}
fn line(x0: Real, y0: Real, x1: Real, y1: Real) -> Curve2 {
    LineSeg2::try_new(Point2::new(x0, y0), Point2::new(x1, y1))
        .unwrap()
        .into()
}
fn has_source(curve: &Curve2, original: &BivariatePolynomial, exponent: usize) {
    let branch = curve.polynomial_branch().unwrap();
    assert_eq!(branch.defining_exponent(), exponent);
    let topology = branch.topology();
    assert_eq!(topology.equation(), original);
    let factors = topology.repeated_factorization().unwrap().factors();
    assert_eq!(factors.len(), 1);
    assert_eq!(factors[0].exponent(), exponent);
}

#[test]
fn repeated_graph_keeps_geometric_jets_and_transverse_contacts() {
    let support = polynomial(&[&[0, 1], &[], &[-1]]); // y=x²
    for exponent in [2, 3] {
        let original = power(&support, exponent);
        let set = curves(&original);
        assert_eq!(set.curves().len(), 1);
        let curve = &set.curves()[0];
        let parameter: CurveParameter2 = q(1, 2).into();
        for piece in [curve.clone(), curve.reversed().unwrap()] {
            let jets = piece.derivatives_at(&parameter, 2).unwrap();
            for (jet, expected) in jets
                .iter()
                .zip([Point2::from_values(1, 1), Point2::from_values(0, 2)])
            {
                assert!(
                    jet.coordinate_image()
                        .unwrap()
                        .coincides_with(&expected.into())
                        .unwrap()
                );
            }
            let cut = line((-1).into(), q(1, 4), 1.into(), q(1, 4));
            let result = piece.intersect_curve(&cut).unwrap();
            assert!(result.is_complete() && result.overlaps().is_empty());
            assert_eq!(result.contacts().len(), 2);
            for contact in result.contacts() {
                assert!(contact.is_certified_transverse());
                assert!(
                    contact
                        .point()
                        .rational_form(support.clone(), polynomial(&[&[1]]))
                        .unwrap()
                        .compare_to_real(&Real::zero())
                        .unwrap()
                        .is_eq()
                );
            }
            let split = piece.intersection_topology(&cut).unwrap();
            assert_eq!(split.first().len(), 3);
            for part in split.first() {
                has_source(part, &original, exponent);
                assert!(part.intersect_curve(&cut).unwrap().is_complete());
            }
        }
    }
}

#[test]
fn repeated_circle_keeps_vertical_event_tangents() {
    let original = power(&polynomial(&[&[-1, 0, 1], &[], &[1]]), 3);
    let set = curves(&original);
    assert_eq!(set.curves().len(), 2);
    for (index, curve) in set.curves().iter().enumerate() {
        let branch = curve.polynomial_branch().unwrap();
        let tangent = branch.tangent_direction_at(&Real::from(-1).into()).unwrap();
        let point = tangent.coordinate_image().unwrap();
        assert!(
            point
                .affine_form(1.into(), 0.into(), 0.into())
                .compare_to_real(&Real::zero())
                .unwrap()
                .is_eq()
        );
        let side = point
            .affine_form(0.into(), 1.into(), 0.into())
            .compare_to_real(&Real::zero())
            .unwrap();
        assert_eq!(
            side,
            if index == 0 {
                std::cmp::Ordering::Less
            } else {
                std::cmp::Ordering::Greater
            }
        );
        let (left, right) = curve.split_at(Real::zero().into()).unwrap();
        assert!(left.end().coincides_with(&right.start()).unwrap());
        has_source(&left, &original, 3);
        has_source(&right, &original, 3);
    }
}

#[test]
fn mixed_factor_powers_preserve_component_overlaps_and_other_contacts() {
    let parabola = polynomial(&[&[0, 1], &[], &[-1]]);
    let diagonal = polynomial(&[&[0, 1], &[-1]]);
    let original = bivariate_multiply(&power(&parabola, 2), &power(&diagonal, 3));
    let set = curves(&original);
    let cut = line((-1).into(), (-1).into(), 1.into(), 1.into());
    let mut overlaps = 0;
    let mut finite_contacts = 0;
    for curve in set.curves() {
        let result = curve.intersect_curve(&cut).unwrap();
        assert!(result.is_complete());
        overlaps += result.overlaps().len();
        if result.overlaps().is_empty() {
            finite_contacts += result.contacts().len();
            for contact in result.contacts() {
                let point = contact.point();
                // The only finite contacts solve x²=x, so x is exactly 0 or 1.
                assert!(
                    point
                        .coincides_with(&Point2::from_values(0, 0).into())
                        .unwrap()
                        || point
                            .coincides_with(&Point2::from_values(1, 1).into())
                            .unwrap()
                );
            }
        }
        assert_eq!(
            curve.polynomial_branch().unwrap().topology().equation(),
            &original
        );
    }
    assert_eq!(overlaps, 2);
    assert_eq!(finite_contacts, 3);
}

fn parabola_region(exponent: usize) -> (CurveRegion2, BivariatePolynomial) {
    let original = power(&polynomial(&[&[0, 0, 1], &[-1]]), exponent);
    let set = curves(&original);
    assert_eq!(set.curves().len(), 2);
    let range = CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap();
    let path = CurvePath2::try_new(vec![
        set.curves()[0].restrict_source(range.clone()).unwrap(),
        line(q(1, 4), q(-1, 2), q(1, 4), q(1, 2)),
        set.curves()[1]
            .restrict_source(range)
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap();
    (
        CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap(),
        original,
    )
}

fn check_region(region: &CurveRegion2, original: &BivariatePolynomial) {
    assert_eq!(region.boundary_paths().unwrap().len(), 1);
    for curve in region.boundary_paths().unwrap()[0].curves() {
        if curve.polynomial_branch().is_some() {
            has_source(curve, original, 2);
            let point = curve.point_at(&q(1, 8).into()).unwrap();
            // The retained point satisfies the independent reduced relation y²=x.
            assert!(
                point
                    .rational_form(polynomial(&[&[0, 0, 1], &[-1]]), polynomial(&[&[1]]))
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap()
                    .is_eq()
            );
        }
    }
    for (x, y, location) in [
        (q(1, 8), q(0, 1), RegionPointLocation::Inside),
        (q(1, 8), q(1, 2), RegionPointLocation::Outside),
        (q(1, 4), q(0, 1), RegionPointLocation::Boundary),
    ] {
        assert_eq!(
            region.classify_point(&Point2::new(x, y).into()).unwrap(),
            location
        );
    }
    let (simple, _) = parabola_region(1);
    assert!(
        region
            .boolean_region(&simple, BooleanOp::Xor)
            .unwrap()
            .is_empty()
    );
    let rebuilt =
        CurveRegion2::try_from_boundary_paths(&region.boundary_paths().unwrap(), FillRule::EvenOdd)
            .unwrap();
    let empty = rebuilt
        .boolean_region(region, BooleanOp::Difference)
        .unwrap();
    assert!(empty.is_empty());
    let restored = empty.boolean_region(&rebuilt, BooleanOp::Union).unwrap();
    assert_eq!(restored.boundary_paths().unwrap()[0].curves().len(), 3);
}

#[test]
fn repeated_boundary_uses_the_same_filled_set_and_arrangement() {
    let (region, original) = parabola_region(2);
    check_region(&region, &original);
}

#[cfg(feature = "svg")]
#[test]
fn repeated_source_replays_in_a_fresh_process_and_composes() {
    use hypercurve::{SvgGeometry2, export_svg_document, import_svg_document};
    const INPUT: &str = "HYPERCURVE_MULTIPLICITY_REPLAY_INPUT";
    if let Some(path) = std::env::var_os(INPUT) {
        let text = std::fs::read_to_string(path).unwrap();
        let restored = import_svg_document(&text).unwrap();
        let original = power(&polynomial(&[&[0, 0, 1], &[-1]]), 2);
        check_region(restored.region(), &original);
        assert_eq!(export_svg_document(&restored).unwrap(), text);
        return;
    }
    let (region, original) = parabola_region(2);
    check_region(&region, &original);
    let text = export_svg_document(&SvgGeometry2::new(region, vec![], vec![])).unwrap();
    let path = std::env::temp_dir().join(format!(
        "hypercurve-multiplicities-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, text).unwrap();
    let result = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "repeated_source_replays_in_a_fresh_process_and_composes",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(&path).unwrap();
    assert!(
        result.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
}

#[test]
fn repeated_circle_consumes_an_authored_rational_overlap() {
    let original = power(&polynomial(&[&[-1, 0, 1], &[], &[1]]), 2);
    let set = curves(&original);
    let rational: Curve2 = hypercurve::RationalQuadraticBezier2::try_new(
        Point2::from_values(1, 0),
        Point2::from_values(1, -1),
        Point2::from_values(0, -1),
        1.into(),
        1.into(),
        2.into(),
    )
    .unwrap()
    .into();
    let lower = &set.curves()[0];
    for (first, second) in [(lower, &rational), (&rational, lower)] {
        let result = first.intersect_curve(second).unwrap();
        assert!(result.is_complete());
        assert_eq!(result.overlaps().len(), 1);
        for overlap in result.overlaps() {
            for (a, b) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                assert!(
                    first
                        .point_at(a)
                        .unwrap()
                        .coincides_with(&second.point_at(b).unwrap())
                        .unwrap()
                );
            }
        }
        let split = first.intersection_topology(second).unwrap();
        for part in split.first() {
            assert!(part.intersect_curve(second).unwrap().is_complete());
        }
    }
}

#[test]
fn repeated_parabola_consumes_a_generated_selected_chord() {
    let original = power(&polynomial(&[&[0, 0, 1], &[-1]]), 2);
    let set = curves(&original);
    let positive = &set.curves()[1];
    let constant = curves(&polynomial(&[&[-1, 0, 0, 128]]));
    let source = &constant.curves()[0];
    let map = hypercurve::RationalMap2::new(
        [polynomial(&[&[], &[1]]), polynomial(&[&[0, 1], &[1]])],
        polynomial(&[&[1]]),
    );
    let generated = Curve2::try_line(
        source.start().rational_image(&map).unwrap(),
        source.end().rational_image(&map).unwrap(),
    )
    .unwrap();
    for piece in [positive.clone(), positive.reversed().unwrap()] {
        let result = piece.intersect_curve(&generated).unwrap();
        assert!(result.is_complete() && result.overlaps().is_empty());
        assert_eq!(result.contacts().len(), 2);
        for contact in result.contacts() {
            assert!(contact.is_certified_transverse());
            let point = contact.point();
            assert!(
                point
                    .rational_form(polynomial(&[&[0, 0, 1], &[-1]]), polynomial(&[&[1]]))
                    .unwrap()
                    .compare_to_real(&Real::zero())
                    .unwrap()
                    .is_eq()
            );
            // Original chord definition: (y-x)^3=1/128, without rounding alpha.
            let delta = point
                .rational_form(polynomial(&[&[0, 1], &[-1]]), polynomial(&[&[1]]))
                .unwrap();
            assert!(
                delta
                    .multiply(&delta)
                    .unwrap()
                    .multiply(&delta)
                    .unwrap()
                    .compare_to_real(&q(1, 128))
                    .unwrap()
                    .is_eq()
            );
        }
        has_source(&piece, &original, 2);
    }
}

#[test]
fn repeated_branch_chamfer_retains_circle_setback_and_source() {
    use hypercurve::{CurveCornerMode2, CurveCornerSolutions2};
    let original = power(&polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]), 2);
    let set = curves(&original);
    let branch = set.curves()[1]
        .restrict_source(
            CurveParameterRange2::try_new(Real::from(-1).into(), Real::zero().into()).unwrap(),
        )
        .unwrap();
    let corner = branch.end();
    let path = CurvePath2::try_new(vec![
        branch,
        Curve2::try_line(corner.clone(), Point2::from_values(1, 1).into()).unwrap(),
    ])
    .unwrap();
    let setback = q(1, 8);
    let CurveCornerSolutions2::Unique(result) = path
        .chamfer_vertex_by_setbacks(
            1,
            setback.clone(),
            setback.clone(),
            CurveCornerMode2::TrimOnly,
        )
        .unwrap()
    else {
        panic!("unique trim-only chord setbacks");
    };
    assert_eq!(result.curves().len(), 3);
    has_source(&result.curves()[0], &original, 2);
    for cut in [result.curves()[0].end(), result.curves()[2].start()] {
        let dx = cut
            .rational_form(polynomial(&[&[], &[1]]), polynomial(&[&[1]]))
            .unwrap()
            .subtract(
                &corner
                    .rational_form(polynomial(&[&[], &[1]]), polynomial(&[&[1]]))
                    .unwrap(),
            )
            .unwrap();
        let dy = cut
            .rational_form(polynomial(&[&[0, 1]]), polynomial(&[&[1]]))
            .unwrap()
            .subtract(
                &corner
                    .rational_form(polynomial(&[&[0, 1]]), polynomial(&[&[1]]))
                    .unwrap(),
            )
            .unwrap();
        let distance = dx
            .multiply(&dx)
            .unwrap()
            .add(&dy.multiply(&dy).unwrap())
            .unwrap();
        assert!(
            distance
                .compare_to_real(&(&setback * &setback))
                .unwrap()
                .is_eq()
        );
    }
    let contact = result.curves()[0]
        .intersect_curve(&result.curves()[1])
        .unwrap();
    assert!(contact.is_complete());
    assert_eq!(contact.contacts().len(), 1);
    assert!(result.start().coincides_with(&path.start()).unwrap());
    assert!(result.end().coincides_with(&path.end()).unwrap());
}
