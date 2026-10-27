//! Horizontal domain components remain exact source graphs in later queries.

use hypercurve::{
    BezierParameterPolynomial, Curve2, CurveParameter2, CurveParameterRange2, CurvePoint2,
    CurvePointLocations2, LineSeg2, Point2, PolynomialCurveSet2, Real, Similarity2,
};
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};
use std::cmp::Ordering;

fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}

fn set(equation: BivariatePolynomial, domain: BivariateCurveDomain) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(&equation, &domain, Default::default()).unwrap();
    assert_eq!(topology.equation(), &equation);
    PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap()
}

fn square() -> BivariateCurveDomain {
    BivariateCurveDomain {
        first: [(-2).into(), 2.into()],
        second: [(-2).into(), 2.into()],
    }
}

fn line(x0: Real, y0: Real, x1: Real, y1: Real) -> Curve2 {
    LineSeg2::try_new(Point2::new(x0, y0), Point2::new(x1, y1))
        .unwrap()
        .into()
}

fn located(curve: &Curve2, point: &CurvePoint2, expected: &CurveParameter2) {
    let CurvePointLocations2::Locations(locations) = curve.point_locations(point).unwrap() else {
        panic!("a graph has a unique preimage");
    };
    assert_eq!(locations.len(), 1);
    assert_eq!(
        locations[0].parameter().unwrap().compare(expected).unwrap(),
        Ordering::Equal
    );
    assert!(
        curve
            .point_at(expected)
            .unwrap()
            .coincides_with(point)
            .unwrap()
    );
}

fn alpha() -> CurveParameter2 {
    let roots = BezierParameterPolynomial::try_new_power_basis(
        [-5, 5, 1, 0, 0, 1].into_iter().map(Real::from).collect(),
    )
    .unwrap()
    .isolate_unit_interval_roots()
    .unwrap();
    assert_eq!(roots.len(), 1);
    roots[0].clone().into()
}

#[test]
fn boundary_line_consumes_selected_trims_and_contacts() {
    let set = set(polynomial(&[&[2, 1]]), square());
    assert_eq!(set.curves().len(), 1);
    let source = &set.curves()[0];
    let alpha = alpha();
    let point = source.point_at(&alpha).unwrap();
    assert_eq!(
        point
            .affine_form(0.into(), 1.into(), 2.into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    let replay = point.definition().unwrap().into_point().unwrap();
    let trim = source
        .restrict_source(CurveParameterRange2::try_new(alpha.clone(), Real::one().into()).unwrap())
        .unwrap();
    let cut = line(1.into(), (-3).into(), 1.into(), (-1).into());
    for piece in [trim.clone(), trim.reversed().unwrap()] {
        assert!(
            piece
                .polynomial_branch()
                .unwrap()
                .shares_source(source.polynomial_branch().unwrap())
        );
        located(&piece, &replay, &alpha);
        let intersections = piece.intersect_curve(&cut).unwrap();
        assert!(intersections.is_complete() && intersections.overlaps().is_empty());
        assert_eq!(intersections.contacts().len(), 1);
        let contact = &intersections.contacts()[0];
        assert!(
            contact
                .point()
                .coincides_with(&Point2::from_values(1, -2).into())
                .unwrap()
        );
        located(&piece, contact.point(), &Real::one().into());
        let derivative = piece
            .derivative_at(&alpha)
            .unwrap()
            .coordinate_image()
            .unwrap();
        assert!(
            derivative
                .coincides_with(&Point2::from_values(1, 0).into())
                .unwrap()
        );
    }
}

#[test]
fn both_boundary_lines_distinguish_points_and_overlaps() {
    let set = set(polynomial(&[&[-4, 0, 1]]), square());
    assert_eq!(set.curves().len(), 2);
    for (index, y) in [-2, 2].into_iter().enumerate() {
        let curve = &set.curves()[index];
        let point = CurvePoint2::from(Point2::from_values(0, y));
        located(curve, &point, &Real::zero().into());
        let CurvePointLocations2::Locations(other) =
            set.curves()[1 - index].point_locations(&point).unwrap()
        else {
            panic!("finite graph locations");
        };
        assert!(other.is_empty());
        let authored = line((-1).into(), y.into(), 1.into(), y.into());
        let overlap = curve.intersect_curve(&authored).unwrap();
        assert!(overlap.is_complete());
        assert_eq!(overlap.overlaps().len(), 1);
        let reconstructed = curve.intersection_topology(&authored).unwrap();
        assert_eq!(reconstructed.first().len(), 3);
        for piece in reconstructed.first() {
            assert!(
                piece
                    .polynomial_branch()
                    .unwrap()
                    .shares_source(curve.polynomial_branch().unwrap())
            );
            assert!(piece.intersect_curve(&authored).unwrap().is_complete());
        }
        let vertical = line(0.into(), (-3).into(), 0.into(), 3.into());
        let contact = curve.intersect_curve(&vertical).unwrap();
        assert_eq!(contact.contacts().len(), 1);
        assert!(
            contact.contacts()[0]
                .point()
                .coincides_with(&point)
                .unwrap()
        );
    }
}

#[test]
fn boundary_contact_retains_all_incident_branch_endpoints() {
    // y(y-x^2)=0: four incident arcs meet without losing the boundary line.
    let equation = polynomial(&[&[0, 0, 1], &[], &[0, -1]]);
    let set = set(
        equation.clone(),
        BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [0.into(), 2.into()],
        },
    );
    assert_eq!(set.curves().len(), 4);
    let origin = CurvePoint2::from(Point2::from_values(0, 0));
    for curve in set.curves() {
        assert_eq!(
            curve.polynomial_branch().unwrap().topology().equation(),
            &equation
        );
        located(curve, &origin, &Real::zero().into());
        located(&curve.reversed().unwrap(), &origin, &Real::zero().into());
        let replay = curve
            .point_at(&Real::zero().into())
            .unwrap()
            .definition()
            .unwrap()
            .into_point()
            .unwrap();
        assert!(replay.coincides_with(&origin).unwrap());
    }
    for pair in [(0, 1), (2, 3)] {
        let result = set.curves()[pair.0]
            .intersect_curve(&set.curves()[pair.1])
            .unwrap();
        assert_eq!(result.contacts().len(), 1);
        assert!(result.overlaps().is_empty());
        assert!(
            result.contacts()[0]
                .point()
                .coincides_with(&origin)
                .unwrap()
        );
    }
}

#[test]
fn boundary_graph_similarity_preserves_parameter_and_incidence() {
    let set = set(polynomial(&[&[2, 1]]), square());
    let transform = Similarity2::try_from_real_affine(
        0.into(),
        (-3).into(),
        3.into(),
        0.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let source = &set.curves()[0];
    let curve = source
        .reversed()
        .unwrap()
        .transform_similarity(&transform)
        .unwrap();
    let parameter = alpha();
    let point = hypercurve::CurvePointDefinition2::Similarity {
        source: source.point_at(&parameter).unwrap(),
        transform,
    }
    .into_point()
    .unwrap();
    // T(x,-2)=(8,3x-1): retained x is the parameter even on the vertical image.
    assert_eq!(
        point
            .affine_form(1.into(), 0.into(), (-8).into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    located(&curve, &point, &parameter);
    let cut = line(7.into(), (-1).into(), 9.into(), (-1).into());
    let result = curve.intersect_curve(&cut).unwrap();
    assert_eq!(result.contacts().len(), 1);
    assert!(
        result.contacts()[0]
            .point()
            .coincides_with(&Point2::from_values(8, -1).into())
            .unwrap()
    );
    located(&curve, result.contacts()[0].point(), &Real::zero().into());
}

#[test]
fn boundary_rational_similarity_views_retain_original_guards_and_contacts() {
    // Every map is an exact similarity, while its original quotient remains
    // authoritative. The removable factor 3+x has no zero on [-2,2]^2.
    let identity = hypercurve::RationalMap2::new(
        [polynomial(&[&[], &[1]]), polynomial(&[&[0, 1]])],
        polynomial(&[&[1]]),
    );
    let guarded_identity = hypercurve::RationalMap2::new(
        [
            polynomial(&[&[], &[3], &[1]]),
            polynomial(&[&[0, 3], &[0, 1]]),
        ],
        polynomial(&[&[3], &[1]]),
    );
    // T(x,y)=(5-2y,2x-1).
    let turn = hypercurve::RationalMap2::new(
        [polynomial(&[&[5, -2]]), polynomial(&[&[-1], &[2]])],
        polynomial(&[&[1]]),
    );
    let reflect = hypercurve::RationalMap2::new(
        [polynomial(&[&[], &[1]]), polynomial(&[&[0, -1]])],
        polynomial(&[&[1]]),
    );
    let reflection = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        (-1).into(),
        0.into(),
        0.into(),
    )
    .unwrap();
    let transform = Similarity2::try_from_real_affine(
        0.into(),
        (-2).into(),
        2.into(),
        0.into(),
        5.into(),
        (-1).into(),
    )
    .unwrap();
    let source = set(polynomial(&[&[2, 1]]), square()).curves()[0].clone();
    let contact_point = CurvePoint2::from(Point2::from_values(0, -2));
    for first in [identity, guarded_identity] {
        let original = source.rational_image(&first).unwrap();
        let rotated = original.rational_image(&turn).unwrap();
        let reflected = rotated.rational_image(&reflect).unwrap();
        let outer = rotated.transform_similarity(&reflection).unwrap();
        for (curve, cut, overlap_line, point, maps, sign) in [
            (
                original,
                line(0.into(), (-3).into(), 0.into(), (-1).into()),
                line((-1).into(), (-2).into(), 1.into(), (-2).into()),
                contact_point.clone(),
                vec![first.clone()],
                hyperreal::RealSign::Positive,
            ),
            (
                rotated,
                line(0.into(), (-3).into(), 0.into(), (-1).into())
                    .transform_similarity(&transform)
                    .unwrap(),
                line(9.into(), (-3).into(), 9.into(), 1.into()),
                hypercurve::CurvePointDefinition2::Similarity {
                    source: contact_point.clone(),
                    transform: transform.clone(),
                }
                .into_point()
                .unwrap(),
                vec![first.clone(), turn.clone()],
                hyperreal::RealSign::Positive,
            ),
            (
                reflected,
                line(11.into(), 1.into(), 7.into(), 1.into()),
                line(9.into(), 3.into(), 9.into(), (-1).into()),
                Point2::from_values(9, 1).into(),
                vec![first.clone(), turn.clone(), reflect.clone()],
                hyperreal::RealSign::Negative,
            ),
            (
                outer,
                line(11.into(), 1.into(), 7.into(), 1.into()),
                line(9.into(), 3.into(), 9.into(), (-1).into()),
                Point2::from_values(9, 1).into(),
                vec![first.clone(), turn.clone()],
                hyperreal::RealSign::Negative,
            ),
        ] {
            for (reverse, piece) in [curve.clone(), curve.reversed().unwrap()]
                .into_iter()
                .enumerate()
            {
                let result = piece.intersect_curve(&cut).unwrap();
                assert!(result.is_complete() && result.overlaps().is_empty());
                assert_eq!(result.contacts().len(), 1);
                let contact = &result.contacts()[0];
                assert!(contact.point().coincides_with(&point).unwrap());
                assert_eq!(
                    contact
                        .first()
                        .parameter()
                        .unwrap()
                        .compare(&Real::zero().into())
                        .unwrap(),
                    Ordering::Equal
                );
                assert!(contact.is_certified_transverse());
                let negated = |sign| match sign {
                    hyperreal::RealSign::Positive => hyperreal::RealSign::Negative,
                    hyperreal::RealSign::Negative => hyperreal::RealSign::Positive,
                    hyperreal::RealSign::Zero => hyperreal::RealSign::Zero,
                };
                let expected = if reverse == 0 { sign } else { negated(sign) };
                assert_eq!(contact.tangent_cross_sign(), Some(expected));
                let swapped = cut.intersect_curve(&piece).unwrap();
                assert_eq!(swapped.contacts().len(), 1);
                assert_eq!(
                    swapped.contacts()[0].tangent_cross_sign(),
                    Some(negated(expected))
                );
                assert_eq!(piece.polynomial_branch().unwrap().rational_maps(), maps);
                assert!(
                    piece
                        .point_at(&contact.first().parameter().unwrap())
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
                assert!(
                    cut.point_at(&contact.second().parameter().unwrap())
                        .unwrap()
                        .coincides_with(contact.point())
                        .unwrap()
                );
                let overlap = piece.intersection_topology(&overlap_line).unwrap();
                assert_eq!(overlap.result().overlaps().len(), 1);
                assert_eq!(overlap.first().len(), 3);
                let mut contacts = 0;
                for part in overlap.first() {
                    assert_eq!(part.polynomial_branch().unwrap().rational_maps(), maps);
                    let result = part.intersect_curve(&cut).unwrap();
                    assert!(result.is_complete());
                    contacts += result.contacts().len();
                }
                assert_eq!(contacts, 1);
            }
        }
    }
    // An identical quotient with a zero original denominator is not admitted.
    let pole = hypercurve::RationalMap2::new(
        [polynomial(&[&[], &[], &[1]]), polynomial(&[&[], &[0, 1]])],
        polynomial(&[&[], &[1]]),
    );
    assert!(source.rational_image(&pole).is_err());
    for map in [
        hypercurve::RationalMap2::new(
            [polynomial(&[&[], &[2]]), polynomial(&[&[0, 3]])],
            polynomial(&[&[1]]),
        ),
        hypercurve::RationalMap2::new(
            [polynomial(&[&[], &[8], &[1]]), polynomial(&[&[0, 8]])],
            polynomial(&[&[8]]),
        ),
    ] {
        let image = source.rational_image(&map).unwrap();
        let error = image
            .intersect_curve(&line(0.into(), (-8).into(), 0.into(), 8.into()))
            .unwrap_err();
        assert!(
            matches!(error, hypercurve::ExactCurveError::Blocked(blocker) if blocker.reason() == hypercurve::UncertaintyReason::Unsupported)
        );
    }
}

#[test]
fn boundary_graph_replays_source_in_fresh_process() {
    const REPLAY: &str = "HYPERCURVE_BOUNDARY_SOURCE_REPLAY";
    if let Some(path) = std::env::var_os(REPLAY) {
        // Diagnostic source replay, not a public native-curve archive format.
        // There are no cached root selectors or trusted connectivity flags.
        let record = std::fs::read_to_string(path).unwrap();
        let values = |value: &str| -> Vec<Real> {
            value
                .split_whitespace()
                .map(|v| v.parse().unwrap())
                .collect()
        };
        let mut rows = record.lines();
        let domain = BivariateCurveDomain {
            first: values(rows.next().unwrap()).try_into().unwrap(),
            second: values(rows.next().unwrap()).try_into().unwrap(),
        };
        let equation = BivariatePolynomial::new(rows.map(values).collect());
        let set = set(equation, domain);
        assert_eq!(set.curves().len(), 2);
        for (index, y) in [-2, 2].into_iter().enumerate() {
            let curve = &set.curves()[index];
            let contact = curve
                .intersect_curve(&line(0.into(), (-3).into(), 0.into(), 3.into()))
                .unwrap();
            assert_eq!(contact.contacts().len(), 1);
            located(
                curve,
                &Point2::from_values(0, y).into(),
                &Real::zero().into(),
            );
            let trim = curve
                .restrict_source(
                    CurveParameterRange2::try_new(alpha(), Real::one().into()).unwrap(),
                )
                .unwrap();
            assert!(trim.point_at(trim.parameter_domain().start()).is_ok());
        }
        return;
    }
    let source = set(polynomial(&[&[-4, 0, 1]]), square());
    let strings = |values: &[Real]| {
        values
            .iter()
            .map(|v| v.exact_rational_normal_form().unwrap().to_string())
            .collect::<Vec<_>>()
    };
    let mut rows = vec![
        strings(&source.topology().domain().first).join(" "),
        strings(&source.topology().domain().second).join(" "),
    ];
    rows.extend(
        source
            .topology()
            .equation()
            .coefficients
            .iter()
            .map(|row| strings(row).join(" ")),
    );
    let record = rows.join("\n") + "\n";
    let path = std::env::temp_dir().join(format!(
        "hypercurve-boundary-replay-{}-{}.txt",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(&path, record).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "boundary_graph_replays_source_in_fresh_process",
            "--exact",
            "--nocapture",
        ])
        .env(REPLAY, &path)
        .output()
        .unwrap();
    std::fs::remove_file(&path).unwrap();
    assert!(
        child.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
