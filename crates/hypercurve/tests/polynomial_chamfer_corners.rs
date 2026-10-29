//! Exact chord setbacks retain selected implicit supports and their point evidence.

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveCornerMode2, CurveCornerNoSolution2,
    CurveCornerSolutions2, CurveParameter2, CurveParameterRange2, CurvePath2, CurvePoint2,
    CurvePointRationalForm2, ExactCurveError, Point2, PolynomialCurveSet2, Real, Similarity2,
    UncertaintyReason,
};
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial};
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
    BivariatePolynomial::new(
        rows.iter()
            .map(|row| row.iter().copied().map(Real::from).collect())
            .collect(),
    )
}
fn form(point: &CurvePoint2, numerator: BivariatePolynomial) -> CurvePointRationalForm2 {
    point.rational_form(numerator, polynomial(&[&[1]])).unwrap()
}
fn distance_squared(first: &CurvePoint2, second: &CurvePoint2) -> CurvePointRationalForm2 {
    let dx = form(first, polynomial(&[&[], &[1]]))
        .subtract(&form(second, polynomial(&[&[], &[1]])))
        .unwrap();
    let dy = form(first, polynomial(&[&[0, 1]]))
        .subtract(&form(second, polynomial(&[&[0, 1]])))
        .unwrap();
    dx.multiply(&dx)
        .unwrap()
        .add(&dy.multiply(&dy).unwrap())
        .unwrap()
}
fn source(algebraic: bool) -> CurvePath2 {
    let equation = polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 0.into()],
            second: [(-1).into(), 1.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let set =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    let end: CurveParameter2 = if algebraic {
        BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(vec![
                    q(3, 4),
                    0.into(),
                    0.into(),
                    1.into(),
                ])
                .unwrap(),
                BezierParameterInterval::try_new((-1).into(), q(-1, 2)).unwrap(),
            )
            .unwrap(),
        )
        .into()
    } else {
        Real::zero().into()
    };
    let branch = set.curves()[1]
        .restrict_source(CurveParameterRange2::try_new(Real::from(-1).into(), end).unwrap())
        .unwrap();
    let line = Curve2::try_line(
        branch.end(),
        Point2::new(1.into(), if algebraic { q(1, 2) } else { 1.into() }).into(),
    )
    .unwrap();
    CurvePath2::try_new(vec![branch, line]).unwrap()
}
fn unique(source: &CurvePath2, first: Real, second: Real) -> CurvePath2 {
    match source
        .chamfer_vertex_by_setbacks(1, first, second, CurveCornerMode2::TrimOnly)
        .unwrap()
    {
        CurveCornerSolutions2::Unique(result) => result,
        other => panic!("one finite chord-setback solution expected: {other:?}"),
    }
}
fn verify(source: &CurvePath2, result: &CurvePath2, reversed: bool, setback: &Real) {
    assert_eq!(result.curves().len(), 3);
    let index = if reversed { 2 } else { 0 };
    let original = &source.curves()[if reversed { 1 } else { 0 }];
    let retained = &result.curves()[index];
    assert_eq!(
        original.polynomial_branch().unwrap().topology().equation(),
        retained.polynomial_branch().unwrap().topology().equation()
    );
    let corner = if reversed {
        original.start()
    } else {
        original.end()
    };
    let cut = if reversed {
        retained.start()
    } else {
        retained.end()
    };
    assert_eq!(
        distance_squared(&cut, &corner)
            .compare_to_real(&(setback * setback))
            .unwrap(),
        Ordering::Equal
    );
    let parameter = if reversed {
        retained.parameter_endpoints().0
    } else {
        retained.parameter_endpoints().1
    };
    assert_eq!(
        parameter
            .compare(original.parameter_domain().start())
            .unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        parameter
            .compare(original.parameter_domain().end())
            .unwrap(),
        Ordering::Less
    );
    assert!(
        original
            .point_at(parameter)
            .unwrap()
            .coincides_with(&cut)
            .unwrap()
    );
    for pair in result.curves().windows(2) {
        assert!(pair[0].end().coincides_with(&pair[1].start()).unwrap());
    }
    assert!(result.start().coincides_with(&source.start()).unwrap());
    assert!(result.end().coincides_with(&source.end()).unwrap());
}

#[test]
fn polynomial_chamfer_retains_algebraic_cuts_through_reversal_and_similarity() {
    let transform = Similarity2::try_from_real_affine(
        0.into(),
        (-2).into(),
        2.into(),
        0.into(),
        1.into(),
        3.into(),
    )
    .unwrap();
    for algebraic in [false, true] {
        for moved in [false, true] {
            let original = source(algebraic);
            let original = if moved {
                original.transform_similarity(&transform).unwrap()
            } else {
                original
            };
            for reversed in [false, true] {
                let original = if reversed {
                    original.reversed().unwrap()
                } else {
                    original.clone()
                };
                let setback = if moved { q(1, 32) } else { q(1, 64) };
                let result = unique(&original, setback.clone(), setback.clone());
                verify(&original, &result, reversed, &setback);
                if !moved {
                    let cut = if reversed {
                        result.curves()[2].start()
                    } else {
                        result.curves()[0].end()
                    };
                    assert_eq!(
                        form(&cut, polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]))
                            .compare_to_real(&Real::zero())
                            .unwrap(),
                        Ordering::Equal
                    );
                }
            }
        }
    }
}

#[test]
fn polynomial_chamfer_preserves_zero_invalid_empty_and_extension_outcomes() {
    let original = source(true);
    let result = unique(&original, 0.into(), q(1, 64));
    assert!(
        result.curves()[0]
            .end()
            .coincides_with(&original.curves()[0].end())
            .unwrap()
    );
    assert_eq!(
        original
            .chamfer_vertex_by_setbacks(1, 0.into(), 0.into(), CurveCornerMode2::TrimOnly)
            .unwrap()
            .no_solution_reason(),
        Some(CurveCornerNoSolution2::ZeroDesignValue)
    );
    assert_eq!(
        original
            .chamfer_vertex_by_setbacks(1, 4.into(), q(1, 64), CurveCornerMode2::TrimOnly)
            .unwrap()
            .no_solution_reason(),
        Some(CurveCornerNoSolution2::OutsideTrimDomain)
    );
    assert!(matches!(
        original.chamfer_vertex_by_setbacks(1, (-1).into(), q(1, 64), CurveCornerMode2::TrimOnly),
        Err(ExactCurveError::Invalid { .. })
    ));
    assert!(
        matches!(original.chamfer_vertex_by_setbacks(1, q(1, 64), q(1, 64), CurveCornerMode2::TrimOrExtend), Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::Unsupported)
    );
}

#[cfg(feature = "svg")]
#[test]
fn polynomial_chamfer_replays_in_a_fresh_process_and_reuses_selected_cuts() {
    use hypercurve::{CurveRegion2, SvgGeometry2, export_svg_document, import_svg_document};
    const INPUT: &str = "HYPERCURVE_POLYNOMIAL_CHAMFER_REPLAY";
    fn replay(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        verify(&geometry.paths()[0], &geometry.paths()[1], false, &q(1, 64));
        let cut = geometry.paths()[1].curves()[0].end();
        assert_eq!(
            form(&cut, polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]))
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        replay(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let original = source(true);
    let result = unique(&original, q(1, 64), q(1, 64));
    let text = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![original, result],
    ))
    .unwrap();
    replay(&text);
    let path = std::env::temp_dir().join(format!(
        "hypercurve-polynomial-chamfer-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "polynomial_chamfer_replays_in_a_fresh_process_and_reuses_selected_cuts",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn polynomial_chamfer_enumerates_multiple_finite_cuts() {
    // y=8x(2x+1)(x+1) leaves and re-enters the radius-3/4 disk.
    let equation = polynomial(&[&[0, 1], &[-8], &[-24], &[-16]]);
    let topology = decompose_bivariate_curve(
        &equation,
        &BivariateCurveDomain {
            first: [(-1).into(), 0.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    let set =
        PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
            .unwrap();
    assert_eq!(set.curves().len(), 1);
    let branch = set.curves()[0].clone();
    let corner = branch.end();
    let path = CurvePath2::try_new(vec![
        branch,
        Curve2::try_line(corner.clone(), Point2::from_values(1, 0).into()).unwrap(),
    ])
    .unwrap();
    let solutions = path
        .chamfer_vertex_by_setbacks(1, q(3, 4), q(1, 64), CurveCornerMode2::TrimOnly)
        .unwrap();
    assert_eq!(solutions.candidate_count(), 3);
    let mut counts = [0; 3];
    for candidate in solutions.solutions() {
        let point = candidate.curves()[0].end();
        assert_eq!(
            form(&point, equation.clone())
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            distance_squared(&point, &corner)
                .compare_to_real(&q(9, 16))
                .unwrap(),
            Ordering::Equal
        );
        let x = form(&point, polynomial(&[&[], &[1]]));
        for (index, (lower, upper)) in [
            (q(-3, 4), q(-1, 2)),
            (q(-1, 2), q(-1, 4)),
            (q(-1, 4), Real::zero()),
        ]
        .into_iter()
        .enumerate()
        {
            if x.compare_to_real(&lower).unwrap() == Ordering::Greater
                && x.compare_to_real(&upper).unwrap() == Ordering::Less
            {
                counts[index] += 1;
            }
        }
    }
    assert_eq!(counts, [1, 1, 1]);
}

#[test]
fn polynomial_chamfer_consumes_its_generated_chord_in_a_second_edit() {
    let first = unique(&source(true), q(1, 64), q(1, 64));
    let corner = first.curves()[0].end();
    let second = unique(&first, q(1, 512), q(1, 512));
    assert_eq!(second.curves().len(), 4);
    let retained = &second.curves()[0];
    let cut = retained.end();
    assert_eq!(
        form(&cut, polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]))
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        distance_squared(&cut, &corner)
            .compare_to_real(&(&q(1, 512) * &q(1, 512)))
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        first.curves()[0]
            .point_at(retained.parameter_endpoints().1)
            .unwrap()
            .coincides_with(&cut)
            .unwrap()
    );
    let generated_cut = second.curves()[1].end();
    let chord = &first.curves()[1];
    let x = polynomial(&[&[], &[1]]);
    let y = polynomial(&[&[0, 1]]);
    let dx = form(&chord.end(), x.clone())
        .subtract(&form(&chord.start(), x.clone()))
        .unwrap();
    let dy = form(&chord.end(), y.clone())
        .subtract(&form(&chord.start(), y.clone()))
        .unwrap();
    let vx = form(&generated_cut, x.clone())
        .subtract(&form(&chord.start(), x))
        .unwrap();
    let vy = form(&generated_cut, y.clone())
        .subtract(&form(&chord.start(), y))
        .unwrap();
    assert_eq!(
        dx.multiply(&vy)
            .unwrap()
            .subtract(&dy.multiply(&vx).unwrap())
            .unwrap()
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        distance_squared(&generated_cut, &corner)
            .compare_to_real(&(&q(1, 512) * &q(1, 512)))
            .unwrap(),
        Ordering::Equal
    );
    let progress = dx
        .multiply(&vx)
        .unwrap()
        .add(&dy.multiply(&vy).unwrap())
        .unwrap();
    assert_eq!(
        progress.compare_to_real(&Real::zero()).unwrap(),
        Ordering::Greater
    );
    assert_eq!(
        progress
            .compare(&distance_squared(&chord.start(), &chord.end()))
            .unwrap(),
        Ordering::Less
    );
}

#[cfg(feature = "svg")]
#[test]
fn polynomial_chamfer_cut_points_replay_in_a_fresh_process() {
    use hypercurve::{CurveRegion2, SvgGeometry2, export_svg_document, import_svg_document};
    const INPUT: &str = "HYPERCURVE_POLYNOMIAL_CHAMFER_POINT_REPLAY";
    fn replay(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        let line = &geometry.paths()[0].curves()[0];
        let center = line.start();
        let cut = line.end();
        assert_eq!(
            form(&center, polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]))
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            form(&center, polynomial(&[&[0, 1]]))
                .compare_to_real(&q(1, 2))
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            form(&center, polynomial(&[&[], &[1]]))
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Less
        );
        assert_eq!(
            form(&cut, polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]))
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            distance_squared(&cut, &center)
                .compare_to_real(&q(1, 4096))
                .unwrap(),
            Ordering::Equal
        );
        let original = source(true);
        let parameter = form(&cut, polynomial(&[&[], &[1]])).parameter().unwrap();
        assert_eq!(
            parameter
                .compare(original.curves()[0].parameter_domain().start())
                .unwrap(),
            Ordering::Greater
        );
        assert_eq!(
            parameter
                .compare(original.curves()[0].parameter_domain().end())
                .unwrap(),
            Ordering::Less
        );
        assert!(
            original.curves()[0]
                .point_at(&parameter)
                .unwrap()
                .coincides_with(&cut)
                .unwrap()
        );
        assert_eq!(export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        replay(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let original = source(true);
    let result = unique(&original, q(1, 64), q(1, 64));
    let chord = Curve2::try_line(original.curves()[0].end(), result.curves()[0].end()).unwrap();
    let text = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![CurvePath2::try_new(vec![chord]).unwrap()],
    ))
    .unwrap();
    replay(&text);
    let path = std::env::temp_dir().join(format!(
        "hypercurve-polynomial-chamfer-points-{}.svg",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "polynomial_chamfer_cut_points_replay_in_a_fresh_process",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn generated_polynomial_trim_reuses_independent_slab_incidence() {
    let edited = unique(&source(true), q(1, 64), q(1, 64));
    let upper = &edited.curves()[0];
    let selected = upper.polynomial_branch().unwrap();
    let independent = PolynomialCurveSet2::from_topology(
        selected.topology().clone(),
        selected.evaluation_config(),
    )
    .unwrap();
    let lower = &independent.curves()[0];
    assert!(!selected.shares_source(lower.polynomial_branch().unwrap()));
    let reflection = Similarity2::try_from_real_affine(
        0.into(),
        2.into(),
        2.into(),
        0.into(),
        3.into(),
        1.into(),
    )
    .unwrap();
    for reflected in [false, true] {
        for reverse_upper in [false, true] {
            for reverse_lower in [false, true] {
                let mut upper = upper.clone();
                let mut lower = lower.clone();
                let mut expected = CurvePoint2::from(Point2::from_values(-1, 0));
                if reflected {
                    upper = upper.transform_similarity(&reflection).unwrap();
                    lower = lower.transform_similarity(&reflection).unwrap();
                    expected = hypercurve::CurvePointDefinition2::Similarity {
                        source: expected,
                        transform: reflection.clone(),
                    }
                    .into_point()
                    .unwrap();
                }
                if reverse_upper {
                    upper = upper.reversed().unwrap();
                }
                if reverse_lower {
                    lower = lower.reversed().unwrap();
                }
                for swapped in [false, true] {
                    let (a, b) = if swapped {
                        (&lower, &upper)
                    } else {
                        (&upper, &lower)
                    };
                    let topology = a.intersection_topology(b).unwrap();
                    let result = topology.result();
                    assert!(result.is_complete() && result.overlaps().is_empty());
                    assert_eq!(result.contacts().len(), 1);
                    assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
                    let contact = &result.contacts()[0];
                    assert!(contact.point().coincides_with(&expected).unwrap());
                    // F_y=0 but F_x=-3 at (-1,0): one smooth vertical
                    // tangent on the original support, not a singular node.
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(hyperreal::RealSign::Zero)
                    );
                    assert!(!contact.is_certified_transverse());
                    for (curve, location) in [(a, contact.first()), (b, contact.second())] {
                        assert!(
                            curve
                                .point_at(&location.parameter().unwrap())
                                .unwrap()
                                .coincides_with(contact.point())
                                .unwrap()
                        );
                    }
                }
            }
        }
    }
}

#[cfg(feature = "svg")]
#[test]
fn trimmed_polynomial_svg_replays_without_an_untrimmed_viewport() {
    use hypercurve::{CurveRegion2, SvgGeometry2, export_svg_document, import_svg_document};
    const INPUT: &str = "HYPERCURVE_TRIMMED_POLYNOMIAL_SVG_REPLAY";
    fn replay(text: &str) {
        let geometry = import_svg_document(text).unwrap();
        let curve = &geometry.paths()[0].curves()[0];
        assert!(curve.polynomial_branch().is_some());
        assert_eq!(
            form(&curve.end(), polynomial(&[&[-1, 0, 1], &[], &[], &[-1]]))
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal
        );
        assert_eq!(export_svg_document(&geometry).unwrap(), text);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        replay(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let chamfer = unique(&source(true), q(1, 64), q(1, 64));
    let path = CurvePath2::try_new(vec![chamfer.curves()[0].clone()]).unwrap();
    let text = export_svg_document(&SvgGeometry2::new(
        CurveRegion2::empty(),
        vec![],
        vec![path],
    ))
    .unwrap();
    replay(&text);
    let file = std::env::temp_dir().join(format!(
        "hypercurve-trimmed-polynomial-{}.svg",
        std::process::id()
    ));
    std::fs::write(&file, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "trimmed_polynomial_svg_replays_without_an_untrimmed_viewport",
            "--nocapture",
        ])
        .env(INPUT, &file)
        .output()
        .unwrap();
    std::fs::remove_file(file).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
