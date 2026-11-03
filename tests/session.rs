//! A modeling session keeps the weakest certainty of its whole chain.
//!
//! These are the U02/U03 acceptance chains of
//! `docs/research/simplification-and-api-program.md`: a strict-only chain, a
//! chain with one approximate step, and an approximate step followed by strict
//! steps. No later step or wrapper may upgrade the chain's certainty.
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::errors::ValidationErrorKind;
use csgrs::{Csg, Error, GeometryCertainty, GeometryContext, GeometryOutcome, Real, solid};
use hypermesh::BooleanOp;

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
fn strict_chain_is_certified_and_matches_the_free_functions() {
    let m = Csg::strict();
    let body = m.cube(12).unwrap();
    let tool = m.translate(&m.cube(4).unwrap(), 4, 4, -2).unwrap();
    let part = m.difference(&body, &tool).unwrap();
    assert_eq!(m.certainty(), GeometryCertainty::Certified);

    let expected_tool = solid::transform(
        &solid::cube(Real::from(4)).unwrap(),
        &hyperlattice::Matrix4::affine_translation([
            Real::from(4),
            Real::from(4),
            Real::from(-2),
        ]),
    )
    .unwrap();
    let expected = solid::boolean(
        &solid::cube(Real::from(12)).unwrap(),
        &expected_tool,
        BooleanOp::Difference,
    )
    .unwrap();
    assert_eq!(part, expected);
}

/// The unit cube and its copy translated by `1 + zero`, where strict
/// predicates cannot decide the zero.
fn touching_cubes(m: &Csg) -> (csgrs::TriangleMesh, csgrs::TriangleMesh) {
    let cube = m.cube(1).unwrap();
    let touching = m
        .translate(&cube, Real::one() + unresolved_zero(), 0, 0)
        .unwrap();
    (cube, touching)
}

#[test]
fn an_approximate_step_makes_the_whole_chain_approximate() {
    let m = Csg::approximate_512();
    let (cube, touching) = touching_cubes(&m);
    assert_eq!(m.certainty(), GeometryCertainty::Certified);
    let union = m.union(&cube, &touching).unwrap();
    assert_eq!(m.certainty(), GeometryCertainty::Approximate512Consumed);

    // Later steps, and steps that are certified on their own, cannot upgrade it.
    let moved = m.translate(&union, 5, 0, 0).unwrap();
    let _ = m.union(&moved, &m.cube(1).unwrap()).unwrap();
    m.record(GeometryOutcome {
        value: (),
        certainty: GeometryCertainty::Certified,
    });
    assert_eq!(m.certainty(), GeometryCertainty::Approximate512Consumed);
}

#[test]
fn strict_steps_after_an_approximate_input_keep_both_claims() {
    // An approximate mesh made in one session...
    let approximate = Csg::approximate_512();
    let (cube, touching) = touching_cubes(&approximate);
    let union = approximate.union(&cube, &touching).unwrap();

    // ...consumed by a strict session: the strict session's own decisions are
    // certified, and the mesh keeps its approximate ancestry.
    let strict = Csg::strict();
    let moved = strict.translate(&union, 5, 0, 0).unwrap();
    assert_eq!(strict.certainty(), GeometryCertainty::Certified);
    assert!(moved.construction_evidence().approximate_512_consumed());
    assert_eq!(
        approximate.certainty(),
        GeometryCertainty::Approximate512Consumed
    );
}

#[test]
fn a_strict_session_reports_the_undecided_step_without_recording_it() {
    let m = Csg::strict();
    let (cube, touching) = touching_cubes(&m);
    let error = m.union(&cube, &touching).unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
    assert_eq!(m.certainty(), GeometryCertainty::Certified);
}

#[test]
fn run_and_record_fold_external_receipts() {
    let m = Csg::new(GeometryContext::STRICT);
    let cube = m.cube(2).unwrap();
    let centered = m
        .run(|context| solid::center_with_context(&cube, context))
        .unwrap();
    assert!(!centered.triangles.is_empty());
    assert_eq!(m.certainty(), GeometryCertainty::Certified);
    assert_eq!(
        m.record(GeometryOutcome {
            value: 7,
            certainty: GeometryCertainty::Approximate512Consumed,
        }),
        7
    );
    assert_eq!(m.certainty(), GeometryCertainty::Approximate512Consumed);
}

#[test]
fn ordinary_numbers_enter_exactly() {
    use csgrs::{IntoReal, decimal, ratio};
    let half = ratio(1, 2).unwrap();
    assert_eq!(
        0.5_f64.into_real().unwrap().exact_rational_normal_form(),
        half.exact_rational_normal_form()
    );
    assert_eq!(
        3_u8.into_real().unwrap().exact_rational_normal_form(),
        Real::from(3).exact_rational_normal_form()
    );
    // 0.1_f64 is its binary value, not 1/10; decimal text is exact.
    let tenth = decimal("0.1").unwrap();
    assert_eq!(
        tenth.exact_rational_normal_form(),
        ratio(1, 10).unwrap().exact_rational_normal_form()
    );
    assert_ne!(
        0.1_f64.into_real().unwrap().exact_rational_normal_form(),
        tenth.exact_rational_normal_form()
    );
    assert_eq!(
        decimal("-3/7").unwrap().exact_rational_normal_form(),
        ratio(-3, 7).unwrap().exact_rational_normal_form()
    );

    for invalid in [
        f64::NAN.into_real(),
        f64::INFINITY.into_real(),
        f32::NEG_INFINITY.into_real(),
        ratio(1, 0),
        decimal("one tenth"),
    ] {
        let error = invalid.unwrap_err();
        assert!(matches!(error, Error::InvalidNumber(_)), "{error:?}");
        assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
        assert!(std::error::Error::source(&error).is_some());
    }

    // Invalid numbers fail the operation instead of producing geometry.
    let m = Csg::strict();
    assert!(matches!(
        m.cube(f64::NAN),
        Err(Error::InvalidNumber(hyperreal::Problem::NotANumber))
    ));
}

#[test]
fn solid_methods_cover_every_transform_and_match_the_free_functions() {
    use csgrs::solid::SolidExt;
    let mesh = solid::cuboid(Real::from(2), Real::from(3), Real::from(5)).unwrap();
    let plane = hypermesh::Plane::axis_aligned(0, Real::zero());
    assert_eq!(
        mesh.rotated(Real::from(90), Real::zero(), Real::from(45))
            .unwrap(),
        solid::rotate(&mesh, Real::from(90), Real::zero(), Real::from(45)).unwrap()
    );
    assert_eq!(
        mesh.scaled(Real::from(2), Real::one(), Real::from(3))
            .unwrap(),
        solid::scale(&mesh, Real::from(2), Real::one(), Real::from(3)).unwrap()
    );
    assert_eq!(
        mesh.mirrored(&plane).unwrap(),
        solid::mirror(&mesh, &plane).unwrap()
    );
    assert_eq!(mesh.centered().unwrap(), solid::center(&mesh).unwrap());
}
