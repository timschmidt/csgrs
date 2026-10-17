#[path = "support/closed_cone.rs"]
mod fixture;
use hyperbrep::*;
use hyperlimit::compare_reals;
use std::cmp::Ordering;

#[test]
fn a_closed_spatial_edge_does_not_authorize_missing_or_repeated_angular_coverage() {
    let tiny = (Real::one() / Real::from(1_000_000_000_000i64)).unwrap();
    for span in [
        Real::tau() - &tiny,
        Real::tau() + &tiny,
        Real::tau() * Real::from(2),
    ] {
        let error = fixture::with_span(Real::zero(), 0, false, span).unwrap_err();
        assert!(
            matches!(error, BuildError::EdgeUseSupportMismatch),
            "{error:?}"
        );
    }
    check(&fixture::build(Real::zero(), 0, false).unwrap());
}

fn check(model: &Model) {
    let equal = |a: Real, b: Real| {
        assert_eq!(
            compare_reals(&a, &b, STRICT_PREDICATES).value(),
            Some(Ordering::Equal)
        )
    };
    equal(
        model.solids().next().unwrap().volume().unwrap(),
        Real::from(78) * Real::pi(),
    );
    equal(
        model
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        (Real::from(29) + Real::from(21) * Real::from(5).sqrt().unwrap()) * Real::pi(),
    );
    let tiny = (Real::one() / Real::from(1_000_000_000_000i64)).unwrap();
    let radius = (Real::from(7) / Real::from(2)).unwrap();
    for (x, expected) in [
        (Real::zero(), SolidPointLocation::Inside),
        (radius.clone(), SolidPointLocation::Boundary),
        (&radius + &tiny, SolidPointLocation::Outside),
        (&radius - &tiny, SolidPointLocation::Inside),
    ] {
        assert_eq!(
            model
                .solids()
                .next()
                .unwrap()
                .classify_point(&Point3::new(x, 0.into(), 3.into()))
                .unwrap(),
            expected
        );
    }
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

#[test]
fn one_periodic_cone_face_preserves_complete_exact_frustum_and_replay() {
    for (angle, turns, reversed) in [
        (Real::zero(), 0, false),
        (
            (Real::from(4) / Real::from(3)).unwrap().atan().unwrap(),
            -2,
            true,
        ),
        (-Real::pi(), 3, false),
    ] {
        let model = fixture::build(angle, turns, reversed).unwrap();
        assert_eq!(
            (
                model.counts().vertices,
                model.counts().edges,
                model.counts().faces
            ),
            (2, 3, 3)
        );
        check(&model);
        let json = model.to_json().unwrap();
        let replay = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(replay.to_json().unwrap(), json);
        check(&replay);
        let body = BodySet::from_solid(replay.solids().next().unwrap());
        let canonical = builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap();
        check(boolean::union(&body, &canonical).unwrap().model());
        check(boolean::intersection(&body, &canonical).unwrap().model());
        assert_eq!(
            boolean::difference(&body, &canonical)
                .unwrap()
                .bodies()
                .count(),
            0
        );
        check(boolean::union(&BodySet::empty(), &body).unwrap().model());
    }
}

#[test]
fn independently_lifted_cone_bands_reunite_after_cold_archive_replay() {
    let load = |text: &str| {
        let model = RawModel::from_json(text, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        BodySet::from_solid(model.solids().next().unwrap())
    };
    let band = load(include_str!("fixtures/cone-imported-band.json"));
    let lower = load(include_str!("fixtures/cone-imported-lower.json"));
    for (a, b) in [(&band, &lower), (&lower, &band)] {
        let union = boolean::union(a, b).unwrap();
        let replay = load(&union.model().to_json().unwrap());
        let solid = replay.bodies().next().unwrap();
        assert_eq!(
            compare_reals(
                &solid.volume().unwrap(),
                &(Real::from(39) * Real::pi()),
                STRICT_PREDICATES
            )
            .value(),
            Some(Ordering::Equal)
        );
        let area = replay
            .model()
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b);
        let expected = ((Real::from(29) + Real::from(21) * Real::from(5).sqrt().unwrap())
            * Real::pi()
            / Real::from(2))
        .unwrap()
            + Real::from(42);
        assert_eq!(
            compare_reals(&area, &expected, STRICT_PREDICATES).value(),
            Some(Ordering::Equal)
        );
        for (x, y, expected) in [
            (1, 1, SolidPointLocation::Inside),
            (-1, -1, SolidPointLocation::Outside),
            (0, 0, SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                solid
                    .classify_point(&Point3::new(x.into(), y.into(), 3.into()))
                    .unwrap(),
                expected
            );
        }
        assert_eq!(
            boolean::difference(&replay, &replay)
                .unwrap()
                .bodies()
                .count(),
            0
        );
    }
}
