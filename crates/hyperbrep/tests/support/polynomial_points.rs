#[path = "archive_graph.rs"]
mod archive_graph;
use hyperbrep::{
    ArchiveLimitExceeded, ArchiveLimits, ArchiveResource, BuildError, GeometryError, ModelBuilder,
    Pcurve, PersistenceError, RawModel, Real,
};
use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Curve2, CurveError, CurvePointDefinition2, ExactCurveError, Point2,
};
use hypersolve::{BivariatePolynomial, IsolatedRootInterval};

pub fn polynomial_point_archive_admission_and_replay() {
    let point = CurvePointDefinition2::PolynomialFiber {
        equation: BivariatePolynomial::new(vec![
            vec![0.into(), 0.into(), 0.into(), 1.into()],
            vec![(-1).into()],
        ]),
        parameter: BezierParameter2::Algebraic(
            BezierAlgebraicParameter2::try_isolate(
                BezierParameterPolynomial::try_new_power_basis(vec![
                    (-2).into(),
                    0.into(),
                    1.into(),
                ])
                .unwrap(),
                BezierParameterInterval::try_new(1.into(), 2.into()).unwrap(),
            )
            .unwrap(),
        )
        .into(),
        root: IsolatedRootInterval {
            lower: 1.into(),
            upper: (Real::from(3) / Real::from(2)).unwrap(),
            exact_root: None,
            distinct_root_count: 1,
        },
    }
    .into_point()
    .unwrap();
    let end = Point2::from_values(2, 2).into();
    let curve = Curve2::try_line(point.clone(), end).unwrap();
    let mut builder = ModelBuilder::new();
    builder.pcurve(Pcurve::new(curve)).unwrap();
    let json = builder.finish().unwrap().to_json().unwrap();
    // Four graph definitions, two chord endpoint references and one pcurve
    // reference make seven records. The
    // polynomial stores two row headers and five coefficients; the base adds
    // three coefficients. These limits apply before scalar decoding.
    let exact = ArchiveLimits {
        records: 7,
        degree: 3,
        geometry_elements: 10,
        topology_references: 0,
        ..ArchiveLimits::default()
    };
    let replay = || {
        let cold = RawModel::from_json(&json, &exact)
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(cold.to_json().unwrap(), json);
        let pcurve = cold.pcurves().next().unwrap();
        let restored = pcurve.curve();
        assert!(restored.start().coincides_with(&point).unwrap());
        assert!(
            restored
                .reversed()
                .unwrap()
                .end()
                .coincides_with(&point)
                .unwrap()
        );
    };
    replay();
    for (limits, resource, expected) in [
        (
            ArchiveLimits {
                records: 6,
                ..exact
            },
            ArchiveResource::Records,
            7,
        ),
        (
            ArchiveLimits { degree: 2, ..exact },
            ArchiveResource::Degree,
            3,
        ),
        (
            ArchiveLimits {
                geometry_elements: 9,
                ..exact
            },
            ArchiveResource::GeometryElements,
            10,
        ),
    ] {
        assert!(matches!(
            RawModel::from_json(&json, &limits),
            Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded { resource: actual, observed, .. }))
                if actual == resource && observed == expected
        ));
        replay();
    }
    let raw: serde_json::Value = serde_json::from_str(&json).unwrap();
    for axis in 0..2 {
        let mut malformed = raw.clone();
        let equation = &mut archive_graph::at_mut(
            &mut malformed,
            "/pcurves/0/Chord/start/PolynomialFiber",
        )["equation"];
        *equation = if axis == 0 {
            serde_json::json!([["invalid scalar"], [], [], [], []])
        } else {
            serde_json::json!([["invalid scalar", "0", "0", "0", "0"]])
        };
        assert!(matches!(
            RawModel::from_json(&malformed.to_string(), &exact),
            Err(PersistenceError::ResourceLimit(ArchiveLimitExceeded {
                resource: ArchiveResource::Degree,
                observed: 4,
                limit: 3
            }))
        ));
    }
    for forged in [false, true] {
        let mut bad = raw.clone();
        let definition = archive_graph::at_mut(&mut bad, "/pcurves/0/Chord/start/PolynomialFiber");
        if forged {
            definition["exact_root"] = serde_json::to_value(Real::one()).unwrap();
        } else {
            definition["interval"] = serde_json::json!([Real::from(3), Real::from(4)]);
        }
        assert!(matches!(
            RawModel::from_json(&bad.to_string(), &exact)
                .unwrap()
                .validate(),
            Err(PersistenceError::Build(BuildError::Geometry(
                GeometryError::PlanarCurve(ExactCurveError::Invalid {
                    cause: CurveError::InvalidBezierAlgebraicParameter,
                    ..
                })
            )))
        ));
        replay();
    }
}
