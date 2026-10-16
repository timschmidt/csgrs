//! Exact one-edge query recipes and distinct failure outcomes.
use hyperbrep::*;
use hypercurve::{Curve2, CurveFamily2, LineSeg2, Point2 as Uv};

pub struct PreparedImage {
    builder: ModelBuilder,
    edge: EdgeKey,
    pcurve: PcurveKey,
    surface: SurfaceKey,
    correspondence: ParameterCorrespondence,
}
impl PreparedImage {
    pub fn check(&self) -> Result<(), BuildError> {
        self.builder.validate_edge_image(
            &self.edge,
            Direction::Forward,
            &self.pcurve,
            self.correspondence.clone(),
            &self.surface,
        )
    }
}

pub fn cylinder(radius: Real, phase: Real, span: Real) -> PreparedImage {
    let mut builder = ModelBuilder::new();
    let curve = Curve3::circle_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        radius.clone(),
        Real::zero(),
        Real::tau(),
    )
    .unwrap();
    let point = builder
        .vertex(Point3::new(radius, Real::zero(), Real::zero()).into())
        .unwrap();
    let curve = builder.curve(curve).unwrap();
    let edge = builder
        .edge(
            &point,
            &point,
            &curve,
            ParameterDomain::new(Real::zero(), Real::tau())
                .unwrap()
                .into(),
        )
        .unwrap();
    let pcurve = builder
        .pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(
                Uv::new(phase.clone(), Real::zero()),
                Uv::new(phase + span, Real::zero()),
            )
            .unwrap(),
        )))
        .unwrap();
    let surface = builder
        .surface(
            Surface::cylinder(
                Point3::origin(),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Real::one(),
            )
            .unwrap(),
        )
        .unwrap();
    PreparedImage {
        builder,
        edge,
        pcurve,
        surface,
        correspondence: ParameterCorrespondence::affine(Real::tau(), Real::zero()).unwrap(),
    }
}

pub fn parameter_and_radius_failures() {
    let gap = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    for (radius, phase, span) in [
        (Real::one() + gap, Real::zero(), Real::tau()),
        (Real::from(2), Real::zero(), Real::tau() * Real::from(2)),
        (Real::one(), Real::zero(), Real::tau() * Real::from(2)),
        (Real::one(), Real::zero(), -Real::tau()),
        (Real::one(), Real::pi(), Real::tau()),
    ] {
        let candidate = cylinder(radius, phase, span);
        for _ in 0..2 {
            assert_eq!(candidate.check(), Err(BuildError::EdgeUseSupportMismatch));
        }
    }
    cylinder(Real::one(), Real::zero(), Real::tau())
        .check()
        .unwrap();
}

pub fn unresolved_is_not_success() {
    // This exact zero is outside the current shared trigonometric proof
    // frontier. Keep strict reuse separate from terminal-policy evaluation.
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let phase = &sine * &sine + &cosine * &cosine - Real::one();
    let candidate = cylinder(Real::one(), phase, Real::tau());
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let result = hypercurve::evaluate_under(policy, || candidate.check());
        assert!(matches!(
            result.certified().unwrap(),
            Err(BuildError::Geometry(
                GeometryError::PredicateUnresolved { .. }
            ))
        ));
    }
    cylinder(Real::one(), Real::zero(), Real::tau())
        .check()
        .unwrap();
}

pub fn unsupported_is_not_endpoint_agreement() {
    let mut builder = ModelBuilder::new();
    let end = (Real::pi() / Real::from(2)).unwrap();
    let curve = Curve3::ellipse_arc(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Real::from(2),
        Real::one(),
        Real::zero(),
        end.clone(),
    )
    .unwrap();
    let start = builder
        .vertex(Point3::new(2.into(), 0.into(), 0.into()).into())
        .unwrap();
    let finish = builder
        .vertex(Point3::new(0.into(), 1.into(), 0.into()).into())
        .unwrap();
    let curve = builder.curve(curve).unwrap();
    let edge = builder
        .edge(
            &start,
            &finish,
            &curve,
            ParameterDomain::new(Real::zero(), end.clone())
                .unwrap()
                .into(),
        )
        .unwrap();
    let pcurve = builder
        .pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(Uv::new(2.into(), 0.into()), Uv::new(0.into(), 1.into())).unwrap(),
        )))
        .unwrap();
    let surface = builder
        .surface(Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap())
        .unwrap();
    assert_eq!(
        builder.validate_edge_image(
            &edge,
            Direction::Forward,
            &pcurve,
            ParameterCorrespondence::affine(end, Real::zero()).unwrap(),
            &surface
        ),
        Err(BuildError::UnsupportedEdgeUseAgreement {
            curve: Curve3Kind::EllipseArc,
            pcurve: CurveFamily2::Line,
            surface: SurfaceKind::Plane,
        })
    );
    assert!(matches!(
        builder.validate_edge_image(
            &edge,
            Direction::Forward,
            &pcurve,
            ParameterCorrespondence::identity(),
            &surface
        ),
        Err(BuildError::ParameterCorrespondenceMismatch { .. })
    ));
}
