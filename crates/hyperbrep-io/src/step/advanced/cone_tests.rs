use super::*;

#[test]
fn shared_source_identity_never_aliases_opposite_native_cone_branches() {
    let text = include_str!("../../../tests/fixtures/cone-unit-line.step");
    let file =
        crate::step::StepFile::read(text.as_bytes(), &crate::step::StepReadOptions::default())
            .unwrap();
    let limits = crate::step::ImportLimits::default();
    let mut resolver = Resolver::new(&file, &limits);
    let mut builder = Builder {
        model: ModelBuilder::new(),
        by_vertex: HashMap::new(),
        by_edge: HashMap::new(),
        by_surface: HashMap::new(),
        by_revolution: HashMap::new(),
        angle_scale: Some(Real::one()),
        vertices: Vec::new(),
        edges: Vec::new(),
        faces: Vec::new(),
        loops: Vec::new(),
        edge_uses: Vec::new(),
    };
    let scale = (Real::one() / Real::from(1000)).unwrap();
    let source = builder
        .conical_source(&mut resolver, 12, &scale)
        .unwrap()
        .unwrap();
    let first = builder
        .surface(
            &mut resolver,
            12,
            &scale,
            Some(ConeBranch::IncreasingParameter),
        )
        .unwrap();
    let mut second = builder
        .surface(
            &mut resolver,
            12,
            &scale,
            Some(ConeBranch::DecreasingParameter),
        )
        .unwrap();
    assert_eq!(builder.by_surface.len(), 2);
    assert_eq!(builder.by_revolution.len(), 1);
    for (surface, branch, sign) in [
        (&first, ConeBranch::IncreasingParameter, 1),
        (&second, ConeBranch::DecreasingParameter, -1),
    ] {
        assert_eq!(surface.parameters.surface, 12);
        let surfaces::Support::Cone(chart) = &surface.support else {
            panic!("cone chart")
        };
        assert_eq!(chart.branch(), branch);
        let uv = hyperbrep::Point2::new(Real::zero(), Real::from(2) * &scale);
        let mapped = surface.parameters.map(&uv);
        assert_eq!(
            order(
                Site::entity(file.syntax().entity(12).unwrap()),
                &mapped.y,
                &Real::zero()
            )
            .unwrap(),
            if sign == 1 {
                Ordering::Greater
            } else {
                Ordering::Less
            }
        );
        let expected = source.generator().0.clone() + source.generator().1.clone() * mapped.y;
        let actual = chart.surface().point_at(&uv).unwrap();
        assert!(
            same_vector(
                Site::entity(file.syntax().entity(12).unwrap()),
                &(expected - Point3::origin()),
                &(actual - Point3::origin())
            )
            .unwrap()
        );
    }
    second.parameters.origin.y = Real::from(99);
    let again = builder
        .surface(
            &mut resolver,
            12,
            &scale,
            Some(ConeBranch::DecreasingParameter),
        )
        .unwrap();
    assert_eq!(again.parameters.origin.y, Real::zero());
    let model = builder.model.finish().unwrap();
    assert_ne!(
        model.surface(&first.key).unwrap(),
        model.surface(&second.key).unwrap()
    );
    assert_eq!(
        model.surface(&again.key).unwrap(),
        model.surface(&second.key).unwrap()
    );
}
