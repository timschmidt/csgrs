//! Disjoint plane/tensor sources remain disjoint after exact native replay.
use hyperbrep::*;

fn half(value: i32) -> Real {
    (Real::from(value) / Real::from(2)).unwrap()
}

fn graph(heights: [Real; 3]) -> Surface {
    let points = (0..3)
        .map(|j| {
            (0..3)
                .map(|i| Point3::new(half(i), half(j), heights[i as usize].clone()))
                .collect()
        })
        .collect();
    Surface::rational_bezier(points, vec![vec![Real::one(); 3]; 3]).unwrap()
}

fn check_model(model: &Model) {
    let surfaces = model.surfaces().map(|s| s.geometry()).collect::<Vec<_>>();
    assert_eq!(surfaces.len(), 2);
    assert!(matches!(
        surfaces[0].intersect_surface(surfaces[1]).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
    assert!(matches!(
        surfaces[1].intersect_surface(surfaces[0]).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
}

fn check_archive(archive: &str) {
    let model = RawModel::from_json(archive, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    check_model(&model);
    assert_eq!(model.to_json().unwrap(), archive);
    // This nonsingular affine map preserves disjointness, without assuming it
    // commutes with any Euclidean offset operation.
    let transform = Matrix4::from_row_major([
        1.into(),
        1.into(),
        0.into(),
        2.into(),
        0.into(),
        2.into(),
        0.into(),
        (-1).into(),
        1.into(),
        0.into(),
        1.into(),
        3.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let transformed = model.transformed(&transform).unwrap();
    check_model(&transformed);
    let bytes = transformed.to_json().unwrap();
    let replayed = RawModel::from_json(&bytes, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    check_model(&replayed);
    assert_eq!(replayed.to_json().unwrap(), bytes);
}

#[test]
fn empty_axis_sections_replay_in_a_fresh_process_and_intersect_again() {
    const CHILD: &str = "HYPERBREP_EMPTY_AXIS_SECTION_CHILD";
    if let Ok(archive) = std::env::var(CHILD) {
        check_archive(&archive);
        return;
    }
    // Quadratic Bernstein coefficients independently describe u^2+1, u-3,
    // and (u-2)^2. Their values never vanish on the full [0,1]^2 source box.
    for heights in [
        [1.into(), 1.into(), 2.into()],
        [(-3).into(), half(-5), (-2).into()],
        [4.into(), 2.into(), 1.into()],
    ] {
        let surface = graph(heights);
        let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        assert!(matches!(
            plane.intersect_surface(&surface).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        assert!(matches!(
            surface.intersect_surface(&plane).unwrap(),
            SurfaceSurfaceIntersection::None
        ));
        let mut builder = ModelBuilder::new();
        builder.surface(surface).unwrap();
        builder.surface(plane).unwrap();
        let model = builder.finish().unwrap();
        let archive = model.to_json().unwrap();
        let result = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "empty_axis_sections_replay_in_a_fresh_process_and_intersect_again",
                "--exact",
                "--nocapture",
            ])
            .env(CHILD, &archive)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{}\n{}",
            String::from_utf8_lossy(&result.stdout),
            String::from_utf8_lossy(&result.stderr)
        );
    }
}
