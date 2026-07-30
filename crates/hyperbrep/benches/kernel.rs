use std::hint::black_box;
use std::time::Instant;

use hyperbrep::{
    Curve3, Direction, Model, Point3, RawModel, Real, SolidPointLocation, Surface,
    SurfaceIntersectionOperand, SurfaceSurfaceIntersection, Vector3, boolean, builder,
};
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2 as CurvePoint2, QuadraticBezier2};
use hyperlimit::compare_reals;

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn directed_start(model: &Model, edge_use: hyperbrep::EdgeUseId) -> hyperbrep::VertexId {
    let edge_use = model.edge_use(edge_use).expect("validated edge use");
    let edge = model.edge(edge_use.edge()).expect("validated edge");
    match edge_use.direction() {
        Direction::Forward => edge.start(),
        Direction::Reversed => edge.end(),
    }
}

fn main() {
    const ITERATIONS: usize = 1_000;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..ITERATIONS {
        let (model, solid) =
            builder::cuboid(black_box(point(-2, -3, -5)), black_box(point(7, 11, 13)))
                .expect("benchmark cuboid");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &Real::from(2_268)).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(0, 0, 0))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "planar_kernel/cuboid_build_measure_classify: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32,
    );

    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..ITERATIONS {
        let (model, solid) = builder::cylinder(black_box(Real::from(2)), black_box(Real::from(3)))
            .expect("benchmark cylinder");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(12) * Real::pi())).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(0, 0, 1))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/cylinder_build_measure_classify: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32,
    );

    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..ITERATIONS {
        let (model, solid) = builder::sphere(black_box(Real::from(3))).expect("benchmark sphere");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(36) * Real::pi())).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(0, 0, 0))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/sphere_build_measure_classify: {ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / ITERATIONS as u32,
    );

    const FRUSTUM_ITERATIONS: usize = 100;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..FRUSTUM_ITERATIONS {
        let (model, solid) = builder::cone_frustum(
            black_box(Real::from(2)),
            black_box(Real::one()),
            black_box(Real::from(3)),
        )
        .expect("benchmark cone frustum");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(7) * Real::pi())).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(0, 0, 1))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/cone_frustum_build_measure_classify: {FRUSTUM_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / FRUSTUM_ITERATIONS as u32,
    );

    const TORUS_ITERATIONS: usize = 250;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TORUS_ITERATIONS {
        let (model, solid) = builder::torus(black_box(Real::from(3)), black_box(Real::one()))
            .expect("benchmark torus");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(6) * Real::pi() * Real::pi()),).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(3, 0, 0))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/torus_build_measure_classify: {TORUS_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TORUS_ITERATIONS as u32,
    );

    const REVOLUTION_ITERATIONS: usize = 100;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..REVOLUTION_ITERATIONS {
        let profile = [
            hyperbrep::Point2::new(Real::one(), Real::zero()),
            hyperbrep::Point2::new(Real::from(3), Real::zero()),
            hyperbrep::Point2::new(Real::from(3), Real::from(2)),
            hyperbrep::Point2::new(Real::one(), Real::from(2)),
        ];
        let (model, solid) = builder::revolve(black_box(&profile)).expect("benchmark revolution");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(16) * Real::pi())).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(2, 0, 1))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/revolve_build_measure_classify: {REVOLUTION_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / REVOLUTION_ITERATIONS as u32,
    );

    const CURVED_REVOLUTION_ITERATIONS: usize = 100;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..CURVED_REVOLUTION_ITERATIONS {
        let center = hypercurve::Point2::new(Real::from(3), Real::zero());
        let profile = hypercurve::Contour2::try_new(vec![
            hypercurve::Segment2::Arc(
                hypercurve::CircularArc2::try_from_center(
                    hypercurve::Point2::new(Real::from(4), Real::zero()),
                    hypercurve::Point2::new(Real::from(2), Real::zero()),
                    center.clone(),
                    false,
                )
                .expect("benchmark upper profile arc"),
            ),
            hypercurve::Segment2::Arc(
                hypercurve::CircularArc2::try_from_center(
                    hypercurve::Point2::new(Real::from(2), Real::zero()),
                    hypercurve::Point2::new(Real::from(4), Real::zero()),
                    center,
                    false,
                )
                .expect("benchmark lower profile arc"),
            ),
        ])
        .expect("benchmark profile contour");
        let (model, solid) =
            builder::revolve_contour(black_box(&profile)).expect("benchmark curved revolution");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(6) * Real::pi() * Real::pi()),).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(3, 0, 0))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/line_arc_profile_revolution_build_measure_classify: {CURVED_REVOLUTION_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / CURVED_REVOLUTION_ITERATIONS as u32,
    );

    const SWEEP_ITERATIONS: usize = 250;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SWEEP_ITERATIONS {
        let profile = [
            hyperbrep::Point2::new(Real::zero(), Real::zero()),
            hyperbrep::Point2::new(Real::from(2), Real::zero()),
            hyperbrep::Point2::new(Real::from(2), Real::from(3)),
            hyperbrep::Point2::new(Real::zero(), Real::from(3)),
        ];
        let (model, solid) = builder::sweep(
            black_box(&profile),
            Point3::origin(),
            hyperbrep::Vector3::from_xyz(Real::from(2), Real::zero(), Real::zero()),
            hyperbrep::Vector3::from_xyz(Real::zero(), Real::from(3), Real::zero()),
            hyperbrep::Vector3::from_xyz(Real::one(), Real::zero(), Real::from(4)),
        )
        .expect("benchmark linear sweep");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &Real::from(144)).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(
                    solid,
                    &Point3::new(
                        (Real::from(5) / Real::from(2)).unwrap(),
                        Real::from(3),
                        Real::from(2),
                    ),
                )
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/linear_sweep_build_measure_classify: {SWEEP_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SWEEP_ITERATIONS as u32,
    );

    const CURVED_SWEEP_ITERATIONS: usize = 100;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..CURVED_SWEEP_ITERATIONS {
        let profile = [
            hyperbrep::Point2::new(Real::zero(), Real::zero()),
            hyperbrep::Point2::new(Real::from(2), Real::zero()),
            hyperbrep::Point2::new(Real::from(2), Real::from(2)),
            hyperbrep::Point2::new(Real::zero(), Real::from(2)),
        ];
        let path = Curve3::rational_bezier(
            vec![point(0, 0, 0), point(1, 0, 1), point(0, 0, 4)],
            vec![Real::one(), Real::from(2), Real::from(3)],
        )
        .expect("benchmark curved sweep path");
        let (model, solid) = builder::sweep_curve(
            black_box(&profile),
            Vector3::x(),
            Vector3::y(),
            black_box(path),
        )
        .expect("benchmark curved sweep");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &Real::from(16)).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(
                    solid,
                    &Point3::new(
                        (Real::from(3) / Real::from(2)).unwrap(),
                        Real::one(),
                        Real::from(2),
                    ),
                )
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/fixed_frame_curved_sweep_build_measure_classify: {CURVED_SWEEP_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / CURVED_SWEEP_ITERATIONS as u32,
    );

    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..CURVED_SWEEP_ITERATIONS {
        let profile = [
            hyperbrep::Point2::new(Real::zero(), Real::zero()),
            hyperbrep::Point2::new(Real::from(2), Real::zero()),
            hyperbrep::Point2::new(Real::from(2), Real::from(2)),
            hyperbrep::Point2::new(Real::zero(), Real::from(2)),
        ];
        let frame = hyperbrep::RationalBezierSweepFrame::try_new(
            vec![point(0, 0, 0), point(0, 0, 3)],
            vec![
                Vector3::x(),
                Vector3::from_xyz(Real::from(2), Real::zero(), Real::zero()),
            ],
            vec![
                Vector3::y(),
                Vector3::from_xyz(Real::zero(), Real::from(2), Real::zero()),
            ],
            vec![Real::one(), Real::one()],
        )
        .expect("benchmark moving sweep frame");
        let (model, solid) = builder::sweep_moving_frame(black_box(&profile), black_box(frame))
            .expect("benchmark moving-frame sweep");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &Real::from(28)).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(
                    solid,
                    &Point3::new(
                        (Real::from(3) / Real::from(2)).unwrap(),
                        (Real::from(3) / Real::from(2)).unwrap(),
                        (Real::from(3) / Real::from(2)).unwrap(),
                    ),
                )
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/moving_frame_taper_build_measure_classify: {CURVED_SWEEP_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / CURVED_SWEEP_ITERATIONS as u32,
    );

    const LOFT_ITERATIONS: usize = 250;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..LOFT_ITERATIONS {
        let sections = [
            hyperbrep::LoftSection {
                profile: vec![
                    hyperbrep::Point2::new(Real::zero(), Real::zero()),
                    hyperbrep::Point2::new(Real::from(2), Real::zero()),
                    hyperbrep::Point2::new(Real::from(2), Real::from(2)),
                    hyperbrep::Point2::new(Real::zero(), Real::from(2)),
                ],
                z: Real::zero(),
            },
            hyperbrep::LoftSection {
                profile: vec![
                    hyperbrep::Point2::new(Real::one(), Real::one()),
                    hyperbrep::Point2::new(Real::from(5), Real::one()),
                    hyperbrep::Point2::new(Real::from(5), Real::from(5)),
                    hyperbrep::Point2::new(Real::one(), Real::from(5)),
                ],
                z: Real::from(3),
            },
        ];
        let (model, solid) = builder::loft(black_box(&sections)).expect("benchmark loft");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &Real::from(28)).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(2, 2, 1))
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "analytic_kernel/homothetic_loft_build_measure_classify: {LOFT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / LOFT_ITERATIONS as u32,
    );

    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..LOFT_ITERATIONS {
        let sections = [
            hyperbrep::LoftSection {
                profile: vec![
                    hyperbrep::Point2::new(Real::zero(), Real::zero()),
                    hyperbrep::Point2::new(Real::from(2), Real::zero()),
                    hyperbrep::Point2::new(Real::from(2), Real::from(2)),
                    hyperbrep::Point2::new(Real::zero(), Real::from(2)),
                ],
                z: Real::zero(),
            },
            hyperbrep::LoftSection {
                profile: vec![
                    hyperbrep::Point2::new(Real::one(), Real::one()),
                    hyperbrep::Point2::new(Real::from(5), Real::one()),
                    hyperbrep::Point2::new(Real::from(4), Real::from(5)),
                    hyperbrep::Point2::new(Real::one(), Real::from(5)),
                ],
                z: Real::from(3),
            },
        ];
        let (model, solid) =
            builder::loft(black_box(&sections)).expect("benchmark convex corresponding loft");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(51) / Real::from(2)).unwrap()).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(
                    solid,
                    &Point3::new(
                        Real::one(),
                        Real::one(),
                        (Real::from(3) / Real::from(2)).unwrap(),
                    ),
                )
                .expect("benchmark classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/convex_corresponding_bilinear_loft_build_measure_classify: {LOFT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / LOFT_ITERATIONS as u32,
    );

    const MULTI_LOFT_ITERATIONS: usize = 100;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..MULTI_LOFT_ITERATIONS {
        let sections = [
            hyperbrep::LoftSection {
                profile: vec![
                    hyperbrep::Point2::new(Real::zero(), Real::zero()),
                    hyperbrep::Point2::new(Real::from(2), Real::zero()),
                    hyperbrep::Point2::new(Real::from(2), Real::from(2)),
                    hyperbrep::Point2::new(Real::zero(), Real::from(2)),
                ],
                z: Real::zero(),
            },
            hyperbrep::LoftSection {
                profile: vec![
                    hyperbrep::Point2::new(Real::one(), Real::one()),
                    hyperbrep::Point2::new(Real::from(5), Real::one()),
                    hyperbrep::Point2::new(Real::from(5), Real::from(5)),
                    hyperbrep::Point2::new(Real::one(), Real::from(5)),
                ],
                z: Real::from(2),
            },
            hyperbrep::LoftSection {
                profile: vec![
                    hyperbrep::Point2::new(Real::zero(), Real::zero()),
                    hyperbrep::Point2::new(Real::from(6), Real::zero()),
                    hyperbrep::Point2::new(Real::from(6), Real::from(3)),
                    hyperbrep::Point2::new(Real::zero(), Real::from(3)),
                ],
                z: Real::from(5),
            },
        ];
        let (model, solid) =
            builder::loft(black_box(&sections)).expect("benchmark multi-section loft");
        let volume = model.solid_volume(solid).expect("benchmark volume");
        assert_eq!(
            compare_reals(&volume, &(Real::from(212) / Real::from(3)).unwrap()).value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += usize::from(
            model
                .classify_point(solid, &point(2, 2, 2))
                .expect("benchmark seam classification")
                == SolidPointLocation::Inside,
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/multi_section_c0_loft_build_measure_classify: {MULTI_LOFT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / MULTI_LOFT_ITERATIONS as u32,
    );

    const EDIT_ITERATIONS: usize = 250;
    let (edit_source, solid) =
        builder::cuboid(point(0, 0, 0), point(2, 2, 2)).expect("benchmark edit source");
    let (edge, edge_record) = edit_source.edges().next().expect("cuboid has edges");
    let edge_midpoint = ((edge_record.domain().start() + edge_record.domain().end())
        / Real::from(2))
    .expect("two is nonzero");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..EDIT_ITERATIONS {
        let (edited, _) = edit_source
            .split_edge(edge, black_box(edge_midpoint.clone()))
            .expect("benchmark edge split");
        checksum += usize::from(
            compare_reals(
                &edited.solid_volume(solid).expect("edited volume"),
                &Real::from(8),
            )
            .value()
                == Some(std::cmp::Ordering::Equal),
        );
        black_box(edited);
    }
    let elapsed = started.elapsed();
    println!(
        "editing_kernel/cuboid_edge_split_revalidate: {EDIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / EDIT_ITERATIONS as u32,
    );

    let (face, face_record) = edit_source.faces().nth(1).expect("cuboid has top cap");
    let outer = face_record.outer().expect("top cap is trimmed");
    let uses = edit_source
        .wire(outer)
        .expect("validated cap wire")
        .edge_uses();
    let start = directed_start(&edit_source, uses[0]);
    let end = directed_start(&edit_source, uses[2]);
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..EDIT_ITERATIONS {
        let (edited, _) = edit_source
            .split_face(face, black_box(start), black_box(end))
            .expect("benchmark face split");
        checksum += usize::from(
            compare_reals(
                &edited.solid_volume(solid).expect("edited volume"),
                &Real::from(8),
            )
            .value()
                == Some(std::cmp::Ordering::Equal),
        );
        black_box(edited);
    }
    let elapsed = started.elapsed();
    println!(
        "editing_kernel/cuboid_face_split_revalidate: {EDIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / EDIT_ITERATIONS as u32,
    );

    let edge_midpoint = |use_id| {
        let edge = edit_source
            .edge(
                edit_source
                    .edge_use(use_id)
                    .expect("validated cap use")
                    .edge(),
            )
            .expect("validated cap edge");
        let parameter = ((edge.domain().start() + edge.domain().end()) / Real::from(2))
            .expect("two is nonzero");
        edit_source
            .curve(edge.curve())
            .expect("validated cap curve")
            .point_at(&parameter)
            .expect("edge midpoint evaluates")
    };
    let trace =
        Curve3::line(edge_midpoint(uses[0]), edge_midpoint(uses[2])).expect("planar split trace");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..EDIT_ITERATIONS {
        let (edited, _) = edit_source
            .split_face_by_curve(face, black_box(&trace))
            .expect("benchmark curve-driven face split");
        checksum += usize::from(
            compare_reals(
                &edited.solid_volume(solid).expect("edited volume"),
                &Real::from(8),
            )
            .value()
                == Some(std::cmp::Ordering::Equal),
        );
        black_box(edited);
    }
    let elapsed = started.elapsed();
    println!(
        "editing_kernel/cuboid_curve_face_split_revalidate: {EDIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / EDIT_ITERATIONS as u32,
    );

    let directed_point = |use_id, numerator: i32| {
        let edge_use = edit_source
            .edge_use(use_id)
            .expect("validated cap edge use");
        let edge = edit_source
            .edge(edge_use.edge())
            .expect("validated cap edge");
        let fraction =
            (Real::from(numerator) / Real::from(3)).expect("three is a nonzero denominator");
        let span = edge.domain().end() - edge.domain().start();
        let offset = span * fraction;
        let parameter = match edge_use.direction() {
            Direction::Forward => edge.domain().start() + offset,
            Direction::Reversed => edge.domain().end() - offset,
        };
        edit_source
            .curve(edge.curve())
            .expect("validated cap curve")
            .point_at(&parameter)
            .expect("directed edge fraction evaluates")
    };
    let trace_at = |numerator: i32| {
        Curve3::line(
            directed_point(uses[0], numerator),
            directed_point(uses[2], 3 - numerator),
        )
        .expect("opposite edge fractions define a split trace")
    };
    let traces = [trace_at(2), trace_at(1)];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..EDIT_ITERATIONS {
        let (edited, partition) = edit_source
            .split_face_by_curves(face, black_box(&traces))
            .expect("benchmark deterministic multi-trace face partition");
        checksum += partition.faces.len();
        assert_eq!(
            compare_reals(
                &edited.solid_volume(solid).expect("partitioned volume"),
                &Real::from(8),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        black_box(edited);
    }
    let elapsed = started.elapsed();
    println!(
        "editing_kernel/cuboid_multi_trace_face_partition_revalidate: {EDIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / EDIT_ITERATIONS as u32,
    );

    let vertex_point = |use_id| {
        edit_source
            .vertex(directed_start(&edit_source, use_id))
            .expect("validated cap vertex")
            .point()
            .clone()
    };
    let crossing_traces = [
        Curve3::line(vertex_point(uses[0]), vertex_point(uses[2])).expect("first cap diagonal"),
        Curve3::line(vertex_point(uses[1]), vertex_point(uses[3])).expect("second cap diagonal"),
    ];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..EDIT_ITERATIONS {
        let (edited, partition) = edit_source
            .split_face_by_curves(face, black_box(&crossing_traces))
            .expect("benchmark exact crossing-trace arrangement");
        checksum += partition.faces.len();
        assert_eq!(
            compare_reals(
                &edited.solid_volume(solid).expect("arranged volume"),
                &Real::from(8),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        black_box(edited);
    }
    let elapsed = started.elapsed();
    println!(
        "editing_kernel/cuboid_crossing_trace_arrangement_revalidate: {EDIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / EDIT_ITERATIONS as u32,
    );

    let (tensor_cap_source, tensor_cap_solid) =
        builder::cuboid(point(0, 0, 0), point(1, 1, 1)).expect("tensor-cap benchmark source");
    let tensor_cap_surface = tensor_cap_source
        .faces()
        .find_map(|(_, face)| {
            let surface = tensor_cap_source
                .surface(face.surface())
                .expect("validated benchmark surface");
            let origin = surface
                .point_at(&hyperbrep::Point2::new(Real::zero(), Real::zero()))
                .ok()?;
            (surface.kind() == hyperbrep::SurfaceKind::Plane
                && compare_reals(&origin.z, &Real::one()).value()
                    == Some(std::cmp::Ordering::Equal))
            .then_some(face.surface())
        })
        .expect("unit cuboid has an upper plane");
    let tensor_cap = Surface::rational_bezier(
        vec![
            vec![point(0, 0, 1), point(1, 0, 1)],
            vec![point(0, 1, 1), point(1, 1, 1)],
        ],
        vec![vec![Real::one(), Real::one()]; 2],
    )
    .expect("benchmark affine tensor cap");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..EDIT_ITERATIONS {
        let mut edit = tensor_cap_source.edit();
        edit.replace_surface(tensor_cap_surface, black_box(tensor_cap.clone()))
            .expect("benchmark cap replacement");
        let edited = edit.commit().expect("benchmark tensor-cap certificate");
        checksum += usize::from(
            compare_reals(
                &edited
                    .solid_volume(tensor_cap_solid)
                    .expect("tensor-cap volume"),
                &Real::one(),
            )
            .value()
                == Some(std::cmp::Ordering::Equal),
        );
        black_box(edited);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/affine_tensor_cap_revalidate_measure: {EDIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / EDIT_ITERATIONS as u32,
    );

    const SPLINE_ITERATIONS: usize = 500;
    let controls = vec![
        vec![point(0, 0, 0), point(1, 0, 1), point(2, 0, 0)],
        vec![point(0, 1, 1), point(1, 1, 2), point(2, 1, 1)],
        vec![point(0, 2, 0), point(1, 2, 1), point(2, 2, 0)],
    ];
    let weights = vec![
        vec![Real::one(), Real::from(2), Real::one()],
        vec![Real::one(), Real::from(3), Real::one()],
        vec![Real::one(), Real::from(2), Real::one()],
    ];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let (model, _) =
            builder::rational_bezier_patch(black_box(controls.clone()), black_box(weights.clone()))
                .expect("benchmark rational Bézier patch");
        checksum += model.counts().faces;
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/rational_bezier_patch_build_validate: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let knots = vec![
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::one(),
        Real::one(),
    ];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let (model, _) = builder::nurbs_patch(
            2,
            2,
            black_box(controls.clone()),
            black_box(weights.clone()),
            black_box(knots.clone()),
            black_box(knots.clone()),
        )
        .expect("benchmark NURBS patch");
        checksum += model.counts().faces;
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/nurbs_patch_build_validate: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let curve_point = |x, y| CurvePoint2::new(Real::from(x), Real::from(y));
    let line = |x0, y0, x1, y1| {
        Curve2::from(
            LineSeg2::try_new(curve_point(x0, y0), curve_point(x1, y1))
                .expect("benchmark planar line"),
        )
    };
    let planar_outer = CurvePath2::try_new(vec![
        Curve2::try_nurbs(
            2,
            vec![curve_point(0, 0), curve_point(2, 0), curve_point(4, 0)],
            vec![Real::one(), Real::from(2), Real::from(3)],
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                Real::from(5),
                Real::from(5),
                Real::from(5),
            ],
        )
        .expect("benchmark planar NURBS boundary"),
        line(4, 0, 4, 4),
        line(4, 4, 0, 4),
        line(0, 4, 0, 0),
    ])
    .expect("benchmark planar outer path");
    let planar_hole = CurvePath2::try_new(vec![
        line(1, 1, 2, 1),
        line(2, 1, 2, 2),
        line(2, 2, 1, 2),
        line(1, 2, 1, 1),
    ])
    .expect("benchmark planar hole path");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let (model, face) = builder::planar_face(
            black_box(&planar_outer),
            black_box(std::slice::from_ref(&planar_hole)),
            black_box(point(5, -2, 7)),
            black_box(Vector3::from_xyz(Real::from(2), Real::zero(), Real::zero())),
            black_box(Vector3::from_xyz(Real::one(), Real::from(3), Real::zero())),
        )
        .expect("benchmark exact planar face");
        let area = model
            .face_area(face)
            .expect("benchmark exact planar spline region area");
        checksum += usize::from(
            compare_reals(&area, &Real::from(90)).value() == Some(std::cmp::Ordering::Equal),
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/nurbs_planar_face_build_exact_area: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let (model, solid) = builder::extrude_path_region(
            black_box(&planar_outer),
            black_box(std::slice::from_ref(&planar_hole)),
            black_box(-Real::one()),
            black_box(Real::from(2)),
        )
        .expect("benchmark exact path extrusion");
        let volume = model
            .solid_volume(solid)
            .expect("benchmark exact path extrusion volume");
        checksum += usize::from(
            compare_reals(&volume, &Real::from(45)).value() == Some(std::cmp::Ordering::Equal),
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/nurbs_path_extrusion_build_exact_volume: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let extrusion_profile = Curve3::nurbs(
        2,
        vec![point(0, 0, 0), point(2, 1, 0), point(0, 2, 0)],
        vec![Real::one(), Real::from(2), Real::from(3)],
        vec![
            Real::from(2),
            Real::from(2),
            Real::from(2),
            Real::from(5),
            Real::from(5),
            Real::from(5),
        ],
    )
    .expect("benchmark planar NURBS extrusion profile");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let (model, face) = builder::extrusion_patch(
            black_box(extrusion_profile.clone()),
            black_box(Vector3::x()),
            black_box(-Real::one()),
            black_box(Real::from(2)),
        )
        .expect("benchmark exact extrusion patch");
        let area = model
            .face_area(face)
            .expect("benchmark exact planar spline extrusion area");
        checksum += usize::from(
            compare_reals(&area, &Real::from(6)).value() == Some(std::cmp::Ordering::Equal),
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/nurbs_extrusion_patch_build_exact_area: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let revolution_profile = Curve3::nurbs(
        2,
        vec![point(2, 0, 0), point(3, 0, 1), point(4, 0, 2)],
        vec![Real::one(), Real::from(2), Real::from(3)],
        vec![
            Real::from(2),
            Real::from(2),
            Real::from(2),
            Real::from(5),
            Real::from(5),
            Real::from(5),
        ],
    )
    .expect("benchmark NURBS revolution profile");
    let quarter = (Real::pi() / Real::from(2)).expect("two is nonzero");
    let expected_revolution_area = Real::from(3)
        * Real::pi()
        * Real::from(2)
            .sqrt()
            .expect("positive integer has an exact square root expression");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let (model, face) = builder::revolution_patch(
            black_box(revolution_profile.clone()),
            black_box(Point3::origin()),
            black_box(Vector3::z()),
            black_box(Real::zero()),
            black_box(quarter.clone()),
        )
        .expect("benchmark exact revolution patch");
        let area = model
            .face_area(face)
            .expect("benchmark exact rational line-image revolution area");
        checksum += usize::from(
            compare_reals(&area, &expected_revolution_area).value()
                == Some(std::cmp::Ordering::Equal),
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/nurbs_revolution_patch_build_exact_area: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    const TENSOR_AREA_ITERATIONS: usize = 1_000;
    let affine_controls = vec![
        vec![point(0, 0, 0), point(2, 0, 0), point(4, 0, 0)],
        vec![point(0, 3, 0), point(2, 3, 0), point(4, 3, 0)],
        vec![point(0, 6, 0), point(2, 6, 0), point(4, 6, 0)],
    ];
    let separable_weights = vec![
        vec![Real::from(2), Real::from(4), Real::from(6)],
        vec![Real::from(5), Real::from(10), Real::from(15)],
        vec![Real::from(7), Real::from(14), Real::from(21)],
    ];
    let (affine_bezier, affine_bezier_face) =
        builder::rational_bezier_patch(affine_controls.clone(), separable_weights.clone())
            .expect("benchmark separably parameterized Bézier patch");
    let native_knots = vec![
        Real::from(2),
        Real::from(2),
        Real::from(2),
        Real::from(5),
        Real::from(5),
        Real::from(5),
    ];
    let (affine_nurbs, affine_nurbs_face) = builder::nurbs_patch(
        2,
        2,
        affine_controls,
        separable_weights,
        native_knots.clone(),
        native_knots,
    )
    .expect("benchmark separably parameterized NURBS patch");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_AREA_ITERATIONS {
        for (model, face) in [
            (&affine_bezier, affine_bezier_face),
            (&affine_nurbs, affine_nurbs_face),
        ] {
            let area = black_box(model)
                .face_area(face)
                .expect("benchmark exact tensor area");
            checksum += usize::from(
                compare_reals(&area, &Real::from(24)).value() == Some(std::cmp::Ordering::Equal),
            );
            black_box(area);
        }
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/separable_affine_tensor_exact_area: {TENSOR_AREA_ITERATIONS} paired iterations in {elapsed:?} ({:?}/pair), checksum={checksum}",
        elapsed / TENSOR_AREA_ITERATIONS as u32,
    );

    let stitched_specs = vec![
        hyperbrep::TensorPatch::RationalBezier {
            control_points: vec![
                vec![point(0, 0, 0), point(1, 0, 0)],
                vec![point(0, 1, 0), point(1, 1, 1)],
            ],
            weights: vec![
                vec![Real::one(), Real::one()],
                vec![Real::one(), Real::from(2)],
            ],
        },
        hyperbrep::TensorPatch::RationalBezier {
            control_points: vec![
                vec![point(1, 0, 0), point(2, 0, 0)],
                vec![point(1, 1, 1), point(2, 1, 0)],
            ],
            weights: vec![
                vec![Real::from(3), Real::one()],
                vec![Real::from(6), Real::one()],
            ],
        },
    ];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let (model, faces) = builder::tensor_patch_shell(black_box(stitched_specs.clone()))
            .expect("benchmark stitched tensor shell");
        checksum += faces.len();
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/projectively_stitched_tensor_patch_shell: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    #[cfg(feature = "tessellation")]
    {
        const CHORDAL_ITERATIONS: usize = 100;
        let (model, faces) = builder::tensor_patch_shell(stitched_specs.clone())
            .expect("benchmark chordal source shell");
        let policy = hyperbrep::tessellation::ChordalApproximationPolicy::uniform(
            std::num::NonZeroUsize::new(4).expect("nonzero boundary subdivision"),
            3,
        );
        let started = Instant::now();
        let mut checksum = 0_usize;
        for _ in 0..CHORDAL_ITERATIONS {
            let artifact = hyperbrep::tessellation::approximate_face_chordally(
                black_box(&model),
                faces[0],
                black_box(policy),
            )
            .expect("benchmark explicit chordal face output");
            assert_eq!(
                artifact.source_relation(),
                hyperbrep::tessellation::ChordalSourceRelation::ExactAtVerticesOnly
            );
            checksum += artifact.triangles().len();
            black_box(artifact);
        }
        let elapsed = started.elapsed();
        println!(
            "derived_output/face_exact_vertex_chordal_approximation: {CHORDAL_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
            elapsed / CHORDAL_ITERATIONS as u32,
        );

        let (analytic, analytic_solid) =
            builder::cylinder(Real::from(2), Real::from(3)).expect("benchmark analytic source");
        let face = *analytic
            .solid(analytic_solid)
            .and_then(|solid| analytic.shell(solid.outer()))
            .and_then(|shell| shell.faces().first())
            .expect("benchmark analytic face");
        let started = Instant::now();
        let mut checksum = 0_usize;
        for _ in 0..CHORDAL_ITERATIONS {
            let artifact = hyperbrep::tessellation::approximate_face_chordally(
                black_box(&analytic),
                face,
                black_box(policy),
            )
            .expect("benchmark curved-trim analytic chordal output");
            checksum += artifact.triangles().len();
            black_box(artifact);
        }
        let elapsed = started.elapsed();
        println!(
            "derived_output/analytic_face_exact_vertex_chordal_approximation: {CHORDAL_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
            elapsed / CHORDAL_ITERATIONS as u32,
        );
    }

    let linear_tensor = Surface::rational_bezier(
        vec![
            vec![point(0, 0, 0), point(1, 2, 0), point(2, 0, 0)],
            vec![point(0, 0, 2), point(1, 2, 2), point(2, 0, 2)],
        ],
        vec![
            vec![Real::one(), Real::from(2), Real::one()],
            vec![Real::one(), Real::from(2), Real::one()],
        ],
    )
    .expect("benchmark linear tensor surface");
    let iso_plane =
        Surface::plane(point(0, 0, 1), Vector3::x(), Vector3::y()).expect("benchmark iso plane");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let SurfaceSurfaceIntersection::Curve(curve) = black_box(&linear_tensor)
            .intersect_surface(black_box(&iso_plane))
            .expect("benchmark exact tensor iso-curve intersection")
        else {
            panic!("linear tensor plane intersection must retain one curve");
        };
        checksum += 1;
        black_box(curve);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/linear_tensor_plane_iso_curve_intersection: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let curved_translation_tensor = Surface::rational_bezier(
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 1), point(2, 2, 1)],
            vec![point(0, 2, 2), point(2, 2, 2)],
        ],
        vec![
            vec![Real::one(), Real::one()],
            vec![Real::from(2), Real::from(2)],
            vec![Real::from(3), Real::from(3)],
        ],
    )
    .expect("benchmark curved translation tensor");
    let oblique_plane = Surface::plane(
        point(2, 0, 0),
        Vector3::y(),
        Vector3::from_xyz(Real::one(), Real::zero(), -Real::one()),
    )
    .expect("benchmark oblique plane");
    let midpoint = (Real::one() / Real::from(2)).expect("exact benchmark midpoint");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let SurfaceSurfaceIntersection::Curve(curve) = black_box(&curved_translation_tensor)
            .intersect_surface(black_box(&oblique_plane))
            .expect("benchmark exact non-isoparametric tensor section")
        else {
            panic!("curved translation tensor must retain one exact section");
        };
        let pcurve = curve
            .second_pcurve()
            .materialize()
            .expect("benchmark exact rational graph pcurve");
        black_box(curve.curve().point_at(black_box(&midpoint)))
            .expect("benchmark exact section evaluation");
        checksum += 1;
        black_box(pcurve);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/curved_translation_tensor_plane_noniso_intersection: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let rational_bilinear_tensor = Surface::rational_bezier(
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 0), point(2, 2, 1)],
        ],
        vec![
            vec![Real::one(), Real::from(2)],
            vec![Real::from(3), Real::from(4)],
        ],
    )
    .expect("benchmark weighted rational bilinear tensor");
    let bilinear_plane = Surface::plane(
        point(1, 0, 0),
        Vector3::y(),
        Vector3::from_xyz(Real::from(2), Real::zero(), -Real::one()),
    )
    .expect("benchmark bilinear section plane");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let SurfaceSurfaceIntersection::Curve(curve) = black_box(&rational_bilinear_tensor)
            .intersect_surface(black_box(&bilinear_plane))
            .expect("benchmark exact weighted bilinear section")
        else {
            panic!("weighted bilinear tensor must retain one exact section");
        };
        black_box(curve.curve().point_at(black_box(&midpoint)))
            .expect("benchmark exact rational-quartic section evaluation");
        black_box(curve.first_pcurve().point_at(black_box(&midpoint)))
            .expect("benchmark exact rational-quadratic graph evaluation");
        checksum += 1;
        black_box(curve);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/rational_bilinear_tensor_plane_rational_quartic_intersection: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let fraction = |numerator: i32, denominator: i32| {
        (Real::from(numerator) / Real::from(denominator)).expect("benchmark rational denominator")
    };
    let pole_branch_tensor = Surface::rational_bezier(
        vec![
            vec![
                Point3::new(Real::zero(), Real::zero(), fraction(3, 16)),
                Point3::new(Real::from(2), Real::zero(), fraction(-5, 32)),
            ],
            vec![
                Point3::new(Real::zero(), Real::from(2), fraction(-5, 48)),
                Point3::new(Real::from(2), Real::from(2), fraction(3, 64)),
            ],
        ],
        vec![
            vec![Real::one(), Real::from(2)],
            vec![Real::from(3), Real::from(4)],
        ],
    )
    .expect("benchmark weighted rational bilinear pole tensor");
    let pole_plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y())
        .expect("benchmark bilinear pole plane");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let SurfaceSurfaceIntersection::Curves(branches) = black_box(&pole_branch_tensor)
            .intersect_surface(black_box(&pole_plane))
            .expect("benchmark exact bounded bilinear pole branches")
        else {
            panic!("weighted bilinear pole tensor must retain two exact branches");
        };
        assert_eq!(branches.len(), 2);
        for branch in &branches {
            black_box(branch.curve().point_at(black_box(&midpoint)))
                .expect("benchmark bounded rational-quartic branch evaluation");
            black_box(branch.first_pcurve().point_at(black_box(&midpoint)))
                .expect("benchmark bounded rational-quadratic branch evaluation");
            checksum += 1;
        }
        black_box(branches);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/rational_bilinear_tensor_plane_two_pole_branches: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let contained_tensor = Surface::rational_bezier(
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 0), point(2, 2, 0)],
        ],
        vec![
            vec![Real::one(), Real::from(2)],
            vec![Real::from(3), Real::from(4)],
        ],
    )
    .expect("benchmark plane-contained weighted bilinear tensor");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let SurfaceSurfaceIntersection::ContainedSurface(SurfaceIntersectionOperand::Second) =
            black_box(&pole_plane)
                .intersect_surface(black_box(&contained_tensor))
                .expect("benchmark exact bounded tensor containment")
        else {
            panic!("planar weighted tensor must retain its exact contained-surface relation");
        };
        checksum += 1;
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/rational_bilinear_tensor_plane_complete_containment: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    let (contained_patch, contained_face) = builder::rational_bezier_patch(
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 0), point(2, 2, 0)],
        ],
        vec![
            vec![Real::one(), Real::from(2)],
            vec![Real::from(3), Real::from(4)],
        ],
    )
    .expect("benchmark contained tensor face");
    let partial_outline = CurvePath2::try_new(vec![
        Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(Real::one(), -Real::one()),
                CurvePoint2::new(Real::from(3), -Real::one()),
            )
            .expect("benchmark partial plane edge"),
        ),
        Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(Real::from(3), -Real::one()),
                CurvePoint2::new(Real::from(3), Real::from(3)),
            )
            .expect("benchmark partial plane edge"),
        ),
        Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(Real::from(3), Real::from(3)),
                CurvePoint2::new(Real::one(), Real::from(3)),
            )
            .expect("benchmark partial plane edge"),
        ),
        Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(Real::one(), Real::from(3)),
                CurvePoint2::new(Real::one(), -Real::one()),
            )
            .expect("benchmark partial plane edge"),
        ),
    ])
    .expect("benchmark partial plane outline");
    let (partial_plane, partial_face) = builder::planar_face(
        &partial_outline,
        &[],
        point(0, 0, 0),
        Vector3::from_xyz(Real::one(), Real::zero(), Real::zero()),
        Vector3::from_xyz(Real::zero(), Real::one(), Real::zero()),
    )
    .expect("benchmark partial plane face");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPLINE_ITERATIONS {
        let pair = boolean::intersect_faces(
            black_box(&contained_patch),
            contained_face,
            black_box(&partial_plane),
            partial_face,
        )
        .expect("benchmark exact contained-face trim")
        .expect("benchmark overlapping contained faces");
        let boolean::FacePairTrim::SurfaceRegion { region, .. } = pair.trim() else {
            panic!("partial tensor/plane faces must retain an exact surface region");
        };
        checksum += usize::from(!region.is_empty());
        black_box(pair);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/rational_bilinear_tensor_plane_partial_face_region: {SPLINE_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPLINE_ITERATIONS as u32,
    );

    const TENSOR_GRAPH_SPLIT_ITERATIONS: usize = 100;
    let (graph_patch, graph_face) = builder::rational_bezier_patch(
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 1), point(2, 2, 1)],
            vec![point(0, 2, 2), point(2, 2, 2)],
        ],
        vec![
            vec![Real::one(), Real::one()],
            vec![Real::from(2), Real::from(2)],
            vec![Real::from(3), Real::from(3)],
        ],
    )
    .expect("benchmark graph tensor patch");
    let graph_surface = graph_patch
        .surface(
            graph_patch
                .face(graph_face)
                .expect("benchmark graph face")
                .surface(),
        )
        .expect("benchmark graph surface");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_GRAPH_SPLIT_ITERATIONS {
        let SurfaceSurfaceIntersection::Curve(curve) = black_box(graph_surface)
            .intersect_surface(black_box(&oblique_plane))
            .expect("benchmark exact graph tensor section")
        else {
            panic!("graph tensor must retain one exact section");
        };
        let (split, _) = graph_patch
            .split_face_by_surface_curve(graph_face, curve.curve(), curve.first_pcurve())
            .expect("benchmark non-isoparametric tensor face split");
        checksum += split.counts().faces;
        black_box(split);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/noniso_tensor_face_intersection_and_validated_split: {TENSOR_GRAPH_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_GRAPH_SPLIT_ITERATIONS as u32,
    );

    let (nurbs_graph_patch, nurbs_graph_face) = builder::nurbs_patch(
        1,
        2,
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 1), point(2, 2, 1)],
            vec![point(0, 2, 2), point(2, 2, 2)],
        ],
        vec![
            vec![Real::one(), Real::one()],
            vec![Real::from(2), Real::from(2)],
            vec![Real::from(3), Real::from(3)],
        ],
        vec![Real::zero(), Real::zero(), Real::one(), Real::one()],
        vec![
            Real::from(2),
            Real::from(2),
            Real::from(2),
            Real::from(5),
            Real::from(5),
            Real::from(5),
        ],
    )
    .expect("benchmark NURBS graph tensor patch");
    let nurbs_graph_surface = nurbs_graph_patch
        .surface(
            nurbs_graph_patch
                .face(nurbs_graph_face)
                .expect("benchmark NURBS graph face")
                .surface(),
        )
        .expect("benchmark NURBS graph surface");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_GRAPH_SPLIT_ITERATIONS {
        let SurfaceSurfaceIntersection::Curve(curve) = black_box(nurbs_graph_surface)
            .intersect_surface(black_box(&oblique_plane))
            .expect("benchmark exact NURBS graph tensor section")
        else {
            panic!("NURBS graph tensor must retain one exact section");
        };
        let (split, _) = nurbs_graph_patch
            .split_face_by_surface_curve(nurbs_graph_face, curve.curve(), curve.first_pcurve())
            .expect("benchmark non-isoparametric NURBS tensor face split");
        checksum += split.counts().faces;
        black_box(split);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/noniso_nurbs_tensor_face_intersection_and_validated_split: {TENSOR_GRAPH_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_GRAPH_SPLIT_ITERATIONS as u32,
    );

    let (v_nurbs_graph_patch, v_nurbs_graph_face) = builder::nurbs_patch(
        2,
        1,
        vec![
            vec![point(0, 0, 0), point(1, 2, 0), point(2, 2, 0)],
            vec![point(0, 0, 2), point(1, 2, 2), point(2, 2, 2)],
        ],
        vec![
            vec![Real::one(), Real::from(2), Real::from(3)],
            vec![Real::one(), Real::from(2), Real::from(3)],
        ],
        vec![
            Real::from(2),
            Real::from(2),
            Real::from(2),
            Real::from(5),
            Real::from(5),
            Real::from(5),
        ],
        vec![Real::from(7), Real::from(7), Real::from(11), Real::from(11)],
    )
    .expect("benchmark v-linear NURBS graph tensor patch");
    let v_nurbs_graph_surface = v_nurbs_graph_patch
        .surface(
            v_nurbs_graph_patch
                .face(v_nurbs_graph_face)
                .expect("benchmark v-linear NURBS graph face")
                .surface(),
        )
        .expect("benchmark v-linear NURBS graph surface");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_GRAPH_SPLIT_ITERATIONS {
        let SurfaceSurfaceIntersection::Curve(curve) = black_box(v_nurbs_graph_surface)
            .intersect_surface(black_box(&oblique_plane))
            .expect("benchmark exact v-linear NURBS graph tensor section")
        else {
            panic!("v-linear NURBS graph tensor must retain one exact section");
        };
        let (split, _) = v_nurbs_graph_patch
            .split_face_by_surface_curve(v_nurbs_graph_face, curve.curve(), curve.first_pcurve())
            .expect("benchmark non-isoparametric v-linear NURBS tensor face split");
        checksum += split.counts().faces;
        black_box(split);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/noniso_v_nurbs_tensor_face_intersection_and_validated_split: {TENSOR_GRAPH_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_GRAPH_SPLIT_ITERATIONS as u32,
    );

    let (multi_span_graph_patch, multi_span_graph_face) = builder::nurbs_patch(
        1,
        2,
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 1), point(2, 2, 1)],
            vec![point(0, 3, 1), point(2, 3, 1)],
            vec![point(0, 1, 2), point(2, 1, 2)],
        ],
        vec![
            vec![Real::one(), Real::one()],
            vec![Real::from(2), Real::from(2)],
            vec![Real::from(3), Real::from(3)],
            vec![Real::one(), Real::one()],
        ],
        vec![Real::from(7), Real::from(7), Real::from(11), Real::from(11)],
        vec![
            Real::from(2),
            Real::from(2),
            Real::from(2),
            Real::from(3),
            Real::from(5),
            Real::from(5),
            Real::from(5),
        ],
    )
    .expect("benchmark multi-span NURBS graph tensor patch");
    let multi_span_graph_surface = multi_span_graph_patch
        .surface(
            multi_span_graph_patch
                .face(multi_span_graph_face)
                .expect("benchmark multi-span NURBS graph face")
                .surface(),
        )
        .expect("benchmark multi-span NURBS graph surface");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_GRAPH_SPLIT_ITERATIONS {
        let SurfaceSurfaceIntersection::Curve(curve) = black_box(multi_span_graph_surface)
            .intersect_surface(black_box(&oblique_plane))
            .expect("benchmark exact multi-span NURBS graph tensor section")
        else {
            panic!("multi-span NURBS graph tensor must retain one exact section");
        };
        let (split, _) = multi_span_graph_patch
            .split_face_by_surface_curve(multi_span_graph_face, curve.curve(), curve.first_pcurve())
            .expect("benchmark non-isoparametric multi-span NURBS tensor face split");
        checksum += split.counts().faces;
        black_box(split);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/noniso_multi_span_nurbs_tensor_face_intersection_and_validated_split: {TENSOR_GRAPH_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_GRAPH_SPLIT_ITERATIONS as u32,
    );

    let (partial_graph_patch, partial_graph_face) = builder::nurbs_patch(
        1,
        1,
        vec![
            vec![point(0, 0, 3), point(2, 0, 3)],
            vec![point(0, 1, 1), point(2, 1, 1)],
            vec![point(0, 2, 3), point(2, 2, 3)],
            vec![point(0, 3, 1), point(2, 3, 1)],
            vec![point(0, 4, 3), point(2, 4, 3)],
        ],
        vec![vec![Real::one(), Real::one()]; 5],
        vec![Real::from(7), Real::from(7), Real::from(11), Real::from(11)],
        vec![
            Real::from(2),
            Real::from(2),
            Real::from(3),
            Real::from(4),
            Real::from(5),
            Real::from(6),
            Real::from(6),
        ],
    )
    .expect("benchmark partially bounded NURBS graph tensor patch");
    let partial_graph_surface = partial_graph_patch
        .surface(
            partial_graph_patch
                .face(partial_graph_face)
                .expect("benchmark partial NURBS graph face")
                .surface(),
        )
        .expect("benchmark partial NURBS graph surface");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_GRAPH_SPLIT_ITERATIONS {
        let SurfaceSurfaceIntersection::Curves(curves) = black_box(partial_graph_surface)
            .intersect_surface(black_box(&oblique_plane))
            .expect("benchmark exact partially bounded NURBS graph tensor sections")
        else {
            panic!("partial NURBS graph tensor must retain disjoint exact sections");
        };
        let (partitioned, partition) = partial_graph_patch
            .split_face_by_surface_curves(
                partial_graph_face,
                &curves,
                SurfaceIntersectionOperand::First,
            )
            .expect("benchmark partially bounded NURBS tensor face partition");
        assert_eq!(partition.faces.len(), curves.len() + 1);
        checksum += curves.len() + partitioned.counts().faces;
        black_box(partitioned);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/partial_noniso_nurbs_tensor_intersection_and_validated_partition: {TENSOR_GRAPH_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_GRAPH_SPLIT_ITERATIONS as u32,
    );

    let (crossing_patch, crossing_face) = builder::rational_bezier_patch(
        vec![
            vec![point(0, 0, 0), point(2, 0, 0)],
            vec![point(0, 2, 0), point(2, 2, 0)],
        ],
        vec![vec![Real::one(), Real::one()]; 2],
    )
    .expect("benchmark crossing tensor patch");
    let crossing_surface = crossing_patch
        .surface(
            crossing_patch
                .face(crossing_face)
                .expect("benchmark crossing tensor face")
                .surface(),
        )
        .expect("benchmark crossing tensor surface");
    let x_plane = Surface::plane(point(1, 0, 0), Vector3::y(), Vector3::z())
        .expect("benchmark x selection plane");
    let y_plane = Surface::plane(point(0, 1, 0), Vector3::x(), Vector3::z())
        .expect("benchmark y selection plane");
    let SurfaceSurfaceIntersection::Curve(x_trace) = crossing_surface
        .intersect_surface(&x_plane)
        .expect("benchmark x tensor trace")
    else {
        panic!("x selection must retain one tensor trace");
    };
    let SurfaceSurfaceIntersection::Curve(y_trace) = crossing_surface
        .intersect_surface(&y_plane)
        .expect("benchmark y tensor trace")
    else {
        panic!("y selection must retain one tensor trace");
    };
    let crossing_traces = [*x_trace, *y_trace];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_GRAPH_SPLIT_ITERATIONS {
        let (partitioned, partition) = crossing_patch
            .split_face_by_surface_curves(
                crossing_face,
                black_box(&crossing_traces),
                SurfaceIntersectionOperand::First,
            )
            .expect("benchmark exact crossing tensor partition");
        assert_eq!(partition.faces.len(), 4);
        checksum += partitioned.counts().faces;
        black_box(partitioned);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/crossing_tensor_pcurve_arrangement_and_validated_partition: {TENSOR_GRAPH_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_GRAPH_SPLIT_ITERATIONS as u32,
    );

    const TENSOR_SPLIT_ITERATIONS: usize = 100;
    let (tensor_patch, tensor_face) = builder::rational_bezier_patch(
        vec![
            vec![point(0, 0, 0), point(1, 2, 0), point(2, 0, 0)],
            vec![point(0, 0, 2), point(1, 2, 2), point(2, 0, 2)],
        ],
        vec![
            vec![Real::one(), Real::from(2), Real::one()],
            vec![Real::one(), Real::from(2), Real::one()],
        ],
    )
    .expect("benchmark tensor patch");
    let (tensor_cutter, tensor_cutter_solid) =
        builder::cuboid(point(-1, -1, 1), point(3, 3, 2)).expect("benchmark tensor cutter");
    let tensor_plane_face = *tensor_cutter
        .shell(
            tensor_cutter
                .solid(tensor_cutter_solid)
                .expect("benchmark cutter solid")
                .outer(),
        )
        .expect("benchmark cutter shell")
        .faces()
        .first()
        .expect("benchmark cutter face");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_SPLIT_ITERATIONS {
        let pair = boolean::intersect_faces(
            black_box(&tensor_patch),
            tensor_face,
            black_box(&tensor_cutter),
            tensor_plane_face,
        )
        .expect("benchmark tensor face intersection")
        .expect("benchmark tensor faces meet");
        let boolean::FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim() else {
            panic!("benchmark tensor face trim must retain one native fragment");
        };
        let (split, _) = tensor_patch
            .split_face_by_surface_curve(
                tensor_face,
                fragments[0].curve(),
                fragments[0].first_pcurve(),
            )
            .expect("benchmark tensor face transfer");
        checksum += split.counts().faces;
        black_box(split);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/tensor_face_intersection_and_validated_split: {TENSOR_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_SPLIT_ITERATIONS as u32,
    );

    let (affine_tensor, affine_tensor_face) = builder::rational_bezier_patch(
        vec![
            vec![point(0, 0, 1), point(1, 0, 1)],
            vec![point(0, 1, 1), point(1, 1, 1)],
        ],
        vec![vec![Real::one(), Real::one()]; 2],
    )
    .expect("benchmark affine tensor patch");
    let affine_tensor_surface = affine_tensor
        .surface(
            affine_tensor
                .face(affine_tensor_face)
                .expect("benchmark affine tensor face")
                .surface(),
        )
        .expect("benchmark affine tensor surface");
    let half = (Real::one() / Real::from(2)).expect("two is nonzero");
    let boundary_support = Surface::plane(
        Point3::new(half, Real::zero(), Real::one()),
        Vector3::y(),
        Vector3::z(),
    )
    .expect("benchmark boundary support plane");
    let SurfaceSurfaceIntersection::Curve(boundary_trace) = affine_tensor_surface
        .intersect_surface(&boundary_support)
        .expect("benchmark inverse tensor pcurve")
    else {
        panic!("affine tensor boundary support must retain one exact trace");
    };
    let boundary_trace = [*boundary_trace];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_SPLIT_ITERATIONS {
        let (partitioned, partition) = affine_tensor
            .split_face_by_surface_curves(
                affine_tensor_face,
                black_box(&boundary_trace),
                SurfaceIntersectionOperand::First,
            )
            .expect("benchmark affine tensor boundary partition");
        assert_eq!(partition.faces.len(), 2);
        checksum += partitioned.counts().faces;
        black_box(partitioned);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/affine_tensor_inverse_pcurve_boundary_partition: {TENSOR_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_SPLIT_ITERATIONS as u32,
    );

    let quarter = (Real::one() / Real::from(4)).expect("four is nonzero");
    let half = (Real::one() / Real::from(2)).expect("two is nonzero");
    let three_quarters = (Real::from(3) / Real::from(4)).expect("four is nonzero");
    let curve_start = CurvePoint2::new(Real::zero(), quarter.clone());
    let curve_end = CurvePoint2::new(Real::one(), three_quarters.clone());
    let curved_region = CurvePath2::try_new(vec![
        Curve2::from(QuadraticBezier2::new(
            curve_start.clone(),
            CurvePoint2::new(half, Real::zero()),
            curve_end.clone(),
        )),
        Curve2::from(
            LineSeg2::try_new(curve_end, CurvePoint2::new(Real::one(), Real::from(2)))
                .expect("benchmark curved region right edge"),
        ),
        Curve2::from(
            LineSeg2::try_new(
                CurvePoint2::new(Real::one(), Real::from(2)),
                CurvePoint2::new(Real::zero(), Real::from(2)),
            )
            .expect("benchmark curved region upper edge"),
        ),
        Curve2::from(
            LineSeg2::try_new(CurvePoint2::new(Real::zero(), Real::from(2)), curve_start)
                .expect("benchmark curved region left edge"),
        ),
    ])
    .expect("benchmark curved region");
    let (curved_plane, curved_plane_face) = builder::planar_face(
        &curved_region,
        &[],
        point(0, 0, 1),
        Vector3::x(),
        Vector3::y(),
    )
    .expect("benchmark curved plane region");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_SPLIT_ITERATIONS {
        let (partitioned, partition) = boolean::partition_contained_face_by_plane_region(
            black_box(&affine_tensor),
            affine_tensor_face,
            black_box(&curved_plane),
            curved_plane_face,
        )
        .expect("benchmark exact curved contained-region partition")
        .expect("benchmark curved boundary is represented");
        assert_eq!(partition.faces.len(), 2);
        checksum += partitioned.counts().faces;
        black_box(partitioned);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/curved_affine_tensor_inverse_pcurve_partition: {TENSOR_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_SPLIT_ITERATIONS as u32,
    );

    let outer = [
        hyperbrep::Point2::new(Real::zero(), Real::zero()),
        hyperbrep::Point2::new(Real::one(), Real::zero()),
        hyperbrep::Point2::new(Real::one(), Real::one()),
        hyperbrep::Point2::new(Real::zero(), Real::one()),
    ];
    let hole = vec![
        hyperbrep::Point2::new(quarter.clone(), quarter.clone()),
        hyperbrep::Point2::new(three_quarters.clone(), quarter.clone()),
        hyperbrep::Point2::new(three_quarters.clone(), three_quarters.clone()),
        hyperbrep::Point2::new(quarter.clone(), three_quarters.clone()),
    ];
    let (holed_source, holed_solid) =
        builder::extrude_region(&outer, &[hole], Real::zero(), Real::one())
            .expect("benchmark holed extrusion");
    let holed_face = holed_source
        .shell(
            holed_source
                .solid(holed_solid)
                .expect("benchmark holed solid")
                .outer(),
        )
        .expect("benchmark holed shell")
        .faces()[1];
    let holed_surface_id = holed_source
        .face(holed_face)
        .expect("benchmark holed cap")
        .surface();
    let holed_tensor_surface = Surface::rational_bezier(
        vec![
            vec![point(0, 0, 1), point(1, 0, 1)],
            vec![point(0, 1, 1), point(1, 1, 1)],
        ],
        vec![vec![Real::one(), Real::one()]; 2],
    )
    .expect("benchmark holed tensor surface");
    let mut edit = holed_source.edit();
    edit.replace_surface(holed_surface_id, holed_tensor_surface.clone())
        .expect("benchmark holed tensor replacement");
    let holed_tensor = edit.commit().expect("benchmark certified holed tensor");
    let SurfaceSurfaceIntersection::Curve(holed_trace) = holed_tensor_surface
        .intersect_surface(&boundary_support)
        .expect("benchmark holed inverse tensor pcurve")
    else {
        panic!("holed affine tensor boundary support must retain one exact trace");
    };
    let holed_traces = [
        holed_trace
            .subcurve(&Real::zero(), &quarter)
            .expect("benchmark lower bridge"),
        holed_trace
            .subcurve(&three_quarters, &Real::one())
            .expect("benchmark upper bridge"),
    ];
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_SPLIT_ITERATIONS {
        let (partitioned, partition) = holed_tensor
            .split_face_by_surface_curves(
                holed_face,
                black_box(&holed_traces),
                SurfaceIntersectionOperand::First,
            )
            .expect("benchmark paired tensor bridges");
        assert_eq!(partition.faces.len(), 2);
        assert_eq!(
            compare_reals(
                &partitioned
                    .solid_volume(holed_solid)
                    .expect("benchmark holed tensor volume"),
                &(Real::from(3) / Real::from(4)).expect("four is nonzero"),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += partitioned.counts().faces;
        black_box(partitioned);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/holed_affine_tensor_paired_bridge_partition: {TENSOR_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_SPLIT_ITERATIONS as u32,
    );

    let eighth = (Real::one() / Real::from(8)).expect("eight is nonzero");
    let three_eighths = (Real::from(3) / Real::from(8)).expect("eight is nonzero");
    let five_eighths = (Real::from(5) / Real::from(8)).expect("eight is nonzero");
    let seven_eighths = (Real::from(7) / Real::from(8)).expect("eight is nonzero");
    let threaded_holes = [
        vec![
            hyperbrep::Point2::new(quarter.clone(), eighth.clone()),
            hyperbrep::Point2::new(three_quarters.clone(), eighth.clone()),
            hyperbrep::Point2::new(three_quarters.clone(), three_eighths.clone()),
            hyperbrep::Point2::new(quarter.clone(), three_eighths.clone()),
        ],
        vec![
            hyperbrep::Point2::new(quarter.clone(), five_eighths.clone()),
            hyperbrep::Point2::new(three_quarters.clone(), five_eighths.clone()),
            hyperbrep::Point2::new(three_quarters.clone(), seven_eighths.clone()),
            hyperbrep::Point2::new(quarter.clone(), seven_eighths.clone()),
        ],
    ];
    let (threaded_source, threaded_solid) =
        builder::extrude_region(&outer, &threaded_holes, Real::zero(), Real::one())
            .expect("benchmark two-hole extrusion");
    let threaded_face = threaded_source
        .shell(
            threaded_source
                .solid(threaded_solid)
                .expect("benchmark two-hole solid")
                .outer(),
        )
        .expect("benchmark two-hole shell")
        .faces()[1];
    let threaded_surface_id = threaded_source
        .face(threaded_face)
        .expect("benchmark two-hole cap")
        .surface();
    let threaded_tensor_surface = Surface::rational_bezier(
        vec![
            vec![point(0, 0, 1), point(1, 0, 1)],
            vec![point(0, 1, 1), point(1, 1, 1)],
        ],
        vec![vec![Real::one(), Real::one()]; 2],
    )
    .expect("benchmark two-hole tensor surface");
    let mut edit = threaded_source.edit();
    edit.replace_surface(threaded_surface_id, threaded_tensor_surface.clone())
        .expect("benchmark two-hole tensor replacement");
    let threaded_tensor = edit.commit().expect("benchmark certified two-hole tensor");
    let SurfaceSurfaceIntersection::Curve(threaded_trace) = threaded_tensor_surface
        .intersect_surface(&boundary_support)
        .expect("benchmark threaded inverse tensor pcurve")
    else {
        panic!("threaded affine tensor support must retain one exact trace");
    };
    let threaded_traces = [
        threaded_trace
            .subcurve(&Real::zero(), &eighth)
            .expect("benchmark first bridge"),
        threaded_trace
            .subcurve(&three_eighths, &five_eighths)
            .expect("benchmark middle bridge"),
        threaded_trace
            .subcurve(&seven_eighths, &Real::one())
            .expect("benchmark last bridge"),
    ];
    let threaded_source_wires = threaded_tensor.counts().wires;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..TENSOR_SPLIT_ITERATIONS {
        let (partitioned, partition) = threaded_tensor
            .split_face_by_surface_curves(
                threaded_face,
                black_box(&threaded_traces),
                SurfaceIntersectionOperand::First,
            )
            .expect("benchmark multi-hole bridge cycle");
        assert_eq!(partition.faces.len(), 2);
        assert_eq!(partitioned.counts().wires + 1, threaded_source_wires);
        assert_eq!(
            compare_reals(
                &partitioned
                    .solid_volume(threaded_solid)
                    .expect("benchmark threaded tensor volume"),
                &(Real::from(3) / Real::from(4)).expect("four is nonzero"),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        checksum += partitioned.counts().faces;
        black_box(partitioned);
    }
    let elapsed = started.elapsed();
    println!(
        "spline_kernel/multi_hole_affine_tensor_bridge_cycle_partition: {TENSOR_SPLIT_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / TENSOR_SPLIT_ITERATIONS as u32,
    );

    const BOOLEAN_ITERATIONS: usize = 250;
    let (first_box, first_box_solid) =
        builder::cuboid(point(0, 0, 0), point(2, 2, 2)).expect("first graph cuboid");
    let (second_box, second_box_solid) =
        builder::cuboid(point(3, 0, 0), point(4, 1, 1)).expect("second graph cuboid");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..BOOLEAN_ITERATIONS {
        let graph =
            boolean::intersection_graph(&first_box, first_box_solid, &second_box, second_box_solid)
                .expect("benchmark disjoint intersection graph");
        assert_eq!(graph.candidate_pairs(), 36);
        assert_eq!(graph.broad_phase_rejections(), 36);
        checksum += graph.broad_phase_rejections();
        black_box(graph);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/disjoint_cuboid_intersection_graph: {BOOLEAN_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / BOOLEAN_ITERATIONS as u32,
    );

    let (overlapping_box, overlapping_box_solid) =
        builder::cuboid(point(1, 1, 0), point(3, 3, 2)).expect("overlapping graph cuboid");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..BOOLEAN_ITERATIONS {
        let graph = boolean::intersection_graph(
            &first_box,
            first_box_solid,
            &overlapping_box,
            overlapping_box_solid,
        )
        .expect("benchmark intersecting planar graph");
        checksum += graph
            .intersections()
            .iter()
            .filter_map(|pair| match pair.trim() {
                boolean::FacePairTrim::CurveFragments(fragments) => Some(fragments.len()),
                boolean::FacePairTrim::SurfaceCurveFragments(fragments) => Some(fragments.len()),
                boolean::FacePairTrim::Components {
                    surface_curve_fragments,
                    ..
                } => Some(surface_curve_fragments.len()),
                boolean::FacePairTrim::NotAvailable
                | boolean::FacePairTrim::CompleteCarrier
                | boolean::FacePairTrim::CoincidentPlanar { .. }
                | boolean::FacePairTrim::SurfaceRegion { .. }
                | boolean::FacePairTrim::PointContact(_)
                | boolean::FacePairTrim::NoContact
                | boolean::FacePairTrim::NoCurveInterior
                | boolean::FacePairTrim::Unresolved(_) => None,
            })
            .sum::<usize>();
        black_box(graph);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/overlapping_cuboid_trimmed_intersection_graph: {BOOLEAN_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / BOOLEAN_ITERATIONS as u32,
    );

    let graph = boolean::intersection_graph(
        &first_box,
        first_box_solid,
        &overlapping_box,
        overlapping_box_solid,
    )
    .expect("benchmark retained intersecting planar graph");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..BOOLEAN_ITERATIONS {
        let (partitioned, partitions) = black_box(&graph)
            .partition_first_planar_faces()
            .expect("benchmark retained-graph planar partitions");
        checksum += partitions
            .iter()
            .map(|partition| partition.faces.len())
            .sum::<usize>();
        assert_eq!(
            compare_reals(
                &partitioned
                    .solid_volume(first_box_solid)
                    .expect("partitioned graph volume"),
                &Real::from(8),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        black_box(partitioned);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/retained_graph_planar_face_partition: {BOOLEAN_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / BOOLEAN_ITERATIONS as u32,
    );

    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..BOOLEAN_ITERATIONS {
        let selected = black_box(&graph)
            .select_first_faces(boolean::BooleanOperation::Intersection)
            .expect("benchmark exact planar fragment selection");
        checksum += selected
            .faces
            .iter()
            .filter(|face| face.action == boolean::FaceSelectionAction::Keep)
            .count();
        assert_eq!(selected.partitions.len(), 4);
        assert_eq!(
            compare_reals(
                &selected
                    .model
                    .solid_volume(first_box_solid)
                    .expect("selected model volume"),
                &Real::from(8),
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        black_box(selected);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/retained_graph_planar_face_classify_select: {BOOLEAN_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / BOOLEAN_ITERATIONS as u32,
    );

    const PLANAR_STITCH_ITERATIONS: usize = 50;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..PLANAR_STITCH_ITERATIONS {
        let boolean::BooleanResult::Solid { model, solid } = black_box(&graph)
            .stitch_selected_faces(boolean::BooleanOperation::Union)
            .expect("benchmark exact planar selected-face stitching")
        else {
            panic!("overlapping cuboid union must stitch one solid");
        };
        checksum += model.counts().faces;
        black_box(model.solid_volume(solid).expect("stitched planar volume"));
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/retained_graph_planar_face_stitch_union: {PLANAR_STITCH_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / PLANAR_STITCH_ITERATIONS as u32,
    );

    const SKEW_PLANAR_ITERATIONS: usize = 25;
    let three_fifths = (Real::from(3) / Real::from(5)).expect("three fifths");
    let four_fifths = (Real::from(4) / Real::from(5)).expect("four fifths");
    let half = (Real::one() / Real::from(2)).expect("one half");
    let skew = hyperbrep::Matrix4::affine_orthonormal(
        [
            [Real::one(), Real::zero(), Real::zero()],
            [Real::zero(), three_fifths.clone(), -four_fifths.clone()],
            [Real::zero(), four_fifths, three_fifths],
        ],
        [half.clone(), half.clone(), half],
    );
    let skew_box = first_box.transformed(&skew).expect("benchmark skew cuboid");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SKEW_PLANAR_ITERATIONS {
        let boolean::BooleanResult::Solid { model, solid } =
            boolean::intersection(&first_box, first_box_solid, &skew_box, first_box_solid)
                .expect("benchmark skew convex planar intersection")
        else {
            panic!("skew cuboid intersection must be one solid");
        };
        checksum += model.counts().faces;
        black_box(model.solid_volume(solid).expect("skew convex volume"));
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/skew_cuboid_convex_intersection: {SKEW_PLANAR_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SKEW_PLANAR_ITERATIONS as u32,
    );

    const SKEW_GENERAL_ITERATIONS: usize = 10;
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SKEW_GENERAL_ITERATIONS {
        let boolean::BooleanResult::Solid { model, solid } =
            boolean::difference(&first_box, first_box_solid, &skew_box, first_box_solid)
                .expect("benchmark skew general planar difference")
        else {
            panic!("generic skew cuboid difference must be one solid");
        };
        checksum += model.counts().faces;
        black_box(model.solid_volume(solid).expect("skew difference volume"));
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/skew_cuboid_general_difference: {SKEW_GENERAL_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SKEW_GENERAL_ITERATIONS as u32,
    );

    const SKEW_VOID_ITERATIONS: usize = 10;
    let (void_inner, void_inner_solid) =
        builder::cuboid(point(0, 0, 0), point(1, 1, 1)).expect("benchmark void cuboid");
    let fraction = |numerator: i32| {
        (Real::from(numerator) / Real::from(25)).expect("nonzero benchmark denominator")
    };
    let void_transform = hyperbrep::Matrix4::affine_orthonormal(
        [
            [fraction(9), fraction(-12), fraction(20)],
            [fraction(20), fraction(15), Real::zero()],
            [fraction(-12), fraction(16), fraction(15)],
        ],
        [fraction(16), fraction(5), fraction(13)],
    );
    let void_inner = void_inner
        .transformed(&void_transform)
        .expect("benchmark skew void transform");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SKEW_VOID_ITERATIONS {
        let boolean::BooleanResult::Solid { model, solid } =
            boolean::difference(&first_box, first_box_solid, &void_inner, void_inner_solid)
                .expect("benchmark contained skew planar void difference")
        else {
            panic!("contained skew cuboid must produce one void solid");
        };
        checksum += model.counts().faces;
        black_box(model.solid_volume(solid).expect("skew void volume"));
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/skew_cuboid_general_void_difference: {SKEW_VOID_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SKEW_VOID_ITERATIONS as u32,
    );

    let (graph_sphere, graph_sphere_solid) = builder::sphere(Real::from(2)).expect("graph sphere");
    let (conic_box, conic_box_solid) =
        builder::cuboid(point(1, 1, -3), point(3, 3, 3)).expect("conic trim box");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..BOOLEAN_ITERATIONS {
        let graph = boolean::intersection_graph(
            &graph_sphere,
            graph_sphere_solid,
            &conic_box,
            conic_box_solid,
        )
        .expect("benchmark planar conic trim graph");
        checksum += graph.trimmed_curve_fragments();
        black_box(graph);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/sphere_planar_conic_trim_graph: {BOOLEAN_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / BOOLEAN_ITERATIONS as u32,
    );

    let (first, first_solid) = builder::sphere(Real::one()).expect("first Boolean sphere");
    let (second, second_solid) = builder::sphere(Real::one()).expect("second Boolean sphere");
    let second = second
        .transformed(&hyperbrep::Matrix4::affine_translation([
            Real::from(5),
            Real::zero(),
            Real::zero(),
        ]))
        .expect("translate second Boolean sphere");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..BOOLEAN_ITERATIONS {
        let boolean::BooleanResult::Solids { model, solids } =
            boolean::union(&first, first_solid, &second, second_solid)
                .expect("benchmark disjoint Boolean")
        else {
            panic!("disjoint sphere union must retain two solids");
        };
        checksum += solids.len();
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/disjoint_sphere_union_remap_validate: {BOOLEAN_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / BOOLEAN_ITERATIONS as u32,
    );

    const SPHERE_PAIR_ITERATIONS: usize = 100;
    let overlapping = first
        .transformed(&hyperbrep::Matrix4::affine_translation([
            Real::one(),
            Real::zero(),
            Real::zero(),
        ]))
        .expect("translate overlapping Boolean sphere");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..SPHERE_PAIR_ITERATIONS {
        let boolean::BooleanResult::Solid { model, solid } =
            boolean::intersection(&first, first_solid, &overlapping, first_solid)
                .expect("benchmark partial sphere intersection")
        else {
            panic!("partial sphere intersection must retain one solid");
        };
        checksum += model.counts().faces;
        black_box(model.solid_volume(solid).expect("sphere lens volume"));
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/partial_sphere_intersection_build_measure: {SPHERE_PAIR_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / SPHERE_PAIR_ITERATIONS as u32,
    );

    const CYLINDER_INTERVAL_ITERATIONS: usize = 100;
    let orient = |offset: i32| {
        hyperbrep::Matrix4::affine_orthonormal(
            [
                [Real::zero(), Real::zero(), Real::one()],
                [Real::one(), Real::zero(), Real::zero()],
                [Real::zero(), Real::one(), Real::zero()],
            ],
            [Real::from(offset), Real::zero(), Real::zero()],
        )
    };
    let (interval, interval_solid) =
        builder::cylinder(Real::from(2), Real::from(4)).expect("interval cylinder");
    let interval = interval
        .transformed(&orient(10))
        .expect("orient interval cylinder");
    let (cut, cut_solid) =
        builder::cylinder(Real::from(2), Real::one()).expect("interval cut cylinder");
    let cut = cut
        .transformed(&orient(11))
        .expect("orient interval cut cylinder");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..CYLINDER_INTERVAL_ITERATIONS {
        let boolean::BooleanResult::Solids { model, solids } =
            boolean::difference(&interval, interval_solid, &cut, cut_solid)
                .expect("benchmark oriented cylinder interval difference")
        else {
            panic!("interior cylinder interval cut must return two solids");
        };
        checksum += solids.len();
        black_box(
            solids
                .iter()
                .map(|solid| model.solid_volume(*solid).expect("interval volume"))
                .fold(Real::zero(), |sum, volume| sum + volume),
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/oriented_cylinder_interval_cut_build_measure: {CYLINDER_INTERVAL_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / CYLINDER_INTERVAL_ITERATIONS as u32,
    );

    const FRUSTUM_INTERVAL_ITERATIONS: usize = 25;
    let (frustum, frustum_solid) =
        builder::cone_frustum(Real::from(4), Real::one(), Real::from(3)).expect("interval frustum");
    let (frustum_cut, frustum_cut_solid) =
        builder::cone_frustum(Real::from(3), Real::from(2), Real::one())
            .expect("interval frustum cut");
    let frustum_cut = frustum_cut
        .transformed(&hyperbrep::Matrix4::affine_translation([
            Real::zero(),
            Real::zero(),
            Real::one(),
        ]))
        .expect("place interval frustum cut");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..FRUSTUM_INTERVAL_ITERATIONS {
        let boolean::BooleanResult::Solids { model, solids } =
            boolean::difference(&frustum, frustum_solid, &frustum_cut, frustum_cut_solid)
                .expect("benchmark cone-frustum interval difference")
        else {
            panic!("interior frustum interval cut must return two solids");
        };
        checksum += solids.len();
        black_box(
            solids
                .iter()
                .map(|solid| model.solid_volume(*solid).expect("frustum interval volume"))
                .fold(Real::zero(), |sum, volume| sum + volume),
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/cone_frustum_interval_cut_build_measure: {FRUSTUM_INTERVAL_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / FRUSTUM_INTERVAL_ITERATIONS as u32,
    );

    const AXIAL_FRUSTUM_ITERATIONS: usize = 10;
    let diagonal =
        (Real::one() / Real::from(2).sqrt().expect("sqrt two")).expect("inverse sqrt two");
    let axial_frame = hyperbrep::Matrix4::affine_orthonormal(
        [
            [diagonal.clone(), -diagonal.clone(), Real::zero()],
            [diagonal.clone(), diagonal, Real::zero()],
            [Real::zero(), Real::zero(), Real::one()],
        ],
        [Real::zero(), Real::zero(), Real::zero()],
    );
    let axial_frustum = frustum
        .transformed(&axial_frame)
        .expect("rotate frustum parameter frame");
    let (axial_cutter, axial_cutter_solid) =
        builder::cuboid(point(0, -5, -1), point(5, 5, 4)).expect("axial frustum cutter");
    let expected_axial_volume =
        (Real::from(21) * Real::pi() / Real::from(2)).expect("half-frustum volume");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..AXIAL_FRUSTUM_ITERATIONS {
        let boolean::BooleanResult::Solid { model, solid } = boolean::intersection(
            black_box(&axial_frustum),
            frustum_solid,
            black_box(&axial_cutter),
            axial_cutter_solid,
        )
        .expect("benchmark axial half-frustum intersection") else {
            panic!("axial frustum cut must retain one solid");
        };
        assert_eq!(
            compare_reals(
                &model.solid_volume(solid).expect("half-frustum volume"),
                &expected_axial_volume,
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = model.to_json().expect("half-frustum JSON");
        assert!(
            json.len() < 100_000,
            "mixed-shell planar pcurves must remain canonical"
        );
        let decoded = RawModel::from_json(&json)
            .expect("parse half-frustum JSON")
            .validate()
            .expect("revalidate half-frustum JSON");
        checksum += decoded.counts().faces;
        black_box(decoded);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/axial_half_frustum_build_measure_replay: {AXIAL_FRUSTUM_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / AXIAL_FRUSTUM_ITERATIONS as u32,
    );

    const REVOLUTION_BOOLEAN_ITERATIONS: usize = 25;
    let first_profile = [
        hyperbrep::Point2::new(Real::one(), Real::zero()),
        hyperbrep::Point2::new(Real::from(5), Real::zero()),
        hyperbrep::Point2::new(Real::from(5), Real::from(4)),
        hyperbrep::Point2::new(Real::one(), Real::from(4)),
    ];
    let second_profile = [
        hyperbrep::Point2::new(Real::from(2), Real::one()),
        hyperbrep::Point2::new(Real::from(3), Real::one()),
        hyperbrep::Point2::new(Real::from(3), Real::from(2)),
        hyperbrep::Point2::new(Real::from(2), Real::from(2)),
    ];
    let (first, first_solid) =
        builder::revolve(&first_profile).expect("benchmark revolution outer");
    let (second, second_solid) =
        builder::revolve(&second_profile).expect("benchmark revolution cavity");
    let started = Instant::now();
    let mut checksum = 0_usize;
    for _ in 0..REVOLUTION_BOOLEAN_ITERATIONS {
        let result = boolean::difference(
            black_box(&first),
            first_solid,
            black_box(&second),
            second_solid,
        )
        .expect("benchmark coaxial revolution difference");
        let boolean::BooleanResult::Solid { model, solid } = result else {
            panic!("contained revolution cut is connected");
        };
        checksum += usize::from(
            compare_reals(
                &model.solid_volume(solid).expect("revolution cut volume"),
                &(Real::from(91) * Real::pi()),
            )
            .value()
                == Some(std::cmp::Ordering::Equal),
        );
        black_box(model);
    }
    let elapsed = started.elapsed();
    println!(
        "boolean_kernel/coaxial_revolution_cavity_build_measure: {REVOLUTION_BOOLEAN_ITERATIONS} iterations in {elapsed:?} ({:?}/iter), checksum={checksum}",
        elapsed / REVOLUTION_BOOLEAN_ITERATIONS as u32,
    );
}
