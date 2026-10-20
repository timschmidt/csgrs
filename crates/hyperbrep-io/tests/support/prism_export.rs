//! Independent half-cylinder expectations and complete exact exchange workflows.
use hyperbrep::*;
use hyperbrep_io::step::{StepBody, StepWriteOptions};

#[allow(dead_code)]
#[path = "analytic_export.rs"]
mod analytic;
#[allow(dead_code)]
#[path = "../../../hyperbrep/tests/support/seam_cuts.rs"]
mod seam;

pub fn same_height() -> u64 {
    let source = builder::cylinder(5.into(), 6.into()).unwrap();
    let cutter = builder::cuboid(seam::point(0, -20, 0), seam::point(20, 20, 6)).unwrap();
    let half = boolean::difference(&source, &cutter).unwrap();
    let mut receipt = 0xcbf29ce484222325_u64;
    for reflected in [false, true] {
        let scale = if reflected {
            seam::ratio(1, 1000)
        } else {
            Real::one()
        };
        let map = if reflected {
            seam::similarity(&scale, true)
        } else {
            Matrix4::identity()
        };
        let moved = half.model().transformed(&map).unwrap();
        let replay = seam::reload(&moved);
        let output = analytic::export(
            &[StepBody {
                solid: replay.solids().next().unwrap(),
                name: "half cylinder",
            }],
            &StepWriteOptions::new("prism.step", "2026-10-16T00:00:00"),
        );
        for byte in output.text().bytes() {
            receipt = (receipt ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
        let imported = analytic::reimport(&output);
        let body = imported.bodies()[0].solid();
        analytic::equal(
            &body.volume().unwrap(),
            &(Real::from(75) * Real::pi() * &scale * &scale * &scale),
        );
        let area = body
            .outer()
            .faces()
            .map(|face| face.area().unwrap())
            .fold(Real::zero(), |a, b| a + b);
        analytic::equal(
            &area,
            &((Real::from(55) * Real::pi() + Real::from(60)) * &scale * &scale),
        );
        for (point, expected) in [
            (seam::point(-1, 0, 3), SolidPointLocation::Inside),
            (seam::point(1, 0, 3), SolidPointLocation::Outside),
            (seam::point(0, 0, 3), SolidPointLocation::Boundary),
            (seam::point(-5, 0, 3), SolidPointLocation::Boundary),
        ] {
            assert_eq!(
                body.classify_point(&map.transform_point3(&point).unwrap())
                    .unwrap(),
                expected
            );
        }
        let retained = BodySet::from_solid(body);
        let empty = boolean::difference(&retained, &retained).unwrap();
        assert_eq!(
            empty.model().to_json().unwrap(),
            BodySet::empty().model().to_json().unwrap()
        );
        let recovered = boolean::union(&empty, &retained).unwrap();
        analytic::equal(
            &recovered.bodies().next().unwrap().volume().unwrap(),
            &body.volume().unwrap(),
        );
    }
    receipt
}

fn annular_prism() -> BodySet {
    use hypercurve::{CircularArc2, Curve2, CurvePath2, Point2 as UV};
    let circle = |r: i32| {
        CurvePath2::try_new(
            [((r, 0), (-r, 0)), ((-r, 0), (r, 0))]
                .into_iter()
                .map(|(a, b)| {
                    Curve2::from(
                        CircularArc2::try_from_center(
                            UV::new(a.0.into(), a.1.into()),
                            UV::new(b.0.into(), b.1.into()),
                            UV::new(0.into(), 0.into()),
                            false,
                        )
                        .unwrap(),
                    )
                })
                .collect(),
        )
        .unwrap()
    };
    builder::extrude_path_region(&circle(5), &[circle(2)], 0.into(), 6.into()).unwrap()
}

pub fn holes() -> u64 {
    let source = annular_prism();
    let mut receipt = 0xcbf29ce484222325_u64;
    for reflected in [false, true] {
        let scale = if reflected {
            seam::ratio(1, 1000)
        } else {
            Real::one()
        };
        let map = if reflected {
            seam::similarity(&scale, true)
        } else {
            Matrix4::identity()
        };
        let moved = source.model().transformed(&map).unwrap();
        let replay = seam::reload(&moved);
        let output = analytic::export(
            &[StepBody {
                solid: replay.solids().next().unwrap(),
                name: "annular prism",
            }],
            &StepWriteOptions::new("annulus.step", "2026-10-16T00:00:00"),
        );
        assert!(
            output.bodies()[0]
                .faces()
                .iter()
                .all(|f| f.orientation == f.source.orientation())
        );
        assert!(
            output.bodies()[0]
                .faces()
                .iter()
                .any(|f| f.source.surface().kind() == SurfaceKind::Cylinder
                    && f.source.orientation() == Orientation::Reversed)
        );
        for byte in output.text().bytes() {
            receipt = (receipt ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
        let imported = analytic::reimport(&output);
        let body = imported.bodies()[0].solid();
        analytic::equal(
            &body.volume().unwrap(),
            &(Real::from(126) * Real::pi() * &scale * &scale * &scale),
        );
        let area = body
            .outer()
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b);
        analytic::equal(&area, &(Real::from(126) * Real::pi() * &scale * &scale));
        let delta = seam::ratio(1, 1_000_000) * seam::ratio(1, 1_000_000);
        for (x, expected) in [
            (Real::zero(), SolidPointLocation::Outside),
            (Real::from(2), SolidPointLocation::Boundary),
            (Real::from(2) - &delta, SolidPointLocation::Outside),
            (Real::from(2) + &delta, SolidPointLocation::Inside),
            (Real::from(5), SolidPointLocation::Boundary),
            (Real::from(5) + &delta, SolidPointLocation::Outside),
        ] {
            assert_eq!(
                body.classify_point(
                    &map.transform_point3(&Point3::new(x, 0.into(), 3.into()))
                        .unwrap()
                )
                .unwrap(),
                expected
            );
        }
        let edge = imported.bodies()[0]
            .model()
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
            .unwrap();
        let (a, b) = edge.domain().try_real_endpoints().unwrap();
        let split = edge.split(((a + b) * seam::ratio(1, 2)).into()).unwrap();
        let replay = seam::reload(split.model());
        analytic::equal(
            &replay.solids().next().unwrap().volume().unwrap(),
            &body.volume().unwrap(),
        );
        let set = BodySet::from_solid(body);
        assert!(boolean::difference(&set, &set).unwrap().is_empty());
    }
    receipt
}

pub fn malformed() {
    use hyperbrep_io::step::{ImportLimits, StepFile, StepReadOptions};
    let source = builder::cylinder(5.into(), 6.into()).unwrap();
    let cutter = builder::cuboid(seam::point(0, -20, 0), seam::point(20, 20, 6)).unwrap();
    let half = boolean::difference(&source, &cutter).unwrap();
    let output = analytic::export(
        &[StepBody {
            solid: half.bodies().next().unwrap(),
            name: "half",
        }],
        &StepWriteOptions::new("invalid.step", "2026-10-16T00:00:00"),
    );
    let face = output.bodies()[0]
        .faces()
        .iter()
        .find(|f| f.source.surface().kind() == SurfaceKind::Cylinder)
        .unwrap();
    let row = output
        .text()
        .lines()
        .find(|line| line.starts_with(&format!("#{}=", face.entity)))
        .unwrap();
    let flipped = if row.ends_with(".T.);") {
        row.replace(".T.);", ".F.);")
    } else {
        row.replace(".F.);", ".T.);")
    };
    let bad_orientation = output.text().replace(row, &flipped);
    // Changing an actual circle radius while retaining its vertices and trims
    // must fail the geometric image contract before solid admission.
    let circle = output
        .text()
        .lines()
        .find(|line| line.contains("=CIRCLE("))
        .unwrap();
    let cut = circle.rfind(',').unwrap();
    let bad_radius = output
        .text()
        .replace(circle, &format!("{},4.);", &circle[..cut]));
    for (index, text) in [bad_orientation, bad_radius].into_iter().enumerate() {
        let file = StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap();
        let error = file
            .import_representation(
                output.representation(),
                &ImportLimits::default(),
                &analytic::context(),
            )
            .result
            .unwrap()
            .unwrap_err();
        assert!(error.entity.is_some());
        if index == 0 {
            assert!(
                matches!(
                    error.kind,
                    hyperbrep_io::step::ErrorKind::Invalid(
                        "declared outer bound has inner orientation"
                    )
                ),
                "{error:?}"
            );
        } else {
            assert!(
                matches!(
                    error.kind,
                    hyperbrep_io::step::ErrorKind::Invalid(
                        "point is not on the exact source circle"
                    )
                ),
                "{error:?}"
            );
        }
        // Reusing the same public pipeline after rejection remains successful.
        analytic::reimport(&output);
    }
}

fn expected_import(
    imported: &hyperbrep_io::step::ImportedRepresentation,
    volumes: &[Real],
    area: &Real,
) {
    let mut actual = imported
        .bodies()
        .iter()
        .map(|b| b.solid().volume().unwrap())
        .collect::<Vec<_>>();
    assert_eq!(actual.len(), volumes.len());
    for expected in volumes {
        let i = actual
            .iter()
            .position(|a| {
                hyperlimit::compare_reals(a, expected, STRICT_PREDICATES).value()
                    == Some(std::cmp::Ordering::Equal)
            })
            .expect("exact imported component volume");
        actual.remove(i);
    }
    let sum = imported
        .bodies()
        .iter()
        .flat_map(|b| b.solid().outer().faces())
        .map(|f| f.area().unwrap())
        .fold(Real::zero(), |a, b| a + b);
    analytic::equal(&sum, area);
    for body in imported.bodies() {
        let model = seam::reload(body.model());
        analytic::equal(
            &model.solids().next().unwrap().volume().unwrap(),
            &body.solid().volume().unwrap(),
        );
        let set = BodySet::from_solid(model.solids().next().unwrap());
        assert!(boolean::difference(&set, &set).unwrap().is_empty());
        analytic::equal(
            &boolean::union(&BodySet::empty(), &set)
                .unwrap()
                .bodies()
                .next()
                .unwrap()
                .volume()
                .unwrap(),
            &body.solid().volume().unwrap(),
        );
    }
}

/// Planar fixtures already exported before this representation migration remain
/// standard STEP inputs. Dimensional recipes, rather than hashes, define the set.
pub fn planar_controls() -> u64 {
    use hyperbrep_io::step::{ImportLimits, StepFile, StepReadOptions};
    let q = seam::ratio;
    let p = seam::point;
    let delta = q(1, 1_000_000) * q(1, 1_000_000);
    let mut receipt = 0xcbf29ce484222325_u64;
    for (kind, old_text) in [
        (0, include_str!("../fixtures/prism-box-cover.step")),
        (1, include_str!("../fixtures/prism-box-tiny-upper-gap.step")),
        (2, include_str!("../fixtures/prism-strip-cover.step")),
        (3, include_str!("../fixtures/prism-ring-cover.step")),
    ] {
        let square = |r: i32| {
            [(-r, -r), (r, -r), (r, r), (-r, r)]
                .map(|(x, y)| Point2::new(x.into(), y.into()))
                .to_vec()
        };
        let source = if kind == 3 {
            builder::extrude_region(&square(5), &[square(2)], 0.into(), 6.into()).unwrap()
        } else {
            builder::cuboid(p(-5, -5, 0), p(5, 5, 6)).unwrap()
        };
        let cutter = builder::cuboid(
            p(if kind == 2 { -1 } else { 0 }, -20, -20),
            Point3::new(
                Real::from(if kind == 2 { 1 } else { 20 }),
                20.into(),
                if kind == 1 {
                    Real::from(6) - &delta
                } else {
                    20.into()
                },
            ),
        )
        .unwrap();
        let set = boolean::difference(&source, &cutter).unwrap();
        let (volumes, area) = match kind {
            0 => (vec![Real::from(300)], Real::from(280)),
            1 => (
                vec![Real::from(300) + Real::from(50) * &delta],
                Real::from(380) + Real::from(10) * &delta,
            ),
            2 => (vec![Real::from(240), Real::from(240)], Real::from(496)),
            3 => (vec![Real::from(252)], Real::from(288)),
            _ => unreachable!(),
        };
        let replay = seam::reload(set.model());
        let bodies = replay
            .solids()
            .map(|solid| StepBody {
                solid,
                name: "planar prism control",
            })
            .collect::<Vec<_>>();
        let output = analytic::export(
            &bodies,
            &StepWriteOptions::new("planar-prism.step", "2026-10-16T00:00:00"),
        );
        for text in [old_text, output.text()] {
            let file = StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap();
            let id = file
                .syntax()
                .entities()
                .find(|e| {
                    e.components()
                        .any(|c| c.name() == "ADVANCED_BREP_SHAPE_REPRESENTATION")
                })
                .unwrap()
                .id();
            let imported = file
                .import_representation(id, &ImportLimits::default(), &analytic::context())
                .result
                .unwrap()
                .unwrap();
            expected_import(&imported, &volumes, &area);
            let near_cap = Point3::new(1.into(), 0.into(), Real::from(6) - &delta * q(1, 2));
            let samples = match kind {
                0 => vec![
                    (p(-3, 0, 3), SolidPointLocation::Inside),
                    (p(1, 0, 3), SolidPointLocation::Outside),
                    (p(0, 0, 3), SolidPointLocation::Boundary),
                ],
                1 => vec![
                    (near_cap, SolidPointLocation::Inside),
                    (p(1, 0, 3), SolidPointLocation::Outside),
                ],
                2 => vec![
                    (p(-3, 0, 3), SolidPointLocation::Inside),
                    (p(3, 0, 3), SolidPointLocation::Inside),
                    (p(0, 0, 3), SolidPointLocation::Outside),
                ],
                3 => vec![
                    (p(-3, 0, 3), SolidPointLocation::Inside),
                    (p(-1, 0, 3), SolidPointLocation::Outside),
                    (p(3, 0, 3), SolidPointLocation::Outside),
                ],
                _ => unreachable!(),
            };
            for (point, expected) in samples {
                let locations = imported
                    .bodies()
                    .iter()
                    .map(|b| b.solid().classify_point(&point).unwrap())
                    .collect::<Vec<_>>();
                if expected == SolidPointLocation::Outside {
                    assert!(locations.iter().all(|x| *x == expected));
                } else {
                    assert!(locations.contains(&expected));
                }
            }
        }
        for byte in output.text().bytes() {
            receipt = (receipt ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    receipt
}

pub fn inward_shell_is_rejected() {
    use hyperbrep_io::step::{ErrorKind, ImportLimits, StepFile, StepReadOptions};
    let source = annular_prism();
    let output = analytic::export(
        &[StepBody {
            solid: source.bodies().next().unwrap(),
            name: "annulus",
        }],
        &StepWriteOptions::new("inward.step", "2026-10-16T00:00:00"),
    );
    let mut text = String::new();
    for line in output.text().split_inclusive('\n') {
        if line.contains("=ADVANCED_FACE(")
            || line.contains("=FACE_BOUND(")
            || line.contains("=FACE_OUTER_BOUND(")
        {
            let flipped = if line.trim_end().ends_with(".T.);") {
                line.replace(".T.);", ".F.);")
            } else {
                assert!(line.trim_end().ends_with(".F.);"));
                line.replace(".F.);", ".T.);")
            };
            text.push_str(&flipped);
        } else {
            text.push_str(line);
        }
    }
    let file = StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap();
    let error = file
        .import_representation(
            output.representation(),
            &ImportLimits::default(),
            &analytic::context(),
        )
        .result
        .unwrap()
        .unwrap_err();
    assert!(error.entity.is_some());
    assert!(
        matches!(
            error.kind,
            ErrorKind::Build(BuildError::InwardSolidShell(_))
        ),
        "{error:?}"
    );
    analytic::reimport(&output);
}

pub fn covering_circular_controls() -> u64 {
    use hyperbrep_io::step::{WriteErrorKind, write_advanced};
    let p = seam::point;
    let q = seam::ratio;
    let source = builder::cylinder(5.into(), 6.into()).unwrap();
    let mut receipt = 0xcbf29ce484222325_u64;
    for kind in 0..6 {
        let (c, s) = match kind {
            1 => (q(3, 5), q(4, 5)),
            2 => {
                let v = (Real::from(2).sqrt().unwrap() / Real::from(2)).unwrap();
                (v.clone(), v)
            }
            _ => (Real::one(), Real::zero()),
        };
        let frame = Matrix4::from_row_major([
            c.clone(),
            -&s,
            0.into(),
            0.into(),
            s,
            c,
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
        ]);
        let cutter = if kind == 3 || kind == 4 {
            builder::cylinder(2.into(), 40.into())
                .unwrap()
                .transformed(&Matrix4::affine_translation([
                    Real::from(i32::from(kind == 4)),
                    0.into(),
                    (-20).into(),
                ]))
                .unwrap()
        } else {
            builder::cuboid(p(0, -20, -20), p(20, 20, 20))
                .unwrap()
                .transformed(&frame)
                .unwrap()
        };
        let source = if kind == 5 {
            boolean::difference(&source, &builder::cylinder(2.into(), 6.into()).unwrap()).unwrap()
        } else {
            source.clone()
        };
        let set = boolean::difference(&source, &cutter).unwrap();
        let replay = seam::reload(set.model());
        let bodies = [StepBody {
            solid: replay.solids().next().unwrap(),
            name: "covering circular cut",
        }];
        let options = StepWriteOptions::new("covering-prism.step", "2026-10-16T00:00:00");
        if kind == 2 {
            let error = write_advanced(&bodies, &options, &analytic::context())
                .result
                .unwrap()
                .unwrap_err();
            assert!(matches!(error.kind, WriteErrorKind::NonRational));
            assert_eq!(error.body, Some(0));
            assert!(error.entity.is_some());
            continue;
        }
        let output = analytic::export(&bodies, &options);
        let imported = analytic::reimport(&output);
        let (volume, area) = match kind {
            0 | 1 => (
                Real::from(75) * Real::pi(),
                Real::from(55) * Real::pi() + Real::from(60),
            ),
            3 | 4 => (Real::from(126) * Real::pi(), Real::from(126) * Real::pi()),
            5 => (
                Real::from(63) * Real::pi(),
                Real::from(63) * Real::pi() + Real::from(36),
            ),
            _ => unreachable!(),
        };
        expected_import(&imported, std::slice::from_ref(&volume), &area);
        let body = imported.bodies()[0].solid();
        let samples = match kind {
            0 | 1 => vec![
                (p(-1, 0, 3), SolidPointLocation::Inside),
                (p(1, 0, 3), SolidPointLocation::Outside),
                (p(0, 0, 3), SolidPointLocation::Boundary),
            ],
            3 | 4 => {
                let x = i32::from(kind == 4);
                vec![
                    (p(x, 0, 3), SolidPointLocation::Outside),
                    (p(x + 2, 0, 3), SolidPointLocation::Boundary),
                    (p(-4, 0, 3), SolidPointLocation::Inside),
                ]
            }
            5 => vec![
                (p(-3, 0, 3), SolidPointLocation::Inside),
                (p(-1, 0, 3), SolidPointLocation::Outside),
                (p(3, 0, 3), SolidPointLocation::Outside),
            ],
            _ => unreachable!(),
        };
        for (point, location) in samples {
            assert_eq!(
                body.classify_point(&frame.transform_point3(&point).unwrap())
                    .unwrap(),
                location
            );
        }
        let edge = imported.bodies()[0]
            .model()
            .edges()
            .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
            .unwrap();
        let (a, b) = edge.domain().try_real_endpoints().unwrap();
        let split = edge.split(((a + b) * q(1, 2)).into()).unwrap();
        analytic::equal(
            &seam::reload(split.model())
                .solids()
                .next()
                .unwrap()
                .volume()
                .unwrap(),
            &volume,
        );
        let recut = boolean::difference(&BodySet::from_solid(body), &cutter).unwrap();
        analytic::equal(&recut.bodies().next().unwrap().volume().unwrap(), &volume);
        for byte in output.text().bytes() {
            receipt = (receipt ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    receipt
}
