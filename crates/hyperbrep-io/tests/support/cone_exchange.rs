//! Full-size exact cone exchange and native continuation, shared by consumers.
use hyperbrep::*;
use hyperbrep_io::step::*;
#[allow(dead_code)]
#[path = "analytic_export.rs"]
pub mod analytic;
pub use analytic::{equal, ratio as q};
#[path = "cone_inputs.rs"]
pub mod inputs;

#[derive(Clone, Copy, Debug)]
pub enum Part {
    Whole,
    Positive,
    Negative,
    Band,
    Lower,
}

pub fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
pub fn rotation() -> Matrix4 {
    Matrix4::from_row_major([
        q(3, 5),
        -q(4, 5),
        0.into(),
        0.into(),
        q(4, 5),
        q(3, 5),
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
    ])
}
pub fn cutter() -> BodySet {
    builder::cuboid(p(0, -20, -20), p(20, 20, 20))
        .unwrap()
        .transformed(&rotation())
        .unwrap()
}
pub fn measures(part: Part) -> (Real, Real) {
    let root = Real::from(5).sqrt().unwrap();
    let pi = Real::pi();
    match part {
        Part::Whole => (
            Real::from(78) * &pi,
            (Real::from(29) + Real::from(21) * root) * pi,
        ),
        Part::Positive | Part::Negative => (
            Real::from(39) * &pi,
            (Real::from(29) + Real::from(21) * root) * pi * q(1, 2) + Real::from(42),
        ),
        Part::Band => (
            q(665, 24) * &pi,
            (Real::from(97) + Real::from(65) * root) * pi * q(1, 8) + q(65, 2),
        ),
        Part::Lower => (
            q(271, 24) * &pi,
            (Real::from(181) + Real::from(19) * root) * pi * q(1, 8) + q(19, 2),
        ),
    }
}
pub fn verify(model: &Model, part: Part) {
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    assert_eq!(model.solids().count(), 1);
    let solid = model.solids().next().unwrap();
    let (volume, area) = measures(part);
    equal(&solid.volume().unwrap(), &volume);
    equal(
        &model
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        &area,
    );
    let (lo, hi) = match part {
        Part::Band => (1, 6),
        Part::Lower => (0, 1),
        _ => (0, 6),
    };
    let z = q(lo + hi, 2);
    let r = Real::from(5) - &z * q(1, 2);
    let tiny = q(1, 1_000_000) * q(1, 1_000_000);
    let positive = if matches!(part, Part::Negative) { O } else { I };
    let negative = if matches!(part, Part::Whole | Part::Negative) {
        I
    } else {
        O
    };
    let edge = if matches!(part, Part::Negative) { O } else { B };
    for (point, expected) in [
        (Point3::new(1.into(), 0.into(), z.clone()), positive),
        (Point3::new((-1).into(), 0.into(), z.clone()), negative),
        (
            Point3::new(0.into(), 1.into(), z.clone()),
            if matches!(part, Part::Whole) { I } else { B },
        ),
        (Point3::new(tiny.clone(), 1.into(), z.clone()), positive),
        (Point3::new(-&tiny, 1.into(), z.clone()), negative),
        (Point3::new(&r - &tiny, 0.into(), z.clone()), positive),
        (Point3::new(r.clone(), 0.into(), z.clone()), edge),
        (Point3::new(&r + &tiny, 0.into(), z), O),
        (Point3::new(1.into(), 0.into(), Real::from(lo) - &tiny), O),
        (p(1, 0, lo), edge),
        (Point3::new(1.into(), 0.into(), Real::from(hi) + &tiny), O),
        (p(1, 0, hi), edge),
    ] {
        assert_eq!(
            solid
                .classify_point(&rotation().transform_point3(&point).unwrap())
                .unwrap(),
            expected,
            "{part:?}"
        );
    }
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}
pub fn replay(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let replay = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replay.to_json().unwrap(), json);
    replay
}
pub fn round_trip(
    solid: SolidRef<'_>,
    name: &str,
    options: &StepWriteOptions<'_>,
) -> (BodySet, String) {
    let output = analytic::export(&[StepBody { solid, name }], options);
    let imported = analytic::reimport(&output);
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    analytic::source_maps(&file, &imported);
    let model = replay(imported.bodies()[0].model());
    (
        BodySet::from_solid(model.solids().next().unwrap()),
        output.into_text(),
    )
}
fn exchange(body: &BodySet, part: Part, name: &str) -> (BodySet, u64) {
    verify(body.model(), part);
    let (result, text) = round_trip(
        body.bodies().next().unwrap(),
        name,
        &StepWriteOptions::new("cone.step", "2026-10-17T00:00:00"),
    );
    verify(result.model(), part);
    let receipt = text.bytes().fold(0xcbf29ce484222325, |a, b| {
        (a ^ u64::from(b)).wrapping_mul(0x100000001b3)
    });
    (result, receipt)
}
pub fn complete_workflows() -> u64 {
    let source = builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap();
    let (source, mut receipt) = exchange(&source, Part::Whole, "whole");
    let cutter = cutter();
    let (half, r) = exchange(
        &boolean::intersection(&source, &cutter).unwrap(),
        Part::Positive,
        "positive",
    );
    receipt ^= r;
    let (other, r) = exchange(
        &boolean::difference(&source, &cutter).unwrap(),
        Part::Negative,
        "negative",
    );
    receipt ^= r;
    let axial = builder::cuboid(p(-20, -20, 1), p(20, 20, 20)).unwrap();
    let (band, r) = exchange(
        &boolean::intersection(&half, &axial).unwrap(),
        Part::Band,
        "band",
    );
    receipt ^= r;
    let (lower, r) = exchange(
        &boolean::difference(&half, &axial).unwrap(),
        Part::Lower,
        "lower",
    );
    receipt ^= r;
    for (a, b, part, name) in [
        (&half, &other, Part::Whole, "reunion-first"),
        (&other, &half, Part::Whole, "reunion-second"),
        (&band, &lower, Part::Positive, "band-reunion-first"),
        (&lower, &band, Part::Positive, "band-reunion-second"),
    ] {
        let (result, r) = exchange(&boolean::union(a, b).unwrap(), part, name);
        receipt ^= r;
        let empty = boolean::difference(&result, &result).unwrap();
        assert_eq!(empty.bodies().count(), 0);
        verify(boolean::union(&empty, &result).unwrap().model(), part);
        verify(boolean::difference(&result, &empty).unwrap().model(), part);
        assert_eq!(
            boolean::intersection(&result, &empty)
                .unwrap()
                .bodies()
                .count(),
            0
        );
        let recut = boolean::intersection(&result, &cutter).unwrap();
        verify(&replay(recut.model()), Part::Positive);
    }
    receipt
}

pub fn shifted_apex_and_units() {
    let body = builder::cone_frustum(5.into(), 2.into(), 7.into()).unwrap();
    for unit in [
        WriteLengthUnit {
            name: "metre",
            metres_per_unit: Real::one(),
        },
        WriteLengthUnit {
            name: "millimetre",
            metres_per_unit: q(1, 1000),
        },
    ] {
        let mut options = StepWriteOptions::new("shifted.step", "2026-10-17T00:00:00");
        options.length_unit = unit;
        let (result, _) = round_trip(
            body.bodies().next().unwrap(),
            "nonterminating apex",
            &options,
        );
        equal(
            &result.bodies().next().unwrap().volume().unwrap(),
            &(Real::from(91) * Real::pi()),
        );
        equal(
            &result
                .model()
                .faces()
                .map(|f| f.area().unwrap())
                .fold(Real::zero(), |a, b| a + b),
            &((Real::from(29) + Real::from(7) * Real::from(58).sqrt().unwrap()) * Real::pi()),
        );
    }
}

pub fn transformed_frames_and_explicit_encoding_limits() {
    let reflected = Matrix4::from_row_major([
        -q(6, 5),
        0.into(),
        q(8, 5),
        1.into(),
        0.into(),
        2.into(),
        0.into(),
        (-2).into(),
        q(8, 5),
        0.into(),
        q(6, 5),
        3.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    for height in [6, 7] {
        let source = builder::cone_frustum(5.into(), 2.into(), height.into()).unwrap();
        for (transform, scale) in [
            (rotation(), 1),
            (
                Matrix4::affine_translation([7.into(), (-3).into(), 2.into()]),
                1,
            ),
            (reflected.clone(), 2),
        ] {
            let transformed = source.transformed(&transform).unwrap();
            let (round, _) = round_trip(
                transformed.bodies().next().unwrap(),
                "transformed cone",
                &StepWriteOptions::new("frames.step", "2026-10-17T00:00:00"),
            );
            equal(
                &round.bodies().next().unwrap().volume().unwrap(),
                &(Real::from(if height == 6 { 78 } else { 91 })
                    * Real::pi()
                    * Real::from(scale * scale * scale)),
            );
            let area = (Real::from(29)
                + Real::from(7) * Real::from(9 + height * height).sqrt().unwrap())
                * Real::pi()
                * Real::from(scale * scale);
            equal(
                &round
                    .model()
                    .faces()
                    .map(|f| f.area().unwrap())
                    .fold(Real::zero(), |a, b| a + b),
                &area,
            );
            assert_eq!(
                round
                    .bodies()
                    .next()
                    .unwrap()
                    .classify_point(&transform.transform_point3(&p(0, 0, 3)).unwrap())
                    .unwrap(),
                SolidPointLocation::Inside
            );
        }
    }
    let source = builder::cone_frustum(5.into(), 2.into(), 6.into()).unwrap();
    let edge = source
        .model()
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::CircleArc)
        .unwrap();
    let (a, b) = edge.domain().try_real_endpoints().unwrap();
    let split = edge.split(((a + b) * q(1, 2)).into()).unwrap().into_model();
    assert_eq!(
        split.counts().vertices,
        source.model().counts().vertices + 1
    );
    let split = replay(&split);
    verify(&split, Part::Whole);
    let error = write_advanced(
        &[StepBody {
            solid: split.solids().next().unwrap(),
            name: "irrational split",
        }],
        &StepWriteOptions::new("split.step", "2026-10-17T00:00:00"),
        &analytic::context(),
    )
    .result
    .unwrap()
    .unwrap_err();
    assert!(
        matches!(error.kind, WriteErrorKind::NonRational),
        "{error:?}"
    );
    assert_eq!(error.body, Some(0));
    assert!(error.entity.is_some());
    let body = BodySet::from_solid(split.solids().next().unwrap());
    verify(boolean::union(&body, &source).unwrap().model(), Part::Whole);
    assert_eq!(
        boolean::difference(&body, &source)
            .unwrap()
            .bodies()
            .count(),
        0
    );
    exchange(&source, Part::Whole, "recovery");
}

pub fn manual_line_inputs() {
    for (text, height) in [
        (include_str!("../fixtures/cone-unit-line.step"), 6),
        (include_str!("../fixtures/cone-shifted-unit-line.step"), 7),
    ] {
        let file = StepFile::read(text.as_bytes(), &StepReadOptions::default()).unwrap();
        let imported = file
            .import_representation(60, &ImportLimits::default(), &analytic::context())
            .result
            .unwrap()
            .unwrap();
        analytic::source_maps(&file, &imported);
        let body = &imported.bodies()[0];
        let volume = Real::from(if height == 6 { 78 } else { 91 })
            * Real::pi()
            * q(1, 1000)
            * q(1, 1000)
            * q(1, 1000);
        equal(&body.solid().volume().unwrap(), &volume);
        assert_eq!(
            (
                body.model().counts().vertices,
                body.model().counts().edges,
                body.model().counts().faces
            ),
            (2, 3, 3)
        );
        let (round, _) = round_trip(
            body.solid(),
            "manual",
            &StepWriteOptions::new("manual.step", "2026-10-17T00:00:00"),
        );
        equal(&round.bodies().next().unwrap().volume().unwrap(), &volume);
    }
}
