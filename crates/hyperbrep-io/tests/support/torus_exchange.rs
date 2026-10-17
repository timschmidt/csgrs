//! Exact torus exchange and further modeling, shared by native/WASM consumers.
use hyperbrep::*;
use hyperbrep_io::step::*;
#[allow(dead_code)]
#[path = "analytic_export.rs"]
pub mod analytic;
#[allow(dead_code)]
#[path = "../../../hyperbrep/tests/support/sectors.rs"]
pub mod sectors;
pub use analytic::{equal, ratio};
pub use sectors::seam;

fn receipt(text: &str) -> u64 {
    text.bytes().fold(0xcbf29ce484222325, |a, b| {
        (a ^ u64::from(b)).wrapping_mul(0x100000001b3)
    })
}

pub fn verify(model: &Model, volume: &Real, area: &Real, samples: &[(Point3, SolidPointLocation)]) {
    let solids = model.solids().collect::<Vec<_>>();
    assert_eq!(solids.len(), 1);
    equal(&solids[0].volume().unwrap(), volume);
    equal(
        &model
            .faces()
            .map(|f| f.area().unwrap())
            .fold(Real::zero(), |a, b| a + b),
        area,
    );
    for (point, expected) in samples {
        assert_eq!(
            solids[0].classify_point(point).unwrap(),
            *expected,
            "{point:?}"
        );
    }
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

pub fn round_trip(
    solid: SolidRef<'_>,
    options: &StepWriteOptions<'_>,
    volume: &Real,
    area: &Real,
    samples: &[(Point3, SolidPointLocation)],
) -> (BodySet, String) {
    let output = analytic::export(
        &[StepBody {
            solid,
            name: "torus",
        }],
        options,
    );
    let imported = analytic::reimport(&output);
    let model = imported.bodies()[0].model();
    verify(model, volume, area, samples);
    let replay = seam::reload(model);
    verify(&replay, volume, area, samples);
    let edge = replay.edges().next().unwrap();
    let (a, b) = edge.domain().try_real_endpoints().unwrap();
    let split = edge.split(((a + b) * ratio(1, 2)).into()).unwrap();
    verify(&seam::reload(split.model()), volume, area, samples);
    (
        BodySet::from_solid(replay.solids().next().unwrap()),
        output.into_text(),
    )
}

pub fn whole() -> u64 {
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    let source = builder::torus(3.into(), 1.into()).unwrap();
    let tiny = ratio(1, 1_000_000) * ratio(1, 1_000_000);
    let samples = [
        (Point3::origin(), O),
        (seam::point(3, 0, 0), I),
        (seam::point(2, 0, 0), B),
        (seam::point(4, 0, 0), B),
        (seam::point(3, 0, 1), B),
        (Point3::new(Real::from(4) + &tiny, 0.into(), 0.into()), O),
        (Point3::new(Real::from(4) - &tiny, 0.into(), 0.into()), I),
        (Point3::new(Real::from(2) - &tiny, 0.into(), 0.into()), O),
        (Point3::new(Real::from(2) + &tiny, 0.into(), 0.into()), I),
    ];
    let mut hash = 0_u64;
    for (scale, reflected) in [
        (Real::one(), false),
        (ratio(1, 1000), false),
        (ratio(1, 1000), true),
    ] {
        let map = if scale == Real::one() {
            Matrix4::identity()
        } else {
            seam::similarity(&scale, reflected)
        };
        let moved = source.transformed(&map).unwrap();
        let samples = samples
            .iter()
            .map(|(p, c)| (map.transform_point3(p).unwrap(), *c))
            .collect::<Vec<_>>();
        let volume = Real::from(6) * Real::pi() * Real::pi() * &scale * &scale * &scale;
        let area = Real::from(12) * Real::pi() * Real::pi() * &scale * &scale;
        for output_scale in [Real::one(), ratio(1, 1000)] {
            let mut options = StepWriteOptions::new("torus.step", "2026-10-16T00:00:00");
            options.length_unit.metres_per_unit = output_scale;
            let (reimported, text) = round_trip(
                moved.bodies().next().unwrap(),
                &options,
                &volume,
                &area,
                &samples,
            );
            assert!(
                boolean::difference(&reimported, &reimported)
                    .unwrap()
                    .is_empty()
            );
            verify(
                boolean::union(&BodySet::empty(), &reimported)
                    .unwrap()
                    .model(),
                &volume,
                &area,
                &samples,
            );
            hash = hash.wrapping_mul(0x100000001b3) ^ receipt(&text);
        }
    }
    hash
}

pub fn halves_and_reconstruction() -> u64 {
    let mut hash = 0_u64;
    let options = StepWriteOptions::new("halves.step", "2026-10-16T00:00:00");
    for kind in 0..2 {
        let (source, cutter, volume, area) = seam::fixture(kind);
        let mut imported = Vec::new();
        for difference in [false, true] {
            let half = if difference {
                boolean::difference(&source, &cutter)
            } else {
                boolean::intersection(&source, &cutter)
            }
            .unwrap();
            let (half, text) =
                round_trip(half.bodies().next().unwrap(), &options, &volume, &area, &[]);
            if !difference {
                assert!(boolean::difference(&half, &cutter).unwrap().is_empty());
                verify(
                    boolean::intersection(&half, &cutter).unwrap().model(),
                    &volume,
                    &area,
                    &[],
                );
            }
            imported.push(half);
            hash = hash.wrapping_mul(0x100000001b3) ^ receipt(&text);
        }
        for reverse in [false, true] {
            let (a, b) = if reverse {
                (&imported[1], &imported[0])
            } else {
                (&imported[0], &imported[1])
            };
            let joined = boolean::union(a, b).unwrap();
            let full_volume = Real::from(40) * Real::pi() * Real::pi();
            let full_area = Real::from(40) * Real::pi() * Real::pi();
            let (_, text) = round_trip(
                joined.bodies().next().unwrap(),
                &options,
                &full_volume,
                &full_area,
                &[],
            );
            hash = hash.wrapping_mul(0x100000001b3) ^ receipt(&text);
        }
    }
    hash
}

pub fn rational_band() -> u64 {
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    let source = builder::torus(10.into(), 5.into()).unwrap();
    let cutter = builder::cuboid(seam::point(-40, -40, 3), seam::point(40, 40, 40)).unwrap();
    let cap = boolean::intersection(&source, &cutter).unwrap();
    let a = ratio(3, 4).atan().unwrap();
    // Integrate the annular cross-section 4*pi*R*sqrt(r^2-z^2).
    let volume = Real::from(20)
        * Real::pi()
        * (Real::from(25) * (Real::pi() * ratio(1, 2) - ratio(3, 5).asin().unwrap())
            - Real::from(12));
    // Torus area between v=a and pi-a, plus its planar annular cap.
    let area = Real::from(100) * Real::pi() * (Real::pi() - Real::from(2) * a)
        + Real::from(160) * Real::pi();
    let tiny = ratio(1, 1_000_000) * ratio(1, 1_000_000);
    let samples = [
        (seam::point(10, 0, 4), I),
        (seam::point(10, 0, 3), B),
        (seam::point(10, 0, 5), B),
        (seam::point(6, 0, 3), B),
        (seam::point(14, 0, 3), B),
        (Point3::new(10.into(), 0.into(), Real::from(3) + &tiny), I),
        (Point3::new(10.into(), 0.into(), Real::from(3) - &tiny), O),
        (Point3::new(10.into(), 0.into(), Real::from(5) + &tiny), O),
    ];
    verify(cap.model(), &volume, &area, &samples);
    // Native subdivision/replay is part of the completed workflow, before the
    // independent STEP chart mapping. Measure this continuation in the bench.
    let mut native = seam::reload(cap.model());
    for _ in 0..4 {
        let edge = native.edges().next().unwrap();
        let (start, end) = edge.domain().try_real_endpoints().unwrap();
        let split = edge.split(((start + end) * ratio(1, 2)).into()).unwrap();
        native = seam::reload(split.model());
        equal(&native.solids().next().unwrap().volume().unwrap(), &volume);
    }
    let options = StepWriteOptions::new("band.step", "2026-10-16T00:00:00");
    let (imported, text) = round_trip(
        cap.bodies().next().unwrap(),
        &options,
        &volume,
        &area,
        &samples,
    );
    assert!(boolean::difference(&imported, &cutter).unwrap().is_empty());
    verify(
        boolean::intersection(&imported, &cutter).unwrap().model(),
        &volume,
        &area,
        &samples,
    );
    receipt(&text)
}
