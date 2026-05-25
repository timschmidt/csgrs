#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    LinePathSegment, MeanderKeepout, MeanderObstacle, OffsetSide,
    build_keepout_aware_detour_meander, classify_meander_placement_slots_with_keepouts,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn positive(byte: u8, max: i64) -> i64 {
    i64::from(byte % u8::try_from(max).unwrap()) + 1
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 5 {
        return;
    }

    let length = 40 + positive(data[0], 80);
    let amplitude = positive(data[1], 20);
    let bump_count = u64::from(data[2] % 8) + 1;
    let radius = positive(data[3], amplitude.max(1));
    let side = if data[4] & 1 == 0 {
        OffsetSide::Left
    } else {
        OffsetSide::Right
    };
    let source = LinePathSegment::new(p(0, 0), p(length, 0));
    let center_y = match side {
        OffsetSide::Left => amplitude,
        OffsetSide::Right => -amplitude,
    };
    let keepouts = vec![
        MeanderKeepout::Circular {
            center: p(5, center_y),
            radius: r(radius),
        },
        MeanderKeepout::OrthogonalPolygon {
            vertices: vec![
                p(length + 30, length + 30),
                p(length + 40, length + 30),
                p(length + 40, length + 35),
                p(length + 35, length + 35),
                p(length + 35, length + 40),
                p(length + 30, length + 40),
            ],
        },
        MeanderKeepout::Rectangular(MeanderObstacle {
            min: p(length + 10, length + 10),
            max: p(length + 20, length + 20),
        }),
    ];
    let report = classify_meander_placement_slots_with_keepouts(
        &source,
        r(amplitude),
        bump_count,
        side,
        keepouts.clone(),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(report.slots.len(), bump_count as usize);
    assert!(report.slots[0].preferred_blocked);

    let routed = build_keepout_aware_detour_meander(
        &source,
        r(2 * amplitude * i64::try_from(bump_count).unwrap()),
        bump_count,
        side,
        keepouts,
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(routed.selected_sides.len(), bump_count as usize);
    assert_eq!(routed.selected_sides[0], opposite(side));

    let diagonal = LinePathSegment::new(p(0, 0), p(3, 4));
    assert_eq!(
        build_keepout_aware_detour_meander(
            &diagonal,
            r(2),
            1,
            side,
            Vec::new(),
            PredicatePolicy::default()
        )
        .unwrap_err(),
        hyperpath::MeanderError::UnsupportedSourceGeometry
    );
});

fn opposite(side: OffsetSide) -> OffsetSide {
    match side {
        OffsetSide::Left => OffsetSide::Right,
        OffsetSide::Right => OffsetSide::Left,
    }
}
