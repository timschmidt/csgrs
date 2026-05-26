#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    NetId, PcbViaStack, TraceLayer, ViaAnnularRingReport, ViaAspectRatioReport,
    ViaDrillIntent, ViaFabricationAcceptance, ViaFabricationPolicy,
    certify_via_fabrication_policy,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }

    let board_layers = u16::from(data[0] % 8) + 1;
    let mut start = u16::from(data[1] % 8);
    let mut end = u16::from(data[2] % 8);
    if start > end {
        std::mem::swap(&mut start, &mut end);
    }
    let land = i64::from(data[3] % 80);
    let drill = i64::from(data[4] % 64);
    let board_thickness = i64::from(data[5] % 96) + 1;
    let minimum = i64::from(data[6] % 24);
    let aspect = i64::from(data[7] % 16) + 1;
    let intent = match data[8] % 3 {
        0 => ViaDrillIntent::Plated,
        1 => ViaDrillIntent::NonPlated,
        _ => ViaDrillIntent::Unspecified,
    };
    let allow_all = data[9] & 1 == 1;

    let via = PcbViaStack::with_drill_intent(
        NetId(1),
        TraceLayer(start),
        TraceLayer(end),
        p(0, 0),
        r(land),
        r(drill),
        intent,
    )
    .unwrap();
    let policy = if allow_all {
        ViaFabricationPolicy::all_transitions(
            board_layers,
            r(board_thickness),
            r(minimum),
            r(aspect),
        )
    } else {
        ViaFabricationPolicy::through_only(
            board_layers,
            r(board_thickness),
            r(minimum),
            r(aspect),
        )
    };

    let Ok(report) = certify_via_fabrication_policy(&via, &policy, PredicatePolicy::default()) else {
        return;
    };
    let annular_ok = land >= drill + 2 * minimum;
    let aspect_ok = board_thickness <= drill * aspect;
    assert_eq!(
        report.aspect_ratio,
        if aspect_ok {
            ViaAspectRatioReport::Certified
        } else {
            ViaAspectRatioReport::Violation
        }
    );
    if intent == ViaDrillIntent::Plated {
        assert_eq!(
            report.drill_policy.annular_ring,
            Some(if annular_ok {
                ViaAnnularRingReport::Certified
            } else {
                ViaAnnularRingReport::Violation
            })
        );
    }
    if matches!(report.acceptance, ViaFabricationAcceptance::Accepted) {
        assert!(report.transition_policy.allowed);
        assert!(aspect_ok);
        assert!(intent == ViaDrillIntent::Plated || report.transition_policy.transition.spanned_layers == 1);
        if intent == ViaDrillIntent::Plated {
            assert!(annular_ok);
        }
    }
});
