#![no_main]

use hyperlimit::PredicatePolicy;
use hyperpath::{
    NetId, SpecctraGridRouteRuleRecord, SpecctraGridTraceRecord,
    SpecctraRouteRuleTraceClearanceStatus, TraceLayer, audit_specctra_trace_rule_clearances,
    specctra_grid_route_rule_record, specctra_grid_trace_record,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 7 {
        return;
    }

    let gap = i64::from(data[0] % 128);
    let first_width = i64::from(data[1] % 64);
    let second_width = i64::from(data[2] % 64);
    let clearance = i64::from(data[3] % 64);
    let first_layer = TraceLayer(u16::from(data[4] % 16));
    let same_layer = data[5] & 1 == 0;
    let same_net = data[6] & 1 == 1;
    let second_layer = if same_layer {
        first_layer
    } else {
        TraceLayer(first_layer.0.saturating_add(1))
    };
    let second_net = if same_net { NetId(1) } else { NetId(2) };

    let first = specctra_grid_trace_record(SpecctraGridTraceRecord {
        net: NetId(1),
        layer: first_layer,
        start_x: 0,
        start_y: 0,
        end_x: 32,
        end_y: 0,
        width: first_width,
        grid_denominator: 1,
    })
    .unwrap();
    let second = specctra_grid_trace_record(SpecctraGridTraceRecord {
        net: second_net,
        layer: second_layer,
        start_x: 0,
        start_y: gap,
        end_x: 32,
        end_y: gap,
        width: second_width,
        grid_denominator: 1,
    })
    .unwrap();
    let rule = specctra_grid_route_rule_record(SpecctraGridRouteRuleRecord {
        net: None,
        layer: None,
        clearance,
        width: 0,
        grid_denominator: 1,
    })
    .unwrap();

    let report =
        audit_specctra_trace_rule_clearances(&[first, second], &[rule], PredicatePolicy::default())
            .unwrap();
    assert_eq!(report.pairs.len(), 1);

    let expected_status = if !same_layer || same_net {
        SpecctraRouteRuleTraceClearanceStatus::NotApplicable
    } else if gap == 0 {
        SpecctraRouteRuleTraceClearanceStatus::NoShortViolation
    } else if 2 * gap < first_width + second_width + 2 * clearance {
        SpecctraRouteRuleTraceClearanceStatus::ClearanceViolation
    } else {
        SpecctraRouteRuleTraceClearanceStatus::CertifiedClear
    };
    assert_eq!(report.pairs[0].status, expected_status);
});
