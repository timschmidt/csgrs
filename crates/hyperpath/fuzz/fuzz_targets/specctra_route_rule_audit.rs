#![no_main]

use hyperlimit::PredicatePolicy;
use hyperpath::{
    NetId, SpecctraGridRouteRuleRecord, SpecctraGridTraceRecord, SpecctraRouteRuleWidthStatus,
    TraceLayer, audit_specctra_route_rule_widths, specctra_grid_route_rule_record,
    specctra_grid_trace_record,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 7 {
        return;
    }

    let net = NetId(u32::from(data[0]));
    let layer = TraceLayer(u16::from(data[1] % 16));
    let denominator = u64::from(data[2] % 32) + 1;
    let trace_width = i64::from(data[3] % 64);
    let rule_width = i64::from(data[4] % 64);
    let clearance = i64::from(data[5] % 64);
    let scoped = data[6] & 1 == 1;

    let trace = specctra_grid_trace_record(SpecctraGridTraceRecord {
        net,
        layer,
        start_x: 0,
        start_y: 0,
        end_x: 10,
        end_y: 0,
        width: trace_width,
        grid_denominator: denominator,
    })
    .unwrap();
    let rule = specctra_grid_route_rule_record(SpecctraGridRouteRuleRecord {
        net: scoped.then_some(net),
        layer: scoped.then_some(layer),
        clearance,
        width: rule_width,
        grid_denominator: denominator,
    })
    .unwrap();
    let report = audit_specctra_route_rule_widths(
        std::slice::from_ref(&trace),
        &[],
        std::slice::from_ref(&rule),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(report.items.len(), 1);
    assert_eq!(
        report.items[0].width_status,
        if trace_width >= rule_width {
            SpecctraRouteRuleWidthStatus::Certified
        } else {
            SpecctraRouteRuleWidthStatus::Violation
        }
    );
});
