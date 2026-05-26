#![no_main]

use hyperpath::{
    NetId, SpecctraGridRouteRuleRecord, TraceLayer, parse_specctra_grid_route_records,
    serialize_specctra_grid_route_rule_records, specctra_grid_route_rule_record,
};
use libfuzzer_sys::fuzz_target;

fn signed(byte: u8) -> i64 {
    i64::from(i8::from_ne_bytes([byte]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }

    if let Ok(text) = std::str::from_utf8(data) {
        let _ = parse_specctra_grid_route_records(text);
    }

    let rule = SpecctraGridRouteRuleRecord {
        net: if data[0] & 1 == 0 {
            Some(NetId(u32::from(data[1])))
        } else {
            None
        },
        layer: if data[0] & 2 == 0 {
            Some(TraceLayer(u16::from(data[2] % 32)))
        } else {
            None
        },
        clearance: signed(data[3]),
        width: signed(data[4]),
        grid_denominator: u64::from(data[5] % 32),
    };

    let text = serialize_specctra_grid_route_rule_records(&[rule]);
    let parsed = parse_specctra_grid_route_records(&text);
    let exact = specctra_grid_route_rule_record(rule);
    if rule.grid_denominator == 0 || rule.clearance < 0 || rule.width < 0 {
        assert!(parsed.is_err());
        assert!(exact.is_err());
    } else {
        assert_eq!(parsed.unwrap().rules, vec![rule]);
        assert_eq!(exact.unwrap().net, rule.net);
    }
});
