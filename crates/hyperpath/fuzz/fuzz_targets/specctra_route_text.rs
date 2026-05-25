#![no_main]

use hyperpath::{NetId, parse_specctra_grid_trace_records};
use hyperpath::{
    SpecctraGridKeepoutRecord, SpecctraGridKeepoutShape, SpecctraGridRouteRecords,
    SpecctraGridTraceRecord, SpecctraGridViaRecord, SpecctraLayerAlias, SpecctraNetAlias,
    TraceLayer, ViaDrillIntent, import_specctra_text_route, parse_specctra_grid_route_records,
    serialize_specctra_grid_route_records,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Ok(text) = std::str::from_utf8(data) {
        let _ = parse_specctra_grid_route_records(text);
        let _ = parse_specctra_grid_trace_records(text);
        let _ = import_specctra_text_route(text);
    }

    if data.len() >= 16 {
        let net = NetId(u32::from(data[0]));
        let layer = TraceLayer(u16::from(data[1] % 16));
        let denominator = u64::from(data[2] % 31) + 1;
        let width = i64::from(data[3] % 32);
        let x0 = i64::from(i8::from_ne_bytes([data[4]]));
        let y0 = i64::from(i8::from_ne_bytes([data[5]]));
        let x1 = i64::from(i8::from_ne_bytes([data[6]]));
        let y1 = i64::from(i8::from_ne_bytes([data[7]]));
        let x2 = i64::from(i8::from_ne_bytes([data[8]]));
        let y2 = i64::from(i8::from_ne_bytes([data[9]]));
        let route = SpecctraGridRouteRecords {
            net_aliases: vec![SpecctraNetAlias {
                net,
                name: format!("NET {}", data[10]),
            }],
            layer_aliases: vec![SpecctraLayerAlias {
                layer,
                name: format!("LAYER {}", data[11]),
            }],
            traces: vec![SpecctraGridTraceRecord {
                net,
                layer,
                start_x: x0,
                start_y: y0,
                end_x: x1,
                end_y: y1,
                width,
                grid_denominator: denominator,
            }],
            vias: vec![SpecctraGridViaRecord {
                net,
                start_layer: TraceLayer(0),
                end_layer: layer,
                x: x2,
                y: y2,
                land_diameter: i64::from(data[12] % 64),
                drill_diameter: i64::from(data[13] % 32),
                drill_intent: match data[14] % 3 {
                    0 => ViaDrillIntent::Unspecified,
                    1 => ViaDrillIntent::Plated,
                    _ => ViaDrillIntent::NonPlated,
                },
                grid_denominator: denominator,
            }],
            keepouts: vec![SpecctraGridKeepoutRecord {
                layer: Some(layer),
                shape: SpecctraGridKeepoutShape::Polygon {
                    vertices: vec![
                        (x2, y2),
                        (x2 + i64::from(data[15] % 16) + 1, y2),
                        (
                            x2 + i64::from(data[15] % 16) + 1,
                            y2 + i64::from(data[3] % 16) + 1,
                        ),
                        (x2, y2 + i64::from(data[3] % 16) + 1),
                    ],
                },
                grid_denominator: denominator,
            }],
        };
        let text = serialize_specctra_grid_route_records(&route);
        let reparsed = parse_specctra_grid_route_records(&text).unwrap();
        assert_eq!(reparsed, route);

        let path_text = format!(
            "(session fuzz (routes (wire (net {}) (path {} {} {} {} {} {} {} {}) (grid {}))))",
            net.0, layer.0, width, x0, y0, x1, y1, x2, y2, denominator
        );
        let parsed = parse_specctra_grid_route_records(&path_text).unwrap();
        assert_eq!(parsed.traces.len(), 2);
    }
});
