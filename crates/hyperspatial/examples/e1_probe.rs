//! Isolated native workflow probe for the development lexicase driver.

use std::time::Instant;

use hyperreal::dispatch_trace;
use serde_json::json;

#[path = "../../hyperreal/tests/support/representations.rs"]
mod scalar_cases;

#[path = "../dev/normal_offset.rs"]
mod normal_offset;

#[path = "../dev/mercator.rs"]
mod mercator;

#[path = "../dev/inverse_isometric.rs"]
mod inverse_isometric;

fn main() {
    let encoded = std::env::args()
        .nth(1)
        .expect("provide a 12-digit hex case");
    assert_eq!(encoded.len(), 12, "six input bytes required");
    assert!(encoded.is_ascii(), "ASCII hexadecimal input required");
    let mut data = [0; 6];
    for (index, byte) in data.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&encoded[2 * index..2 * index + 2], 16).expect("hex byte");
    }
    let workload = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "normal_offset".into());
    dispatch_trace::reset();
    let start = Instant::now();
    let result = dispatch_trace::with_recording(|| match workload.as_str() {
        "normal_offset" => normal_offset::exercise(data),
        "mercator" => mercator::exercise(data),
        "inverse_isometric" => inverse_isometric::exercise(data),
        _ => panic!("unknown workload {workload}"),
    });
    let evaluation_ns = start.elapsed().as_nanos();
    let trace = dispatch_trace::take();
    let dispatch_count: u64 = trace.iter().map(|row| row.count).sum();
    let paths: Vec<_> = trace
        .iter()
        .map(|row| {
            json!({
                "layer": row.layer, "operation": row.operation, "path": row.path, "count": row.count
            })
        })
        .collect();
    match result {
        Ok((steps, depth, first, second)) => println!(
            "{}",
            json!({
                "status": "certified_completion", "case": encoded,
                "families": [first, second], "steps": steps, "receipt_depth": depth,
                "evaluation_ns": evaluation_ns, "dispatch_count": dispatch_count, "dispatch": paths
            })
        ),
        Err(error) => {
            eprintln!("{error}");
            println!(
                "{}",
                json!({"status": "native_error", "case": encoded,
                "error": format!("{error:?}"), "evaluation_ns": evaluation_ns, "dispatch": paths})
            );
            std::process::exit(2);
        }
    }
}
