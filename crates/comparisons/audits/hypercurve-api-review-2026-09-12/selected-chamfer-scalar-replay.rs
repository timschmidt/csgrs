use hyperreal::Real;

fn main() {
    let path = std::env::args().nth(1).expect("one exact scalar replay file");
    let data = std::fs::read_to_string(path).expect("read scalar replay");
    assert!(data.len() <= 1024 * 1024);
    let value = Real::from_json(&data).expect("decode exact scalar expression");
    let start = std::time::Instant::now();
    println!(
        "bytes={} immediate={:?} zero={:?} tower={:?} refinement={:?} elapsed_ms={}",
        data.len(),
        value.immediate_sign(),
        value.zero_status(),
        value.quadratic_tower_sign(),
        value.refine_sign_until(-512),
        start.elapsed().as_millis(),
    );
}
