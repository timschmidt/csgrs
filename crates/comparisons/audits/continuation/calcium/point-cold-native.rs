include!("point-cold-common.rs");

fn main() {
    use std::io::Write;
    let args = std::env::args().collect::<Vec<_>>();
    assert_eq!(args.len(), 5);
    let values = args[1..]
        .iter()
        .map(|s| s.parse::<usize>().unwrap())
        .collect::<Vec<_>>();
    std::io::stdout()
        .lock()
        .write_all(&collect_cold_sequence(
            values[0], values[1], values[2], values[3],
        ))
        .unwrap();
}
