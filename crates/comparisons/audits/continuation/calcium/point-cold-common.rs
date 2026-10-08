include!("point-history-base.rs");

pub fn collect_cold_sequence(
    which: usize,
    policy: usize,
    initial_history: usize,
    lifecycle: usize,
) -> Vec<u8> {
    assert!(which < 48 && policy < 2 && initial_history < 4 && lifecycle < 3);
    let name = ["retained", "fresh", "roundtrip"][lifecycle];
    let prepare = || {
        let (mut a, mut b, op) = case(which);
        history(&mut a, initial_history);
        history(&mut b, initial_history);
        (a, b, op)
    };
    let mut inputs: Option<(Root, Root, Op)> = None;
    let mut output = Vec::new();
    for call in 0..9 {
        if lifecycle == 1 {
            // Fresh inputs do not coexist with the prior iteration's graphs.
            drop(inputs.take());
        }
        if let Some((a, b, _)) = inputs.as_mut() {
            if lifecycle == 2 {
                history(a, 3);
                history(b, 3);
            }
        } else {
            inputs = Some(prepare());
        }
        let (a, b, op) = inputs.as_ref().unwrap();
        let before_a = wire_root(a);
        let before_b = wire_root(b);
        // Exactly one public query per recorded call, including the first.
        let report = wire_report(&query(a, b, *op, policy));
        assert_eq!(before_a, wire_root(a));
        assert_eq!(before_b, wire_root(b));
        let row = json!({"type":"query","case":which,"policy":policy,
            "history":initial_history,"lifecycle":name,"call":call,
            "left":before_a,"right":before_b,"report":report});
        serde_json::to_writer(&mut output, &row).unwrap();
        output.push(b'\n');
    }
    serde_json::to_writer(
        &mut output,
        &json!({"type":"terminal","case":which,
        "policy":policy,"history":initial_history,"lifecycle":name,"calls":9}),
    )
    .unwrap();
    output.push(b'\n');
    output
}
