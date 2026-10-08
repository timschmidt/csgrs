include!("point-history-base.rs");

fn main() {
    let mut rows = 0;
    for which in 0..48 {
        for policy in 0..2 {
            for state in 0..4 {
                let (mut a, mut b, op) = case(which);
                history(&mut a, state);
                history(&mut b, state);
                let before_a = wire_root(&a);
                let before_b = wire_root(&b);
                let report = wire_report(&query(&a, &b, op, policy));
                assert_eq!(before_a, wire_root(&a));
                assert_eq!(before_b, wire_root(&b));
                println!(
                    "{}",
                    json!({"type":"query","case":which,"policy":policy,"history":state,
                    "left":before_a,"right":before_b,"report":report})
                );
                rows += 1;
            }
        }
    }
    println!(
        "{}",
        json!({"type":"terminal","rows":rows,"cases":48,"policies":2,"histories":4})
    );
}
