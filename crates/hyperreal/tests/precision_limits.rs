use hyperreal::Computable;
use num::{BigInt, Zero};

#[test]
fn shared_constants_answer_cold_maximum_precision() {
    const CHILD: &str = "HYPERREAL_PRECISION_BOUNDARY_CHILD";
    if let Ok(constant) = std::env::var(CHILD) {
        let value = match constant.as_str() {
            "pi" => Computable::pi(),
            "tau" => Computable::tau(),
            _ => panic!("unexpected precision probe constant"),
        };
        assert!((BigInt::zero()..=BigInt::from(1)).contains(&value.approx(i32::MAX)));
        return;
    }

    // Other tests can warm the process-wide caches and hide a cold lookup
    // overflow. Each constant therefore runs in its own fresh test process.
    for constant in ["pi", "tau"] {
        let status = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "shared_constants_answer_cold_maximum_precision",
                "--test-threads=1",
            ])
            .env(CHILD, constant)
            .status()
            .expect("run the cold precision probe");
        assert!(status.success(), "cold {constant} approximation failed");
    }
}
