use hyperreal::{CertifiedRealEquality, Real};
use serde_json::{Value, json};
use std::hint::black_box;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

#[allow(dead_code)]
mod inputs {
    include!("qqbar-trig-hyper-v76/src/main.rs");

    fn trig_identity(x: Real, pi: bool) -> Real {
        let (s, c) = if pi {
            (x.clone().sin_pi(), x.cos_pi())
        } else {
            (x.clone().sin(), x.cos())
        };
        &s * &s + &c * &c
    }

    pub fn pair(c: &serde_json::Value) -> (Real, Real) {
        let k = c["k"].as_i64().unwrap() as i32;
        let op = c["op"].as_str().unwrap();
        match c["family"].as_str().unwrap() {
            "twelfth" | "known-angle" => (actual(k, 0, op).unwrap(), radical(k, op)),
            "perturbed" => (
                actual(k, 0, op).unwrap(),
                radical(k, op) + Real::from(Rational::fraction(1, 1024).unwrap()),
            ),
            "periodic" => (actual(k, 0, op).unwrap(), actual(k, 1, op).unwrap()),
            "seventh-identity" => (
                trig_identity(Real::from(Rational::fraction(1, 7).unwrap()), true),
                Real::one(),
            ),
            "rational-trig-identity" => (
                trig_identity(Real::from(Rational::fraction(1, 7).unwrap()), false),
                Real::one(),
            ),
            "surd-trig-identity" => (
                trig_identity(
                    (Real::from(5).sqrt().unwrap() / Real::from(8)).unwrap(),
                    false,
                ),
                Real::one(),
            ),
            "exp-identity" => {
                let x = Real::from(2).sqrt().unwrap();
                let y = Real::from(3).sqrt().unwrap();
                (
                    (&x + &y).exp().unwrap(),
                    x.exp().unwrap() * y.exp().unwrap(),
                )
            }
            "rational-equal" => (
                Real::from(Rational::fraction(3, 7).unwrap()),
                Real::from(Rational::fraction(3, 7).unwrap()),
            ),
            "rational-unequal" => (
                Real::from(Rational::fraction(3, 7).unwrap()),
                Real::from(Rational::fraction(4, 7).unwrap()),
            ),
            "quadratic-identity" => (
                Real::from(2).sqrt().unwrap() + Real::from(2).sqrt().unwrap(),
                Real::from(8).sqrt().unwrap(),
            ),
            "nested-surd-identity" => (
                Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
                (Real::from(5) + Real::from(2) * Real::from(6).sqrt().unwrap())
                    .sqrt()
                    .unwrap(),
            ),
            "deep-seventh-identity" => {
                let count = c["count"].as_i64().unwrap();
                let mut sum = Real::zero();
                for n in 0..count {
                    sum += trig_identity(
                        Real::from(Rational::fraction(1, (7 + 2 * n) as u64).unwrap()),
                        true,
                    );
                }
                (sum, Real::from(count))
            }
            "scaled-twelfth" => {
                let scale = Real::from(Rational::from_bigint(
                    BigInt::from(1) << c["bits"].as_u64().unwrap() as usize,
                ));
                (actual(k, 0, op).unwrap() * &scale, radical(k, op) * scale)
            }
            _ => panic!("unknown case family"),
        }
    }
}

fn outcome(r: CertifiedRealEquality) -> usize {
    match r {
        CertifiedRealEquality::Equal { .. } => 0,
        CertifiedRealEquality::NotEqual { .. } => 1,
        CertifiedRealEquality::Unknown { .. } => 2,
    }
}
fn wall() -> String {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos()
        .to_string()
}

fn run(snapshot: Option<fn(bool) -> [usize; 4]>) {
    let args: Vec<String> = std::env::args().collect();
    assert_eq!(args.len(), 4);
    let mode = &args[1];
    assert!(matches!(mode.as_str(), "check" | "cpu" | "allocation"));
    if mode != "check" {
        assert_eq!(mode == "allocation", snapshot.is_some());
    }
    let cases: Vec<Value> =
        serde_json::from_str(&std::fs::read_to_string(&args[2]).unwrap()).unwrap();
    let groups: Vec<Value> =
        serde_json::from_str(&std::fs::read_to_string(&args[3]).unwrap()).unwrap();
    assert_eq!(cases.len(), 96);
    for group in &groups {
        let id = group["case"].as_u64().unwrap() as usize;
        let c = &cases[id];
        assert_eq!(c["id"], id);
        let p = group["precision"].as_i64().unwrap() as i32;
        assert!([-64, -256].contains(&p));
        let life = group["lifecycle"].as_str().unwrap();
        assert!(["fresh", "cold", "retained"].contains(&life));
        let n = group["iterations"].as_u64().unwrap() as usize;
        assert!((1..=2000).contains(&n));
        let retained = inputs::pair(c);
        let query = |a: &Real, b: &Real| a.certified_eq_until(black_box(b), black_box(p));
        let expected = query(&retained.0, &retained.1);
        let code = outcome(expected);
        assert_ne!(
            code,
            if c["truth"] == "equal" { 1 } else { 0 },
            "incorrect preflight {c}"
        );
        // Global constants/process caches are warmed, including for fresh/cold
        // routes. Only retained reuses the pair that receives these warmups.
        for _ in 0..4 {
            assert_eq!(query(&retained.0, &retained.1), expected);
        }
        let cold: Vec<_> = if life == "cold" {
            (0..n).map(|_| inputs::pair(c)).collect()
        } else {
            Vec::new()
        };
        let wall_before = wall();
        let before = snapshot.map_or([0; 4], |s| s(true));
        let start = Instant::now();
        let mut outcomes = [0usize; 3];
        for i in 0..n {
            let result = if life == "fresh" {
                let pair = black_box(inputs::pair(black_box(c)));
                query(black_box(&pair.0), &pair.1)
            } else {
                let pair = if life == "cold" {
                    cold.get(i).expect("cold pair exists")
                } else {
                    &retained
                };
                query(black_box(&pair.0), &pair.1)
            };
            outcomes[outcome(black_box(result))] += 1;
        }
        let elapsed_ns = start.elapsed().as_nanos();
        let after = snapshot.map_or([0; 4], |s| s(false));
        let wall_after = wall();
        let mut expected_counts = [0usize; 3];
        expected_counts[code] = n;
        assert_eq!(outcomes, expected_counts, "lifecycle changed answer {c}");
        assert_eq!(query(&retained.0, &retained.1), expected);
        let independent = inputs::pair(c);
        assert_eq!(query(&independent.0, &independent.1), expected);
        println!(
            "{}",
            json!({"mode":mode,"case":id,"precision":p,"lifecycle":life,"iterations":n,"outcome":code,
            "certificate":format!("{expected:?}"),"counts":outcomes,"elapsed_ns":elapsed_ns,"wall_before":wall_before,"wall_after":wall_after,
            "requests":after[0]-before[0],"requested_bytes":after[1]-before[1],"start_live":before[2],"end_live":after[2],
            "live_delta":after[2] as i128-before[2] as i128,"peak_delta":after[3].saturating_sub(before[2]),"final_matches":true})
        );
    }
    println!(
        "{}",
        json!({"terminal":true,"mode":mode,"groups":groups.len()})
    );
}
