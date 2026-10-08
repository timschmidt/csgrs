use hyperreal::{Computable, Rational};
use num::{BigInt, One};
use rug::{Float, Integer, float::Round};
use std::collections::BTreeSet;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
#[allow(dead_code)]
mod kernel { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/kernel.rs")); }
mod invariant { include!("e-plan-invariant.rs"); }

fn enclose(actual: &BigInt, p: i32, kind: &str) {
    let prec = p.unsigned_abs().max(64) + 192;
    let mut lo = Float::with_val(prec, 1);
    let mut hi = lo.clone();
    lo.exp_round(Round::Down);
    hi.exp_round(Round::Up);
    if p < 0 { lo <<= p.unsigned_abs(); hi <<= p.unsigned_abs(); }
    else { lo >>= p as u32; hi >>= p as u32; }
    let a = Integer::from_str_radix(&actual.to_str_radix(16), 16).unwrap();
    let bottom = Float::with_val(prec, &a - 1);
    let top = Float::with_val(prec, &a + 1);
    assert!(bottom < lo && top > hi, "{kind}: p={p} outside strict one-unit enclosure");
    println!("{{\"kind\":\"{kind}\",\"p\":{p},\"integer\":\"{}\"}}", actual.to_str_radix(16));
}

fn plans() {
    let mut requests: BTreeSet<i32> = (-4096..=16).collect();
    let selected: BTreeSet<u32> = [20,21,22,31,32,33,63,64,65,127,128,129,255,256,257,
        511,512,513,1023,1024,1025,2047,2048,2049,4095,4096,4097,8191,8192].into_iter().collect();
    let mut factorial = Integer::from(1);
    for n in 1..=8192_u32 {
        factorial *= n;
        if selected.contains(&n) {
            let bits = factorial.significant_bits() as i32;
            for delta in [-1,0,1] {
                let needed = bits + delta;
                if needed > 4 { requests.insert(4-needed); }
            }
        }
    }
    for bits in [16384,32768,65536,120700,262144] { requests.insert(-bits); }
    let mut factorial = Integer::from(1);
    let mut k = 1_u32;
    let mut extra = 0_u32;
    for &p in requests.iter().rev() {
        let needed = if p < 0 { p.unsigned_abs()+4 } else {4};
        while factorial.significant_bits() <= needed { k += 1; factorial *= k; }
        let expected = k-1;
        let actual = kernel::e_terms_for_precision(p);
        assert!(actual >= expected, "early stop at {p}");
        assert!(actual <= expected+1, "sampled term overhead at {p}");
        extra += u32::from(actual != expected);
        println!("{{\"kind\":\"plan\",\"p\":{p},\"actual\":{actual},\"expected\":{expected}}}");
    }
    let n = invariant::check_lower_invariant(-262144);
    assert!(n >= kernel::e_terms_for_precision(-262144));
    println!("{{\"kind\":\"plan-summary\",\"requests\":{},\"extraTermRequests\":{extra},\"invariantSteps\":{}}}",requests.len(),n+1);
}

fn numeric() {
    let mut requests: BTreeSet<i32> = (-64..=8).collect();
    for p in [-127,-128,-129,-255,-256,-257,-511,-512,-513,-1023,-1024,-1025,
        -4095,-4096,-4097,-16384,-32768,-65536,-120700,-262144] { requests.insert(p); }
    let public = Computable::e();
    for &p in requests.iter().rev() {
        enclose(&kernel::e(p),p,"kernel");
        enclose(&public.approx(p),p,"public-refine");
    }
    for &p in &requests { enclose(&Computable::e().approx(p),p,"public-coarsen"); }
    println!("{{\"kind\":\"numeric-summary\",\"requests\":{},\"enclosures\":{}}}",requests.len(),3*requests.len());
}

fn state() {
    let stop = Arc::new(AtomicBool::new(true));
    let mut aborted = Computable::e();
    aborted.abort(stop.clone());
    drop(aborted.approx(-4096));
    stop.store(false,Ordering::Relaxed);
    enclose(&aborted.approx(-4096),-4096,"cancel-recovery");
    let e = Computable::e();
    let json = serde_json::to_string(&e).unwrap();
    let restored: Computable = serde_json::from_str(&json).unwrap();
    for p in [-8192,-4096,-128,0] { enclose(&restored.approx(p),p,"serde"); }
    let exp_one = Computable::rational(Rational::one()).exp();
    for p in [-16384,-512,0] { enclose(&exp_one.approx(p),p,"exp-one"); }
    let handles: Vec<_> = [4096_i32,8192,16384,32768].into_iter().map(|bits| {
        let e = e.clone();
        std::thread::spawn(move || [-bits,-bits/2,-64].map(|p|(p,e.approx(p))))
    }).collect();
    for handle in handles { for (p,a) in handle.join().unwrap() { enclose(&a,p,"thread"); } }
    enclose(&Computable::e().approx(-65536),-65536,"post-thread-refine");
    println!("{{\"kind\":\"state-summary\",\"enclosures\":21,\"threads\":4}}");
}

fn main() {
    let mode = std::env::args().nth(1).expect("plans, numeric or state");
    match mode.as_str() { "plans"=>plans(), "numeric"=>numeric(), "state"=>state(), _=>panic!("unknown mode") }
}
