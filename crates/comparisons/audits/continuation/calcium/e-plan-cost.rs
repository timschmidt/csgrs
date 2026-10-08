use hyperreal::Computable;
use num::BigInt;
use std::hint::black_box;
use std::time::Instant;
#[allow(dead_code)]
mod kernel { include!(concat!(env!("CARGO_MANIFEST_DIR"), "/kernel.rs")); }
fn fingerprint(a: &BigInt) -> String {
    let mut h=0xcbf29ce484222325_u64;
    for b in a.to_signed_bytes_le() { h=(h^u64::from(b)).wrapping_mul(0x100000001b3); }
    format!("{h:016x}")
}
fn run(snapshot: Option<fn(bool)->[usize;4]>) {
    let args:Vec<String>=std::env::args().collect();
    assert_eq!(args.len(),5,"precision route lifecycle iterations");
    let p:i32=args[1].parse().unwrap();
    let route=args[2].as_str();let lifecycle=args[3].as_str();let count:usize=args[4].parse().unwrap();
    assert!((-262144..=16).contains(&p));assert!((1..=1048576).contains(&count));
    assert!(["planner","kernel","public","pi-control"].contains(&route));
    let public=if route=="pi-control" {Computable::pi()} else {Computable::e()};
    if ["planner","kernel"].contains(&route) { assert_eq!(lifecycle,"fresh"); }
    else {
        match lifecycle {
            "fresh"=>assert_eq!(count,1),
            "warm"=>{drop(public.approx(p));},
            "coarsen"=>{drop(public.approx(p-256));},
            "refine"=>{assert_eq!(count,1);drop(public.approx(p/4));},
            _=>panic!("lifecycle")
        }
    }
    // No public preflight: in a fresh process, constructing the wrapper above
    // does not populate its shared numeric cache. Warm/coarsen/refine setup and
    // wrapper construction are excluded; output destruction is inside timing.
    let before=snapshot.map(|s|s(true));let start=Instant::now();
    for _ in 0..count {
        match route {
            "planner"=>{black_box(kernel::e_terms_for_precision(black_box(p)));},
            "kernel"=>drop(black_box(kernel::e(black_box(p)))),
            _=>drop(black_box(public.approx(black_box(p))))
        }
    }
    let ns=start.elapsed().as_nanos();let after=snapshot.map(|s|s(false));
    let allocation=match (before,after) {
        (Some(a),Some(b))=>format!("[{}, {}, {}, {}]",b[0]-a[0],b[1]-a[1],b[2] as i128-a[2] as i128,b[3].saturating_sub(a[2])),
        _=>"null".into()
    };
    let answer=match route {
        "planner"=>BigInt::from(kernel::e_terms_for_precision(p)),
        "kernel"=>kernel::e(p),
        _=>public.approx(p)
    };
    println!("{{\"p\":{p},\"route\":\"{route}\",\"lifecycle\":\"{lifecycle}\",\"iterations\":{count},\"ns\":{ns},\"allocation\":{allocation},\"fingerprint\":\"{}\",\"answerBits\":{}}}",fingerprint(&answer),answer.bits());
}
