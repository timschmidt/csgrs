use hyperreal::{Computable, Rational};
use rug::{Float, Integer, Rational as Q, float::Round};
use std::{alloc::{GlobalAlloc, Layout, System}, sync::atomic::{AtomicU64, Ordering}, time::Instant};
struct Counted;
static CALLS: AtomicU64 = AtomicU64::new(0);
static BYTES: AtomicU64 = AtomicU64::new(0);
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 { CALLS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(l.size() as u64, Ordering::Relaxed); unsafe { System.alloc(l) } }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 { CALLS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(l.size() as u64, Ordering::Relaxed); unsafe { System.alloc_zeroed(l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 { CALLS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(n as u64, Ordering::Relaxed); unsafe { System.realloc(p, l, n) } }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { unsafe { System.dealloc(p, l) } }
}
#[global_allocator] static ALLOCATOR: Counted = Counted;
#[repr(C)] struct Timespec { sec: i64, nsec: i64 }
unsafe extern "C" { fn clock_gettime(id: i32, t: *mut Timespec) -> i32; }
fn cpu() -> u64 { let mut t = Timespec { sec: 0, nsec: 0 }; assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0); t.sec as u64 * 1_000_000_000 + t.nsec as u64 }
fn c(q: &Q) -> Computable { Computable::rational(q.to_string().parse::<Rational>().unwrap()) }
fn root(q: &Q) -> (Q, Q) { let mut lo = Float::with_val(4096, q); assert_eq!(lo.to_rational().unwrap(), *q); let mut hi = lo.clone(); lo.sqrt_round(Round::Down); hi.sqrt_round(Round::Up); (lo.to_rational().unwrap(), hi.to_rational().unwrap()) }
fn make(family: &str, k: usize) -> (Computable, (Q, Q)) {
    match family {
        "ln-rational" => { let q = [Q::from((5,4)),Q::from((7,4)),Q::from((17,8)),Q::from((1,16))][k%4].clone(); (c(&q),(q.clone(),q)) },
        "ln-radical" | "asinh-small" | "asinh-negative" | "asinh-warm" | "acosh-moderate" | "acosh-large" => {
            let n = match family { "acosh-moderate" => [5,7,11,17][k%4], "acosh-large" => [101,103,107,109][k%4], _ => [2,3,5,7][k%4] };
            let q = Q::from(n); let (lo,hi) = root(&q); let x = c(&q).sqrt();
            if family.starts_with("asinh") {
                let negative = family == "asinh-negative"; let factor = Q::from((if negative {-1} else {1},64));
                let x = x.multiply(c(&factor)); if family == "asinh-warm" {let _ = x.approx(-256);}
                let b = if negative {(hi*&factor,lo*&factor)} else {(lo*&factor,hi*&factor)};
                (x,b)
            } else {(x,(lo,hi))}
        },
        "ln-near-one" | "acosh-near-one" => {
            let bits = if family == "ln-near-one" {80_i32} else {10};
            let q = Q::from(1) + Q::from(([1,3,5,7][k%4],Integer::from(1)<<bits));
            (c(&q).sqrt(),root(&q))
        },
        "asinh-moderate" | "asinh-large" => {
            let n = if family == "asinh-moderate" {[2,3,4,6][k%4]} else {[16,24,32,64][k%4]};
            let q = Q::from(n); (c(&q),(q.clone(),q))
        },
        _ => panic!("family"),
    }
}
fn truth(op: &str, (lo,hi): (Q,Q)) -> (Q,Q) {
    let mut lo = Float::with_val_round(4096,lo,Round::Down).0; let mut hi = Float::with_val_round(4096,hi,Round::Up).0;
    match op {
        "ln" => {assert!(lo>0);lo.ln_round(Round::Down);hi.ln_round(Round::Up);},
        "asinh" => {lo.asinh_round(Round::Down);hi.asinh_round(Round::Up);},
        "acosh" => {assert!(lo>=1);lo.acosh_round(Round::Down);hi.acosh_round(Round::Up);},
        _ => panic!("op"),
    }
    (lo.to_rational().unwrap(),hi.to_rational().unwrap())
}
fn main() {
    let args: Vec<_> = std::env::args().collect(); let family = &args[1]; let op = family.split('-').next().unwrap();
    let count = args.get(2).map_or(256, |s| s.parse::<usize>().unwrap());
    let inputs: Vec<_> = (0..count).map(|k|make(family,k)).collect(); let refs: Vec<_> = inputs.iter().map(|(_,b)|truth(op,b.clone())).collect();
    let mut outputs = Vec::with_capacity(count);
    let calls = CALLS.load(Ordering::Relaxed); let bytes = BYTES.load(Ordering::Relaxed); let start = Instant::now(); let t = cpu();
    for (k,(x,_)) in inputs.iter().enumerate() {
        let x = match op {"ln"=>x.clone().ln(),"asinh"=>x.clone().asinh(),"acosh"=>x.clone().acosh(),_=>unreachable!()};
        outputs.push(std::hint::black_box(x.approx(-[32,128][k%2])));
    }
    let elapsed = cpu()-t; let wall = start.elapsed().as_nanos(); let calls = CALLS.load(Ordering::Relaxed)-calls; let bytes = BYTES.load(Ordering::Relaxed)-bytes;
    let mut checksum = 0_u64;
    for (k,(a,(lo,hi))) in outputs.iter().zip(refs).enumerate() {
        let s = a.to_string(); let scale = Integer::from(1)<<[32_i32,128][k%2];
        let center = Q::from((Integer::from_str_radix(&s,10).unwrap(),scale.clone())); let radius = Q::from((1,scale));
        assert!(Q::from(&center-&radius)<=lo && Q::from(&center+&radius)>=hi,"non-enclosing {family} {k}");
        for b in s.bytes(){checksum=checksum.wrapping_mul(131).wrapping_add(u64::from(b));}
    }
    println!("{family}\t{count}\t{elapsed}\t{wall}\t{calls}\t{bytes}\t{checksum}");
}
