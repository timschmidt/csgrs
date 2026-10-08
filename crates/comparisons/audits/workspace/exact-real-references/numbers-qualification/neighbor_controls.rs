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
fn root(n: i32) -> (Q, Q) { let mut lo = Float::with_val(4096, n); let mut hi = lo.clone(); lo.sqrt_round(Round::Down); hi.sqrt_round(Round::Up); (lo.to_rational().unwrap(), hi.to_rational().unwrap()) }
fn make(family: &str, k: i32) -> (Computable, (Q, Q)) {
    match family {
        "rational" => { let q = Q::from(([1, -1, 64, -192][k as usize % 4], 256)); (c(&q), (q.clone(), q)) },
        "known-small" | "known-negative" => {
            let n = [2, 3, 5, 7][k as usize % 4]; let sign = if family == "known-negative" { -1 } else { 1 };
            let (lo, hi) = root(n); let bounds = if sign < 0 { (-hi / 64, -lo / 64) } else { (lo / 64, hi / 64) };
            (c(&Q::from(n)).sqrt().multiply(c(&Q::from((sign, 64)))), bounds)
        },
        "opaque-small" | "warm-small" => {
            let n = [4, 6, 10, 100][k as usize % 4]; let mut lo = Float::with_val(4096, n); let mut hi = lo.clone();
            lo.sin_round(Round::Down); hi.sin_round(Round::Up);
            let x = c(&Q::from(n)).sin().multiply(c(&Q::from((1, 64)))); if family == "warm-small" { let _ = x.approx(-256); }
            (x, (lo.to_rational().unwrap() / 64, hi.to_rational().unwrap() / 64))
        },
        "hint-small" | "hint-medium" | "repaired" => {
            let count = match family { "hint-small" => 8, "hint-medium" => 64, _ => 740 };
            let mut x = c(&Q::from(5)).sqrt().multiply(c(&Q::from((1, 64)))); let term = c(&Q::from(7)).sqrt().multiply(c(&Q::from((1, 2048))));
            for _ in 0..count { x = x.add(term.clone()); }
            let (lo, hi) = root(5); let (a, b) = root(7);
            (x, (lo / 64 + a * count / 2048, hi / 64 + b * count / 2048))
        },
        _ => panic!("unknown family"),
    }
}
fn truth(op: &str, (lo, hi): (Q, Q)) -> (Q, Q) {
    assert!(lo > -1 && hi < 1 && lo <= hi);
    let mut lo = Float::with_val_round(4096, lo, Round::Down).0; let mut hi = Float::with_val_round(4096, hi, Round::Up).0;
    match op { "asin" => { lo.asin_round(Round::Down); hi.asin_round(Round::Up); }, "atanh" => { lo.atanh_round(Round::Down); hi.atanh_round(Round::Up); }, _ => panic!("op") }
    (lo.to_rational().unwrap(), hi.to_rational().unwrap())
}
fn main() {
    let args: Vec<_> = std::env::args().collect(); let op = &args[1]; let family = &args[2];
    let count = args.get(3).map_or(256, |s| s.parse::<i32>().unwrap());
    let inputs: Vec<_> = (0..count).map(|k| make(family, k)).collect();
    let refs: Vec<_> = inputs.iter().map(|(_, b)| truth(op, b.clone())).collect();
    let mut outputs = Vec::with_capacity(inputs.len());
    let calls = CALLS.load(Ordering::Relaxed); let bytes = BYTES.load(Ordering::Relaxed); let start = Instant::now(); let t = cpu();
    for (k, (input, _)) in inputs.iter().enumerate() {
        let x = match op.as_str() { "asin" => input.clone().asin(), "atanh" => input.clone().atanh(), _ => panic!("op") };
        outputs.push(std::hint::black_box(x.approx(-[32, 128][k % 2])));
    }
    let elapsed = cpu() - t; let wall = start.elapsed().as_nanos();
    let calls = CALLS.load(Ordering::Relaxed) - calls; let bytes = BYTES.load(Ordering::Relaxed) - bytes;
    let mut checksum = 0_u64;
    for (k, (a, (lo, hi))) in outputs.iter().zip(refs).enumerate() {
        let s = a.to_string(); let scale = Integer::from(1) << [32_i32, 128][k % 2];
        let center = Q::from((Integer::from_str_radix(&s, 10).unwrap(), scale.clone())); let radius = Q::from((1, scale));
        assert!(Q::from(&center - &radius) <= lo && Q::from(&center + &radius) >= hi, "non-enclosing {op} {family} {k}");
        for b in s.bytes() { checksum = checksum.wrapping_mul(131).wrapping_add(u64::from(b)); }
    }
    println!("{op}\t{family}\t{count}\t{elapsed}\t{wall}\t{calls}\t{bytes}\t{checksum}");
}
