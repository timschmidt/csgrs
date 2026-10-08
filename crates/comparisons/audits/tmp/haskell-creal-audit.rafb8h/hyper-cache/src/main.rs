use hyperreal::{Computable, Rational};
use rug::{Float, Integer, float::Round};
use std::{alloc::{GlobalAlloc, Layout, System}, hint::black_box, sync::atomic::{AtomicBool, AtomicU64, Ordering}, time::Instant};

struct Counted;
static TRACK: AtomicBool = AtomicBool::new(false);
static ALLOCS: AtomicU64 = AtomicU64::new(0);
static BYTES: AtomicU64 = AtomicU64::new(0);
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if TRACK.load(Ordering::Relaxed) { ALLOCS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed); }
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if TRACK.load(Ordering::Relaxed) { ALLOCS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed); }
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) { unsafe { System.dealloc(ptr, layout) } }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if TRACK.load(Ordering::Relaxed) { ALLOCS.fetch_add(1, Ordering::Relaxed); BYTES.fetch_add(new_size as u64, Ordering::Relaxed); }
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}
#[global_allocator]
static ALLOCATOR: Counted = Counted;

#[repr(C)]
struct Timespec { seconds: std::ffi::c_long, nanos: std::ffi::c_long }
unsafe extern "C" { fn clock_gettime(id: i32, t: *mut Timespec) -> i32; }
fn cpu_ns() -> u64 {
    let mut t = Timespec { seconds: 0, nanos: 0 };
    assert_eq!(unsafe { clock_gettime(2, &mut t) }, 0);
    t.seconds as u64 * 1_000_000_000 + t.nanos as u64
}
fn input() -> Computable { Computable::rational(Rational::fraction(black_box(17),32).unwrap()).sin() }
fn apply(op: &str, x: Computable) -> Computable {
    match op {
        "negate" => x.negate(),
        "scale-up" => x.multiply(Computable::rational(Rational::from(8))),
        "scale-down" => x.multiply(Computable::rational(Rational::fraction(1,8).unwrap())),
        "square" => x.square(),
        "integer-add" => x.add(Computable::rational(Rational::from(7))),
        "identity" => x,
        _ => panic!("unknown operation"),
    }
}
fn oracle(op: &str, value: &Computable, bits: i32) {
    let mut lo = Float::with_val(2048, 17); lo >>= 5;
    let mut hi = lo.clone();
    lo.sin_round(Round::Down); hi.sin_round(Round::Up);
    match op {
        "negate" => (lo,hi)=(-hi,-lo),
        "scale-up" => {lo <<= 3; hi <<= 3;},
        "scale-down" => {lo >>= 3; hi >>= 3;},
        "square" => {lo.square_round(Round::Down); hi.square_round(Round::Up);},
        "integer-add" => {lo=Float::with_val_round(2048,&lo+7,Round::Down).0;hi=Float::with_val_round(2048,&hi+7,Round::Up).0;},
        "identity" => {},
        _ => unreachable!(),
    }
    lo <<= bits; hi <<= bits;
    let got = Float::with_val(2048,Integer::from_str_radix(&value.approx(-bits).to_string(),10).unwrap());
    assert!(Float::with_val(2048,&got-&lo).abs()<=1 && Float::with_val(2048,&got-&hi).abs()<=1,"{op} at {bits}");
}
fn main() {
    let a: Vec<_> = std::env::args().skip(1).collect();
    let op=&a[0]; let phase=&a[1]; let bits:i32=a[2].parse().unwrap(); let mode=&a[3];
    assert!((1..=1024).contains(&bits));
    let base=input();
    if phase != "unprimed" { black_box(base.approx(-bits-16)); }
    if mode == "oracle" {
        for initial in [0,32,bits+16] {
            for requested in [2,16,128,512,1024] {
                let x=input(); black_box(x.approx(-initial));
                oracle(op,&apply(op,x),requested);
            }
            let x=input(); black_box(x.approx(-initial)); let value=apply(op,x);
            for requested in [2,16,128,512,1024,16,2] { oracle(op,&value,requested); }
        }
        println!("PASS {op} 36 oracle queries"); return;
    }
    let run=|| {
        let arg=match phase.as_str() {
            "cold" => input(),
            "finer" => {let x=input();black_box(x.approx(-32));x},
            "construct"|"warm"|"unprimed" => base.clone(),
            _=>panic!("unknown phase"),
        };
        let value=black_box(apply(black_box(op),black_box(arg)));
        if !matches!(phase.as_str(),"construct"|"unprimed") { black_box(value.approx(black_box(-bits))); }
        black_box(value);
    };
    run();
    if mode=="alloc" {
        ALLOCS.store(0,Ordering::Relaxed);BYTES.store(0,Ordering::Relaxed);TRACK.store(true,Ordering::Relaxed);
        for _ in 0..64 { run(); }
        TRACK.store(false,Ordering::Relaxed);
        println!("{op}\t{phase}\t{bits}\talloc\t{}\t{}",ALLOCS.load(Ordering::Relaxed) as f64/64.,BYTES.load(Ordering::Relaxed) as f64/64.);return;
    }
    assert_eq!(mode,"bench");
    let start=cpu_ns();let wall=Instant::now();let mut count=0u64;
    let end=loop { for _ in 0..64 {run();}count+=64;let now=cpu_ns();if now-start>=200_000_000 {break now;} };
    println!("{op}\t{phase}\t{bits}\t{count}\t{}\t{}",(end-start) as f64/count as f64,wall.elapsed().as_nanos() as f64/count as f64);
}
