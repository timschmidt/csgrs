use hyperreal::{CertifiedRealEquality, Rational, Real};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::{Arc, Barrier, atomic::{AtomicUsize, Ordering::SeqCst}};

// Counts requested live Rust allocation sizes, not allocator overhead or RSS.
struct Counted;
static LIVE_BYTES: AtomicUsize = AtomicUsize::new(0);
static LIVE_BLOCKS: AtomicUsize = AtomicUsize::new(0);
static PEAK_BYTES: AtomicUsize = AtomicUsize::new(0);
fn added(bytes: usize) {
    let now = LIVE_BYTES.fetch_add(bytes, SeqCst) + bytes;
    LIVE_BLOCKS.fetch_add(1, SeqCst);
    PEAK_BYTES.fetch_max(now, SeqCst);
}
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc(layout) };
        if !p.is_null() { added(layout.size()); }
        p
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        let p = unsafe { System.alloc_zeroed(layout) };
        if !p.is_null() { added(layout.size()); }
        p
    }
    unsafe fn dealloc(&self, p: *mut u8, layout: Layout) {
        unsafe { System.dealloc(p, layout) };
        LIVE_BYTES.fetch_sub(layout.size(), SeqCst);
        LIVE_BLOCKS.fetch_sub(1, SeqCst);
    }
    unsafe fn realloc(&self, p: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        let next = unsafe { System.realloc(p, layout, size) };
        if !next.is_null() {
            let now = if size >= layout.size() {
                LIVE_BYTES.fetch_add(size-layout.size(), SeqCst) + size-layout.size()
            } else { LIVE_BYTES.fetch_sub(layout.size()-size, SeqCst) - (layout.size()-size) };
            PEAK_BYTES.fetch_max(now, SeqCst);
        }
        next
    }
}
#[global_allocator]
static ALLOCATOR: Counted = Counted;

fn pair(case: usize, depth: usize) -> (Real, Real) {
    let argument = || {
        let mut x = Real::from(Rational::fraction((case % 31 + 1) as i64, 64).unwrap());
        for _ in 0..depth { x = x.sin(); }
        x
    };
    (argument().exp().unwrap().sqrt().unwrap(),
     (argument() * Real::from(Rational::fraction(1,2).unwrap())).exp().unwrap())
}
fn work(case: usize, depth: usize) -> bool {
    let (a,b)=pair(case,depth);
    match a.certified_eq_until(&b,-64) {
        CertifiedRealEquality::Equal { .. } => true,
        CertifiedRealEquality::Unknown { .. } => false,
        CertifiedRealEquality::NotEqual { .. } => panic!("false inequality"),
    }
}
fn snapshot() -> (usize,usize) { (LIVE_BYTES.load(SeqCst),LIVE_BLOCKS.load(SeqCst)) }
fn main() {
    let args:Vec<_>=std::env::args().collect();
    let threads:usize=args[1].parse().unwrap();
    let rounds:usize=args[2].parse().unwrap();
    let depth:usize=args[3].parse().unwrap();
    assert!([1,8,32].contains(&threads)&&[1,64,512].contains(&rounds)&&[1,8].contains(&depth));
    // Preinitialize shared constants/facts in this process. Worker threads are
    // still new: they have not touched their own proof cache.
    for pass in 0..2 { for case in 0..31 { std::hint::black_box(work(case+pass*31,depth)); } }
    let process_before=snapshot();
    let ready=Arc::new(Barrier::new(threads+1));
    let start=Arc::new(Barrier::new(threads+1));
    let done=Arc::new(Barrier::new(threads+1));
    let exit=Arc::new(Barrier::new(threads+1));
    let known=Arc::new(AtomicUsize::new(0));
    let mut handles=Vec::with_capacity(threads);
    for _ in 0..threads {
        let (ready,start,done,exit,known)=(ready.clone(),start.clone(),done.clone(),exit.clone(),known.clone());
        handles.push(std::thread::spawn(move || {
            ready.wait(); start.wait();
            let mut equal=0;
            for case in 0..rounds { equal+=usize::from(work(case,depth)); }
            known.fetch_add(equal,SeqCst);
            // Every expression in work() has dropped. Keep the worker/TLS alive
            // until the parent has measured the retained allocations.
            done.wait(); exit.wait();
        }));
    }
    ready.wait();
    let worker_before=snapshot();
    PEAK_BYTES.store(worker_before.0,SeqCst);
    start.wait(); done.wait();
    let worker_after=snapshot();
    let peak=PEAK_BYTES.load(SeqCst);
    exit.wait();
    for handle in handles { handle.join().unwrap(); }
    let equal=known.load(SeqCst);
    drop((ready,start,done,exit,known));
    let process_after=snapshot();
    println!("{{\"threads\":{threads},\"rounds\":{rounds},\"depth\":{depth},\"equal\":{equal},\"queries\":{},\"worker_before\":{:?},\"worker_after\":{:?},\"process_before\":{:?},\"process_after\":{:?},\"peak_bytes\":{peak}}}",
        threads*rounds, [worker_before.0,worker_before.1], [worker_after.0,worker_after.1],
        [process_before.0,process_before.1], [process_after.0,process_after.1]);
}
