use std::{alloc::{GlobalAlloc,Layout,System},sync::atomic::{AtomicU64,Ordering}};
struct Counted;
static CALLS:AtomicU64=AtomicU64::new(0);
static BYTES:AtomicU64=AtomicU64::new(0);
static LIVE:AtomicU64=AtomicU64::new(0);
static PEAK:AtomicU64=AtomicU64::new(0);
fn allocated(size:usize){
    CALLS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(size as u64,Ordering::Relaxed);
    let live=LIVE.fetch_add(size as u64,Ordering::Relaxed)+size as u64;PEAK.fetch_max(live,Ordering::Relaxed);
}
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self,l:Layout)->*mut u8 {let p=unsafe{System.alloc(l)};if !p.is_null(){allocated(l.size());}p}
    unsafe fn alloc_zeroed(&self,l:Layout)->*mut u8 {let p=unsafe{System.alloc_zeroed(l)};if !p.is_null(){allocated(l.size());}p}
    unsafe fn realloc(&self,p:*mut u8,l:Layout,n:usize)->*mut u8 {let q=unsafe{System.realloc(p,l,n)};if !q.is_null(){LIVE.fetch_sub(l.size() as u64,Ordering::Relaxed);allocated(n);}q}
    unsafe fn dealloc(&self,p:*mut u8,l:Layout){LIVE.fetch_sub(l.size() as u64,Ordering::Relaxed);unsafe{System.dealloc(p,l)}}
}
#[global_allocator]static ALLOCATOR:Counted=Counted;
mod bench {include!("hyper_field_bench.rs");pub fn run(){main()}}
fn main(){
    bench::run();
    let(calls,bytes,peak)=(CALLS.load(Ordering::Relaxed),BYTES.load(Ordering::Relaxed),PEAK.load(Ordering::Relaxed));
    println!("MEMORY\t{calls}\t{bytes}\t{peak}");
}
