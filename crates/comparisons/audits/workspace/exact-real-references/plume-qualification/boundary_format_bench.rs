use hyperreal::{Computable,Rational};
use rug::{Integer,Rational as Q};
use std::alloc::{GlobalAlloc,Layout,System};
use std::sync::atomic::{AtomicU64,Ordering};
use std::time::Instant;

struct Counted;
static CALLS:AtomicU64=AtomicU64::new(0);
static BYTES:AtomicU64=AtomicU64::new(0);
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self,l:Layout)->*mut u8 {CALLS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(l.size() as u64,Ordering::Relaxed);unsafe {System.alloc(l)}}
    unsafe fn alloc_zeroed(&self,l:Layout)->*mut u8 {CALLS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(l.size() as u64,Ordering::Relaxed);unsafe {System.alloc_zeroed(l)}}
    unsafe fn dealloc(&self,p:*mut u8,l:Layout) {unsafe {System.dealloc(p,l)}}
    unsafe fn realloc(&self,p:*mut u8,l:Layout,n:usize)->*mut u8 {CALLS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(n as u64,Ordering::Relaxed);unsafe {System.realloc(p,l,n)}}
}
#[global_allocator] static ALLOCATOR:Counted=Counted;
#[repr(C)] struct Timespec {sec:i64,nsec:i64}
unsafe extern "C" {fn clock_gettime(id:i32,t:*mut Timespec)->i32;}
fn cpu()->u64 {let mut t=Timespec{sec:0,nsec:0};assert_eq!(unsafe {clock_gettime(2,&mut t)},0);t.sec as u64*1_000_000_000+t.nsec as u64}
fn c(q:&Q)->Computable {Computable::rational(q.to_string().parse::<Rational>().unwrap())}
fn pow2(e:u32)->Q {Q::from((1,Integer::from(1)<<e))}
fn decimal(text:&str)->Q {
    let (neg,s)=text.strip_prefix('-').map_or((false,text),|s|(true,s));
    let (a,b)=s.split_once('.').unwrap_or((s,""));
    assert!(!a.is_empty() && a.bytes().chain(b.bytes()).all(|c|c.is_ascii_digit()));
    let mut n=Integer::from_str_radix(&format!("{a}{b}"),10).unwrap();if neg {n= -n;}
    let mut d=Integer::from(1);for _ in b.bytes(){d*=10;}Q::from((n,d))
}
fn main() {
    let family=std::env::args().nth(1).expect("family");
    let mut inputs=Vec::new();let mut references=Vec::new();
    // Fresh independently constructed nodes. No shared pi/global caches or
    // repeated warm formatting. All construction/oracle allocation excluded.
    for k in 0..512 {
        let (q,scale,radical)=match family.as_str() {
            "rational"=>(Q::from((k%65-32,[3,10,32][k as usize%3])),Q::from(1),false),
            "tiny-rational"=>(Q::from((k%65-32,3)),pow2(512+(k as u32%16)*256),false),
            "tiny-radical"=>(Q::from([2,3,5,7][k as usize%4]),pow2(512+(k as u32%16)*256),true),
            "ordinary-radical"=>(Q::from([2,3,5,7][k as usize%4]),Q::from(k%16+1),true),
            _=>panic!("unknown family")
        };
        let places=[0_usize,2,8,32][k as usize%4];
        let x=if radical {c(&q).sqrt().multiply(c(&scale))}else{c(&(q.clone()*&scale))};
        inputs.push((x,places));
        references.push((if radical {q*Q::from(&scale*&scale)}else{q*scale},radical));
    }
    let mut outputs=Vec::with_capacity(inputs.len());
    let calls=CALLS.load(Ordering::Relaxed);let bytes=BYTES.load(Ordering::Relaxed);
    let started=Instant::now();let cpu0=cpu();
    for (x,places) in &inputs {outputs.push(std::hint::black_box(format!("{x:.places$}")));}
    let elapsed_cpu=cpu()-cpu0;let elapsed_wall=started.elapsed().as_nanos();
    let calls=CALLS.load(Ordering::Relaxed)-calls;let bytes=BYTES.load(Ordering::Relaxed)-bytes;
    let mut checksum=0_u64;
    for ((text,(_,places)),(truth,radical)) in outputs.iter().zip(&inputs).zip(&references) {
        let got=decimal(text);let mut d=Integer::from(1);for _ in 0..*places {d*=10;}
        let radius=Q::from((1,d));
        if *radical {
            let lo=Q::from(&got-&radius);let hi=got+radius;
            assert!(hi>=0 && Q::from(&hi*&hi)>=*truth);
            assert!(lo<=0 || Q::from(&lo*&lo)<=*truth);
        }else{assert!((got-truth).abs()<=radius);}
        for b in text.bytes(){checksum=checksum.wrapping_mul(131).wrapping_add(u64::from(b));}
    }
    println!("{family}\t{}\t{elapsed_cpu}\t{elapsed_wall}\t{calls}\t{bytes}\t{checksum}",inputs.len());
}
