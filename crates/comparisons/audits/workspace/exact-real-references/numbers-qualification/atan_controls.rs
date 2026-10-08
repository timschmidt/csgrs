use hyperreal::{Computable,Rational};
use rug::{Float,Integer,Rational as Q,float::Round};
use std::{alloc::{GlobalAlloc,Layout,System},sync::atomic::{AtomicU64,Ordering},time::Instant};
struct Counted;
static CALLS:AtomicU64=AtomicU64::new(0);
static BYTES:AtomicU64=AtomicU64::new(0);
unsafe impl GlobalAlloc for Counted {
    unsafe fn alloc(&self,l:Layout)->*mut u8 {CALLS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(l.size() as u64,Ordering::Relaxed);unsafe{System.alloc(l)}}
    unsafe fn alloc_zeroed(&self,l:Layout)->*mut u8 {CALLS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(l.size() as u64,Ordering::Relaxed);unsafe{System.alloc_zeroed(l)}}
    unsafe fn dealloc(&self,p:*mut u8,l:Layout){unsafe{System.dealloc(p,l)}}
    unsafe fn realloc(&self,p:*mut u8,l:Layout,n:usize)->*mut u8 {CALLS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(n as u64,Ordering::Relaxed);unsafe{System.realloc(p,l,n)}}
}
#[global_allocator] static ALLOCATOR:Counted=Counted;
#[repr(C)] struct Timespec {sec:i64,nsec:i64}
unsafe extern "C" {fn clock_gettime(id:i32,t:*mut Timespec)->i32;}
fn cpu()->u64 {let mut t=Timespec{sec:0,nsec:0};assert_eq!(unsafe{clock_gettime(2,&mut t)},0);t.sec as u64*1_000_000_000+t.nsec as u64}
fn c(q:&Q)->Computable {Computable::rational(q.to_string().parse::<Rational>().unwrap())}
fn int(s:&str)->Integer {Integer::from_str_radix(s,10).unwrap()}
fn root(q:Q)->(Q,Q) {let mut lo=Float::with_val(4096,&q);assert_eq!(lo.to_rational().unwrap(),q);let mut hi=lo.clone();lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);(lo.to_rational().unwrap(),hi.to_rational().unwrap())}
fn sine(n:i32)->(Q,Q) {let mut lo=Float::with_val(4096,n);let mut hi=lo.clone();lo.sin_round(Round::Down);hi.sin_round(Round::Up);(lo.to_rational().unwrap(),hi.to_rational().unwrap())}
fn make(family:&str,k:i32)->(Computable,(Q,Q)) {
    match family {
        "rational"=>{let q=Q::from((k%31-15,16));(c(&q),(q.clone(),q))},
        "known-small"|"known-negative"=>{
            let n=[2,3,5,7][k as usize%4];let scale=Q::from((if family=="known-negative"{-1}else{1},16));
            let (lo,hi)=root(Q::from(n));let bounds=if scale<0 {(hi*&scale,lo*&scale)}else{(lo*&scale,hi*&scale)};
            (c(&Q::from(n)).sqrt().multiply(c(&scale)),bounds)
        },
        "opaque-small"|"opaque-positive"|"opaque-negative"|"warm-negative"=>{
            let n=if family=="opaque-positive"{-100}else if family=="opaque-small" {k%9-4}else{100};
            let scale=if family=="opaque-small"{Q::from((1,16))}else{Q::from([2,4,8,64][k as usize%4])};
            let (lo,hi)=sine(n);let x=c(&Q::from(n)).sin().multiply(c(&scale));
            if family=="warm-negative" {let _=x.approx(-256);}
            (x,(lo*&scale,hi*&scale))
        },
        "hint-small"|"hint-large"=>{
            let count=if family=="hint-small"{4}else{32};
            let mut x=c(&Q::from(5)).sqrt().multiply(c(&Q::from((1,8))));
            let term=c(&Q::from(7)).sqrt().multiply(c(&Q::from((1,64))));
            for _ in 0..count {x=x.add(term.clone());}
            let (a,b)=root(Q::from(5));let (u,v)=root(Q::from(7));
            (x,(a/8+u*count/64,b/8+v*count/64))
        },
        _=>panic!("unknown family"),
    }
}
fn truth((lo,hi):(Q,Q))->(Q,Q) {
    let mut lo=Float::with_val_round(4096,lo,Round::Down).0;
    let mut hi=Float::with_val_round(4096,hi,Round::Up).0;
    lo.atan_round(Round::Down);hi.atan_round(Round::Up);
    (lo.to_rational().unwrap(),hi.to_rational().unwrap())
}
fn check(a:&str,p:i32,bounds:&(Q,Q)) {
    let scale=Integer::from(1)<<p;
    let q=Q::from((int(a),scale.clone()));let r=Q::from((1,scale));
    assert!(Q::from(&q-&r)<=bounds.0&&Q::from(&q+&r)>=bounds.1,"non-enclosing atan at {p} bits");
}
fn main() {
    let args:Vec<_>=std::env::args().collect();let mode=&args[1];let family=&args[2];
    if mode=="single" {
        let (x,bounds)=make(family,0);let bounds=truth(bounds);
        eprintln!("constructed {family}");let x=x.atan();
        for p in [32,0,128,8,512,32] {check(&x.approx(-p).to_string(),p,&bounds);}
        println!("PASS {family} precision-history=6");return;
    }
    assert_eq!(mode,"bench");
    let count=args.get(3).map_or(256,|x|x.parse::<i32>().unwrap());
    let inputs:Vec<_>=(0..count).map(|k|make(family,k)).collect();
    let references:Vec<_>=inputs.iter().map(|(_,b)|truth(b.clone())).collect();
    let mut outputs=Vec::with_capacity(inputs.len());
    let calls=CALLS.load(Ordering::Relaxed);let bytes=BYTES.load(Ordering::Relaxed);
    let started=Instant::now();let cpu0=cpu();
    for (k,(input,_)) in inputs.iter().enumerate() {
        let p=[32,128][k%2];
        outputs.push(std::hint::black_box(input.clone().atan().approx(-p)));
    }
    let elapsed_cpu=cpu()-cpu0;let elapsed_wall=started.elapsed().as_nanos();
    let calls=CALLS.load(Ordering::Relaxed)-calls;let bytes=BYTES.load(Ordering::Relaxed)-bytes;
    let mut checksum=0_u64;
    for (k,(a,bounds)) in outputs.iter().zip(references).enumerate() {
        let s=a.to_string();check(&s,[32,128][k%2],&bounds);
        for b in s.bytes(){checksum=checksum.wrapping_mul(131).wrapping_add(u64::from(b));}
    }
    println!("{family}\t{count}\t{elapsed_cpu}\t{elapsed_wall}\t{calls}\t{bytes}\t{checksum}");
}
