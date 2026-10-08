use std::{hint::black_box,time::Instant};
use hyperreal::{Rational,Real};
use num::{BigInt,BigRational,One,Zero,Signed};
use std::alloc::{GlobalAlloc,Layout,System};
use std::sync::atomic::{AtomicBool,AtomicUsize,Ordering};
struct Counting;
static COUNT:AtomicBool=AtomicBool::new(false);
static ALLOCS:AtomicUsize=AtomicUsize::new(0);
static BYTES:AtomicUsize=AtomicUsize::new(0);
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self,l:Layout)->*mut u8 {
        if COUNT.load(Ordering::Relaxed){ALLOCS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(l.size(),Ordering::Relaxed);}
        unsafe{System.alloc(l)}
    }
    unsafe fn dealloc(&self,p:*mut u8,l:Layout){unsafe{System.dealloc(p,l)}}
    unsafe fn realloc(&self,p:*mut u8,l:Layout,n:usize)->*mut u8 {
        if COUNT.load(Ordering::Relaxed){ALLOCS.fetch_add(1,Ordering::Relaxed);BYTES.fetch_add(n,Ordering::Relaxed);}
        unsafe{System.realloc(p,l,n)}
    }
}
#[global_allocator] static ALLOCATOR:Counting=Counting;

fn real(q:&BigRational)->Real {
    Real::new(Rational::from_bigint_fraction(q.numer().clone(),q.denom().to_biguint().unwrap()).unwrap())
}
fn exact(x:&Real)->BigRational {
    let q=x.exact_rational_normal_form().unwrap();let n=BigInt::from(q.numerator().clone());
    BigRational::new(if q.is_negative(){-n}else{n},BigInt::from(q.denominator().clone()))
}
fn integerize(xs:&[Real])->Vec<BigInt> {
    Rational::primitive_bigint_ratio(&xs.iter().map(Real::exact_rational_ref).collect::<Option<Vec<_>>>().unwrap())
}

// Current Hypersolve midpoint recurrence, with linear live storage.
fn split_real(coefficients:&[Real])->Option<(Vec<Real>,Vec<Real>)> {
    let degree=coefficients.len().checked_sub(1)?;
    let mut work=coefficients.to_vec();
    let mut left=Vec::with_capacity(coefficients.len());
    let mut right=vec![Real::zero();coefficients.len()];
    left.push(work.first()?.clone());right[degree]=work.get(degree)?.clone();
    for level in 1..=degree {
        for index in 0..=degree-level {
            work[index]=((work[index].clone()+work[index+1].clone())/Real::from(2)).ok()?;
        }
        left.push(work[0].clone());right[degree-level]=work[degree-level].clone();
    }
    Some((left,right))
}
fn split_integer(coefficients:&[BigInt])->Option<(Vec<BigInt>,Vec<BigInt>)> {
    let degree=coefficients.len().checked_sub(1)?;
    let mut work=coefficients.to_vec();
    let mut left=Vec::with_capacity(coefficients.len());
    let mut right=vec![BigInt::zero();coefficients.len()];
    left.push(work.first()?<<degree);right[degree]=work.get(degree)?<<degree;
    for level in 1..=degree {
        for index in 0..=degree-level {
            let (prefix,suffix)=work.split_at_mut(index+1);
            prefix[index]+=&suffix[0];
        }
        left.push(&work[0]<<(degree-level));
        right[degree-level]=&work[degree-level]<<(degree-level);
    }
    Some((left,right))
}
fn binomial(n:usize,k:usize)->BigInt {
    let mut value=BigInt::one();
    for i in 0..k {value*=n-i;value/=i+1;}
    value
}
// Independent closed-form boundary coefficients, not a de Casteljau recurrence.
fn direct(xs:&[BigRational])->(Vec<BigRational>,Vec<BigRational>) {
    let n=xs.len()-1;
    let left=(0..=n).map(|j|{
        let sum:BigRational=(0..=j).map(|k|&xs[k]*binomial(j,k)).sum();
        sum/(BigInt::one()<<j)
    }).collect();
    let right=(0..=n).map(|j|{
        let sum:BigRational=(0..=n-j).map(|k|&xs[j+k]*binomial(n-j,k)).sum();
        sum/(BigInt::one()<<(n-j))
    }).collect();
    (left,right)
}
fn source(n:usize,seed:usize,kind:&str)->Vec<BigRational> {
    (0..=n).map(|i|{
        let value=((i*17+seed*13)%31) as i64-15;
        let (a,b)=match kind {
            "rational"=>(BigInt::from(value),BigInt::from(1+(i*7+seed)%23)),
            "dyadic"=>(BigInt::from(value),BigInt::one()<<(1+(i*7+seed)%33)),
            "wide"=>((BigInt::from(value)<<128)+BigInt::from(seed+i+1),(BigInt::one()<<(1+(i*7+seed)%67))+BigInt::one()),
            "sparse"=>(BigInt::from(if(i+seed)%5==0{value}else{0}),BigInt::one()),
            "alternating"=>(BigInt::from(if(i+seed)%2==0{1}else{-1}),BigInt::one()),
            _=>(BigInt::from(value),BigInt::one()),
        };
        BigRational::new(a,b)
    }).collect()
}
fn check() {
    assert!(split_real(&[]).is_none());assert!(split_integer(&[]).is_none());
    let mut cases=0;let mut splits=0;
    for n in (0..=12).chain([16,32,64]) {for seed in 0..8 {for kind in ["integer","rational","dyadic","wide","sparse","alternating"] {
        let mut qs=source(n,seed,kind);let mut rs:Vec<_>=qs.iter().map(real).collect();let mut ints=integerize(&rs);
        let mut scale=qs.iter().zip(&ints).find(|(q,_)|!q.is_zero()).map(|(q,i)|BigRational::from_integer(i.clone())/q).unwrap_or_else(BigRational::one);
        assert!(scale.is_positive());
        for(q,i)in qs.iter().zip(&ints){assert_eq!(q*&scale,BigRational::from_integer(i.clone()));}
        for depth in 0..4 {
            let(qleft,qright)=direct(&qs);let(rleft,rright)=split_real(&rs).unwrap();let(ileft,iright)=split_integer(&ints).unwrap();
            scale*=BigInt::one()<<n;
            for((q,r),i)in qleft.iter().chain(&qright).zip(rleft.iter().chain(&rright)).zip(ileft.iter().chain(&iright)) {
                assert_eq!(*q,exact(r));assert_eq!(q*&scale,BigRational::from_integer(i.clone()),"n={n} seed={seed} kind={kind} depth={depth}");
            }
            assert_eq!(ileft.last(),iright.first());
            let right=(seed+depth)%2==0;
            qs=if right{qright}else{qleft};rs=if right{rright}else{rleft};ints=if right{iright}else{ileft};splits+=1;
        }
        cases+=1;
    }}}
    println!("{cases} input vectors, {splits} chained splits: Real and common-scale integer coefficients match independent closed-form rational oracles, including exact midpoint identity");
}
struct Input {real:Vec<Real>,integer:Vec<BigInt>}
#[inline(never)]
fn run(method:usize,input:&Input,levels:usize,seed:usize) {
    if method==0 {
        let mut pair=split_real(&input.real).unwrap();
        for level in 1..levels {pair=split_real(if(level+seed)%2==0{&pair.0}else{&pair.1}).unwrap();}
        black_box(pair);
    } else {
        let cold;if method==2 {cold=integerize(&input.real);}else{cold=Vec::new();}
        let mut pair=split_integer(if method==2{&cold}else{&input.integer}).unwrap();
        for level in 1..levels {pair=split_integer(if(level+seed)%2==0{&pair.0}else{&pair.1}).unwrap();}
        black_box(pair);
    }
}
fn batch(method:usize,inputs:&[Input],levels:usize,calls:usize)->f64 {
    let start=Instant::now();
    for k in 0..calls {run(method,black_box(&inputs[k%13]),levels,k%13);}
    start.elapsed().as_secs_f64()/calls as f64
}
fn main() {
    let args:Vec<_>=std::env::args().collect();
    if args.len()==1||args[1]=="check"{check();return;}
    let n:usize=args[2].parse().unwrap();let kind=&args[3];let levels:usize=args[4].parse().unwrap();
    let inputs:Vec<_>=(0..13).map(|seed|{let real:Vec<_>=source(n,seed,kind).iter().map(real).collect();let integer=integerize(&real);Input{real,integer}}).collect();
    if args[1]=="alloc" {
        let method:usize=args[5].parse().unwrap();COUNT.store(true,Ordering::Relaxed);batch(method,&inputs,levels,13);COUNT.store(false,Ordering::Relaxed);
        println!("{}",serde_json::json!({"degree":n,"kind":kind,"levels":levels,"method":method,"calls":13,"allocations":ALLOCS.load(Ordering::Relaxed),"bytes":BYTES.load(Ordering::Relaxed)}));return;
    }
    let mut calls=[13;3];for(method,count)in calls.iter_mut().enumerate(){batch(method,&inputs,levels,13);let cost=batch(method,&inputs,levels,13);*count=(((0.05/cost)as usize/13).max(1)*13).min(13000);}
    for round in 0..21 {for method in if round%2==0{[0,1,2,2,1,0]}else{[2,1,0,0,1,2]} {
        println!("{round},{},{n},{kind},{levels},{},{}",["real","warm","cold"][method],calls[method],batch(method,&inputs,levels,calls[method])*1e9);
    }}
}
