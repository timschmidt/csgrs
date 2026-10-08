use std::{cmp::Ordering, hint::black_box, time::Instant};
use hyperlimit::{compare_reals, PredicatePolicy};
use hyperreal::Real;
use num::{BigInt, BigRational, Zero};
type Matrix=Vec<Vec<Real>>;
type Inverse=Result<Matrix,usize>;

#[allow(dead_code)]
mod reference {
    include!("/tmp/hyper-inverse-audit.nizGgs/src/main.rs");
    pub fn original(m:Matrix,p:PredicatePolicy)->Inverse {old(m,p)}
    pub fn dummy(m:Matrix,p:PredicatePolicy)->Inverse {joint(m,p)}
    pub fn input(n:usize,s:usize,k:&str)->Matrix {matrix(n,s,k)}
    pub fn exact(x:&Real)->BigRational {oracle(x)}
    pub fn counts(enabled:bool) -> (usize,usize) {
        COUNT.store(enabled,AO::Relaxed);
        (ALLOCS.load(AO::Relaxed),BYTES.load(AO::Relaxed))
    }
}

// Same pivot/normalization/elimination sequence, with no dummy RHS operations.
fn eliminate(matrix:&mut [Vec<Real>],policy:PredicatePolicy)->Result<(),usize> {
    let n=matrix.len();
    for pivot in 0..n {
        let pivot_row=(pivot..n).find(|&row| !matches!(
            compare_reals(&matrix[row][pivot],&Real::zero(),policy).value(),
            Some(Ordering::Equal)|None));
        let Some(pivot_row)=pivot_row else{return Err(pivot)};
        if pivot_row!=pivot {matrix.swap(pivot_row,pivot);}
        let pivot_value=matrix[pivot][pivot].clone();
        match matrix[pivot][pivot].clone()/pivot_value.clone() {
            Ok(normalized_pivot)=>{
                matrix[pivot][pivot]=normalized_pivot;
                for value in matrix[pivot].iter_mut().skip(pivot+1) {
                    *value=(value.clone()/pivot_value.clone()).map_err(|_|pivot)?;
                }
            }
            Err(_)=>{
                let reciprocal=pivot_value.inverse_ref_assuming_nonzero().map_err(|_|pivot)?;
                for value in matrix[pivot].iter_mut().skip(pivot) {
                    *value=value.clone()*&reciprocal;
                }
            }
        }
        let pivot_tail=matrix[pivot][pivot..].to_vec();
        for (row_index,row) in matrix.iter_mut().enumerate() {
            if row_index==pivot {continue;}
            let factor=row[pivot].clone();
            if compare_reals(&factor,&Real::zero(),policy).value()==Some(Ordering::Equal){continue;}
            for(value,pivot_value)in row.iter_mut().skip(pivot).zip(&pivot_tail) {
                *value=value.clone()-factor.clone()*pivot_value.clone();
            }
        }
    }
    Ok(())
}

#[inline(never)]
fn joint(mut matrix:Matrix,policy:PredicatePolicy)->Inverse {
    let n=matrix.len();
    for(i,row)in matrix.iter_mut().enumerate() {
        row.reserve_exact(n);
        row.extend((0..n).map(|j|if i==j{Real::one()}else{Real::zero()}));
    }
    eliminate(&mut matrix,policy)?;
    Ok(matrix.into_iter().map(|mut row|row.split_off(n)).collect())
}

fn batch(f:fn(Matrix,PredicatePolicy)->Inverse,inputs:&[Matrix],calls:usize)->f64 {
    let start=Instant::now();
    for k in 0..calls {black_box(f(black_box(inputs[k%inputs.len()].clone()),PredicatePolicy::STRICT).unwrap());}
    start.elapsed().as_secs_f64()/calls as f64
}
fn check() {
    let mut cases=0;
    for kind in ["dense","diagonal","permutation","cauchy"] {
        for n in [0,1,2,3,4,6,8] {for seed in 0..7 {
            let m=reference::input(n,seed,kind);
            let expected=reference::original(m.clone(),PredicatePolicy::STRICT).unwrap();
            let actual=joint(m.clone(),PredicatePolicy::STRICT).unwrap();
            for i in 0..n {for j in 0..n {
                assert_eq!(reference::exact(&expected[i][j]),reference::exact(&actual[i][j]));
                let left:BigRational=(0..n).map(|k|reference::exact(&m[i][k])*reference::exact(&actual[k][j])).sum();
                let right:BigRational=(0..n).map(|k|reference::exact(&actual[i][k])*reference::exact(&m[k][j])).sum();
                let wanted=BigRational::from_integer(BigInt::from(u8::from(i==j)));
                assert_eq!(left,wanted);assert_eq!(right,wanted);
            }}
            cases+=1;
        }}
    }
    for n in [2,3,4,8] {
        let m=vec![vec![Real::one();n];n];
        assert_eq!(joint(m.clone(),PredicatePolicy::STRICT),Err(1));
        assert_eq!(reference::original(m,PredicatePolicy::STRICT),Err(1));
    }
    let zero=Real::zero();assert_eq!(reference::exact(&zero),BigRational::zero());
    println!("{cases} exact inverses match and satisfy two-sided BigRational identities; singular controls pass");
}
fn main() {
    let args:Vec<_>=std::env::args().collect();
    if args.len()==1||args[1]=="check" {check();return;}
    let n:usize=args[2].parse().unwrap();let kind=&args[3];
    let inputs:Vec<_>=(0..13).map(|s|reference::input(n,s,kind)).collect();
    if args[1]=="alloc" {
        let f=if args[4]=="old" {reference::original}else{joint};
        reference::counts(true);batch(f,&inputs,13);let (allocations,bytes)=reference::counts(false);
        println!("{}",serde_json::json!({"n":n,"kind":kind,"method":args[4],"calls":13,"allocations":allocations,"bytes":bytes}));return;
    }
    let fs:[fn(Matrix,PredicatePolicy)->Inverse;3]=[reference::original,reference::dummy,joint];
    let mut calls=[13;3];
    for(i,f)in fs.iter().enumerate() {batch(*f,&inputs,13);let cost=batch(*f,&inputs,13);calls[i]=(((0.05/cost) as usize/13).max(1)*13).min(13000);}
    for round in 0..21 {
        for method in if round%2==0 {[0,1,2,2,1,0]} else {[2,1,0,0,1,2]} {
            println!("{round},{},{n},{kind},{},{}",["old","dummy","joint"][method],calls[method],batch(fs[method],&inputs,calls[method])*1e9);
        }
    }
}
