use std::{cmp::Ordering, hint::black_box, time::Instant};
use hyperlimit::{compare_reals, PredicatePolicy};
use hyperreal::{Rational, Real};
use num::{BigInt, BigRational, One, Zero};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering as AO};

struct Counting;
static COUNT: AtomicBool = AtomicBool::new(false);
static ALLOCS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if COUNT.load(AO::Relaxed) { ALLOCS.fetch_add(1,AO::Relaxed); BYTES.fetch_add(layout.size(),AO::Relaxed); }
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) { unsafe { System.dealloc(p,l) } }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        if COUNT.load(AO::Relaxed) { ALLOCS.fetch_add(1,AO::Relaxed); BYTES.fetch_add(n,AO::Relaxed); }
        unsafe { System.realloc(p,l,n) }
    }
}
#[global_allocator] static ALLOCATOR: Counting = Counting;

type Matrix = Vec<Vec<Real>>;
type Inverse = Result<Matrix, usize>;

// Verbatim old Gauss-Jordan kernel from hypersolve 2bca88cc, apart from formatting.
fn solve_raw(matrix: &mut [Vec<Real>], rhs: &mut [Real], policy: PredicatePolicy) -> Result<Vec<Real>,usize> {
    let n = rhs.len();
    for pivot in 0..n {
        let pivot_row = (pivot..n).find(|&row| {
            !matches!(compare_reals(&matrix[row][pivot], &Real::zero(), policy).value(), Some(Ordering::Equal) | None)
        });
        let Some(pivot_row) = pivot_row else { return Err(pivot); };
        if pivot_row != pivot { matrix.swap(pivot_row,pivot); rhs.swap(pivot_row,pivot); }
        let pivot_value = matrix[pivot][pivot].clone();
        match matrix[pivot][pivot].clone() / pivot_value.clone() {
            Ok(normalized_pivot) => {
                matrix[pivot][pivot] = normalized_pivot;
                for value in matrix[pivot].iter_mut().skip(pivot+1) {
                    *value = (value.clone() / pivot_value.clone()).map_err(|_|pivot)?;
                }
                rhs[pivot] = (rhs[pivot].clone() / pivot_value).map_err(|_|pivot)?;
            }
            Err(_) => {
                let pivot_reciprocal = pivot_value.inverse_ref_assuming_nonzero().map_err(|_|pivot)?;
                for value in matrix[pivot].iter_mut().skip(pivot) { *value = value.clone() * &pivot_reciprocal; }
                rhs[pivot] = rhs[pivot].clone() * pivot_reciprocal;
            }
        }
        let pivot_tail = matrix[pivot][pivot..].to_vec();
        let pivot_rhs = rhs[pivot].clone();
        for row in 0..n {
            if row == pivot { continue; }
            let factor = matrix[row][pivot].clone();
            if compare_reals(&factor, &Real::zero(), policy).value() == Some(Ordering::Equal) { continue; }
            for (value,pivot_value) in matrix[row].iter_mut().skip(pivot).zip(&pivot_tail) {
                *value = value.clone() - factor.clone() * pivot_value.clone();
            }
            rhs[row] = rhs[row].clone() - factor * pivot_rhs.clone();
        }
    }
    Ok(rhs.to_vec())
}

#[inline(never)]
fn old(matrix: Matrix, policy: PredicatePolicy) -> Inverse {
    let n = matrix.len();
    let mut columns=Vec::with_capacity(n);
    for column in 0..n {
        let mut rhs=vec![Real::zero();n]; rhs[column]=Real::one();
        let mut copy=matrix.clone(); columns.push(solve_raw(&mut copy,&mut rhs,policy)?);
    }
    let mut inverse=vec![vec![Real::zero();n];n];
    for(column,solution) in columns.into_iter().enumerate() {
        for(row,value) in solution.into_iter().enumerate() {inverse[row][column]=value;}
    }
    Ok(inverse)
}

#[inline(never)]
fn joint(mut matrix: Matrix, policy: PredicatePolicy) -> Inverse {
    let n=matrix.len();
    for(row_index,row) in matrix.iter_mut().enumerate() {
        row.extend((0..n).map(|column| if row_index==column {Real::one()} else {Real::zero()}));
    }
    solve_raw(&mut matrix,&mut vec![Real::zero();n],policy)?;
    Ok(matrix.into_iter().map(|mut row| row.split_off(n)).collect())
}

#[inline(never)]
fn bareiss(matrix: Matrix, policy: PredicatePolicy) -> Inverse {
    let n=matrix.len();
    let rhs:Matrix=(0..n).map(|i|(0..n).map(|j|if i==j {Real::one()} else {Real::zero()}).collect()).collect();
    let report=hypersolve::solve_dense_linear_system_bareiss_multi_rhs(&matrix,&rhs,-128,policy).map_err(|_|usize::MAX)?;
    Ok((0..n).map(|i|(0..n).map(|j|report.solutions[j][i].clone()).collect()).collect())
}

fn rational(x:&BigRational)->Real {
    Real::new(Rational::from_bigint_fraction(x.numer().clone(), x.denom().to_biguint().unwrap()).unwrap())
}
fn oracle(x:&Real)->BigRational {
    let r=x.exact_rational_normal_form().expect("exact rational output");
    let mut numerator=BigInt::from(r.numerator().clone());
    if r.is_negative() {numerator=-numerator;}
    BigRational::new(numerator,BigInt::from(r.denominator().clone()))
}
fn data(n:usize,seed:usize,kind:&str)->Vec<Vec<BigRational>> {
    let mut m=vec![vec![BigRational::zero();n];n];
    for i in 0..n {for j in 0..n {
        m[i][j]=match kind {
            "cauchy"=>BigRational::new(BigInt::one(),BigInt::from(i+j+n+1+seed)),
            "diagonal"=>if i==j {BigRational::from_integer(BigInt::from(i+seed+1))} else {BigRational::zero()},
            "permutation"=>if j==(i+1)%n {BigRational::new(BigInt::from(i+seed+1),BigInt::from(seed+1))} else {BigRational::zero()},
            _=>BigRational::new(BigInt::from(((i*17+j*13+seed*7)%11) as i64-5),BigInt::from(1+(i*3+j*5+seed)%7)),
        };
    }}
    if kind=="dense" {for i in 0..n {m[i][i]+=BigRational::from_integer(BigInt::from(n*6+seed+1));}}
    m
}
fn matrix(n:usize,seed:usize,kind:&str)->Matrix {data(n,seed,kind).iter().map(|r|r.iter().map(rational).collect()).collect()}
fn validate(n:usize,seed:usize,kind:&str) {
    let q=data(n,seed,kind); let m:Matrix=q.iter().map(|r|r.iter().map(rational).collect()).collect();
    let mut previous=None;
    for f in [old,joint,bareiss] {
        let inverse=f(m.clone(),PredicatePolicy::STRICT).unwrap();
        let inv:Vec<Vec<_>>=inverse.iter().map(|row|row.iter().map(oracle).collect()).collect();
        for i in 0..n {for j in 0..n {
            let left:BigRational=(0..n).map(|k|&q[i][k]*&inv[k][j]).sum();
            let right:BigRational=(0..n).map(|k|&inv[i][k]*&q[k][j]).sum();
            let expected=BigRational::from_integer(BigInt::from(u8::from(i==j)));
            assert_eq!(left,expected,"A*inverse {n}/{seed}/{kind}/{i}/{j}");
            assert_eq!(right,expected,"inverse*A {n}/{seed}/{kind}/{i}/{j}");
        }}
        if let Some(ref p)=previous {assert_eq!(&inv,p);} previous=Some(inv);
    }
}
fn function(name:&str)->fn(Matrix,PredicatePolicy)->Inverse {match name {"old"=>old,"joint"=>joint,"bareiss"=>bareiss,_=>panic!("method")}}
fn batch(f:fn(Matrix,PredicatePolicy)->Inverse,inputs:&[Matrix],calls:usize)->f64 {
    let start=Instant::now();
    for k in 0..calls {black_box(f(black_box(inputs[k%inputs.len()].clone()),PredicatePolicy::STRICT).unwrap());}
    start.elapsed().as_secs_f64()/calls as f64
}
fn main() {
    let args:Vec<String>=std::env::args().collect();
    match args.get(1).map(String::as_str).unwrap_or("check") {
        "two-roots"=>{
            use hypersolve::{Problem,Expr,SymbolId,Constraint,VariableBall,context_from_problem,certify_multivariate_quadratic_krawczyk_box};
            let x=Expr::symbol(SymbolId(0),"x");
            let a=(Real::one()/Real::from(4)).unwrap();
            let mut problem=Problem::default();problem.add_variable("x",Real::zero());
            problem.add_constraint(Constraint::equality("x*(x+1/4)",x.clone().powi(2)+Expr::real(a.clone())*x));
            let roots=[Real::zero(),-a.clone()];
            for root in &roots {assert_eq!(oracle(&(root*root+&a*root)),BigRational::zero());assert!(oracle(root)>=-oracle(&a)&&oracle(root)<=oracle(&a));}
            let report=certify_multivariate_quadratic_krawczyk_box(&problem.analyze(),&context_from_problem(&problem),&[VariableBall{symbol:SymbolId(0),radius:a}],PredicatePolicy::STRICT);
            println!("two distinct exact roots, 0 and -1/4, in [-1/4,1/4]: {:?}",report);
        },
        "public"|"public-alloc"|"public-check"=>{
            use hypersolve::{Problem,Expr,SymbolId,Constraint,VariableBall,context_from_problem,certify_multivariate_quadratic_krawczyk_box,MultivariateQuadraticKrawczykStatus};
            let n:usize=args[2].parse().unwrap();let kind=&args[3];
            let radius=Real::from(2).powi_i64(-512).unwrap();
            let problems:Vec<_>=(0..13).map(|seed| {
                let mut problem=Problem::default();
                for i in 0..n {problem.add_variable(format!("x{i}"),Real::zero());}
                for (i,row) in matrix(n,seed,kind).into_iter().enumerate() {
                    let mut expr=Expr::symbol(SymbolId(i as u32),format!("x{i}")).powi(2);
                    for(j,a)in row.into_iter().enumerate(){expr=expr+Expr::real(a)*Expr::symbol(SymbolId(j as u32),format!("x{j}"));}
                    problem.add_constraint(Constraint::equality(format!("row{i}"),expr));
                }
                problem
            }).collect();
            let analyses:Vec<_>=problems.iter().map(|p|p.analyze()).collect();
            let contexts:Vec<_>=problems.iter().map(context_from_problem).collect();
            let radii:Vec<_>=(0..n).map(|i|VariableBall{symbol:SymbolId(i as u32),radius:radius.clone()}).collect();
            let evaluate=|i:usize|certify_multivariate_quadratic_krawczyk_box(&analyses[i],&contexts[i],&radii,PredicatePolicy::STRICT);
            for seed in 0..13 {
                let report=evaluate(seed);
                assert_eq!(report.status,MultivariateQuadraticKrawczykStatus::CertifiedUniqueRoot);
                assert_eq!(report.variables.len(),n);
                let inv=old(matrix(n,seed,kind),PredicatePolicy::STRICT).unwrap();
                let q=data(n,seed,kind);
                for i in 0..n {for j in 0..n {
                    let entry:BigRational=(0..n).map(|k|&q[i][k]*oracle(&inv[k][j])).sum();
                    assert_eq!(entry,BigRational::from_integer(BigInt::from(u8::from(i==j))));
                }}
                use num::Signed;
                for(i,v)in report.variables.iter().enumerate() {
                    let sum:BigRational=inv[i].iter().map(|x|oracle(x).abs()).sum();
                    let r=oracle(&radius);
                    assert_eq!(oracle(&v.step),BigRational::zero());
                    assert_eq!(oracle(&v.image_radius),&sum*&r*&r);
                    let legacy_scale=if args.get(4).map(String::as_str)==Some("legacy") {r.clone()} else {BigRational::one()};
                    assert_eq!(oracle(&v.contraction_bound),BigRational::from_integer(BigInt::from(2))*sum*&r*legacy_scale);
                }
            }
            if args[1]=="public-check" {println!("13 public certificates and all exact image/contraction fields pass n={n} {kind}");return;}
            let run=|calls:usize| {
                let start=Instant::now();
                for k in 0..calls {black_box(evaluate(k%13));}
                start.elapsed().as_secs_f64()/calls as f64
            };
            if args[1]=="public-alloc" {
                COUNT.store(true,AO::Relaxed);run(13);COUNT.store(false,AO::Relaxed);
                println!("{}",serde_json::json!({"n":n,"kind":kind,"calls":13,"allocations":ALLOCS.load(AO::Relaxed),"bytes":BYTES.load(AO::Relaxed)}));return;
            }
            run(13);let calls=(((0.07/run(13)) as usize/13).max(1)*13).min(13000);
            let rounds:usize=args.get(5).map(|s|s.parse().unwrap()).unwrap_or(7);
            for round in 0..rounds {println!("{round},{n},{kind},{calls},{}",run(calls)*1e9);}
        },
        "check"=>{
            let mut cases=0;
            for kind in ["dense","diagonal","permutation","cauchy"] {for n in [0,1,2,3,4,6,8] {for seed in 0..7 {validate(n,seed,kind);cases+=1;}}}
            for n in [1,2,3,4,8] {let m=vec![vec![Real::one();n];n];for f in [old,joint] {
                if n>1 {assert_eq!(f(m.clone(),PredicatePolicy::STRICT),Err(1));}
            }}
            println!("{} nonsingular matrices checked with all three methods and independent two-sided BigRational identities; singular controls pass",cases);
        },
        "bench"=>{
            let n:usize=args[2].parse().unwrap();let kind=&args[3];
            let inputs:Vec<_>=(0..13).map(|seed|matrix(n,seed,kind)).collect();
            let fs=[old,joint,bareiss];let mut calls=[13;3];
            for(method,f)in fs.iter().enumerate() {batch(*f,&inputs,13);let cost=batch(*f,&inputs,13);calls[method]=(((0.06/cost) as usize/13).max(1)*13).min(13000);}
            for round in 0..21 {
                for method in if round%2==0 {[0,1,2,2,1,0]} else {[2,1,0,0,1,2]} {
                    let ns=batch(fs[method],&inputs,calls[method])*1e9;
                    println!("{round},{},{n},{kind},{},{ns}",["old","joint","bareiss"][method],calls[method]);
                }
            }
        },
        "alloc"=>{
            let n:usize=args[2].parse().unwrap();let kind=&args[3];let method=&args[4];
            let inputs:Vec<_>=(0..13).map(|seed|matrix(n,seed,kind)).collect();
            COUNT.store(true,AO::Relaxed);batch(function(method),&inputs,13);COUNT.store(false,AO::Relaxed);
            println!("{}",serde_json::json!({"n":n,"kind":kind,"method":method,"calls":13,"allocations":ALLOCS.load(AO::Relaxed),"bytes":BYTES.load(AO::Relaxed)}));
        },
        _=>panic!("mode")
    }
}
