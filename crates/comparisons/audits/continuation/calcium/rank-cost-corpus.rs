use hyperreal::{CertifiedRealSign, Real};
use hypersolve::{Constraint, ExactAffineRankStatus, Expr, Problem, ProblemAnalysis, SymbolId, analyze_exact_affine_rank};
use std::{hint::black_box, time::Instant};

fn build_problem(case: usize, width: usize, u: &Real) -> Problem {
    let mut a = vec![vec![Real::zero(); width]; if case < 3 { 1 } else { 2 }];
    match case {
        0 => a[0][0] = Real::one(),
        1 | 2 => {
            a[0].fill(u.clone());
            if case == 1 { a[0][width-1] = Real::one(); }
        }
        3 => {
            a[0][..width-2].fill(u.clone());
            a[0][width-2] = Real::one(); a[1][width-1] = Real::one();
        }
        4 | 5 => {
            for j in 0..width {
                a[0][j] = if case == 4 { u * Real::from((j+1) as i64) }
                    else { Real::from(2*(j+1) as i64) };
                a[1][j] = Real::one();
            }
        }
        6 => {}
        _ => panic!("invalid case"),
    }
    let mut p = Problem::default();
    for j in 0..width { p.add_variable(format!("x{j}"), Real::zero()); }
    for (i,row) in a.iter().enumerate() {
        let e = row.iter().enumerate().fold(Expr::int(0),|sum,(j,c)|
            sum + Expr::real(c.clone()) * Expr::symbol(SymbolId(j as u32),format!("x{j}")));
        p.add_constraint(Constraint::equality(format!("row{i}"),e));
    }
    p
}

fn query(analysis: &ProblemAnalysis<'_>, case: usize, width: usize) -> bool {
    let r = black_box(analyze_exact_affine_rank(black_box(analysis),-128));
    match r.status {
        ExactAffineRankStatus::Certified => {
            let rank = if case == 6 {0} else if case < 3 {1} else {2};
            assert_eq!(r.coefficient_rank,Some(rank)); assert_eq!(r.augmented_rank,Some(rank));
            assert_eq!(r.degrees_of_freedom,Some(width-rank)); assert!(r.error.is_none()); true
        }
        ExactAffineRankStatus::Undecided => {
            assert!(r.coefficient_rank.is_none() && r.augmented_rank.is_none() && r.error.is_some()); false
        }
        _ => panic!("incorrect rank result {r:?}"),
    }
}

fn run(counters: Option<fn(bool) -> [usize;4]>) {
    let a: Vec<_> = std::env::args().collect(); assert_eq!(a.len(),6);
    let variant = &a[1]; assert!(["baseline","candidate"].contains(&variant.as_str()));
    let case: usize = a[2].parse().unwrap(); let width: usize = a[3].parse().unwrap();
    let lifecycle = &a[4]; let iterations: usize = a[5].parse().unwrap();
    assert!(case < 7 && [4,8,16,32].contains(&width) && iterations > 0);
    assert!(["fresh_problem","retained_analysis"].contains(&lifecycle.as_str()));
    let [_,upper] = Real::pi().certified_dyadic_interval(-768).unwrap();
    let u = Real::from(upper) - Real::pi();
    assert!(matches!(u.certified_sign_until(-128),CertifiedRealSign::Unknown{..}));
    let expected_known = [0,5,6].contains(&case) || (variant == "candidate" && [1,3].contains(&case));
    let p = build_problem(case,width,&u); let analysis = p.analyze();
    for _ in 0..8 { assert_eq!(query(&analysis,case,width),expected_known); }
    let before = counters.map_or([0;4],|f|f(true)); let start = Instant::now(); let mut known = 0;
    for _ in 0..iterations {
        known += usize::from(if lifecycle == "fresh_problem" {
            let fresh = build_problem(case,width,&u); query(&fresh.analyze(),case,width)
        } else { query(&analysis,case,width) });
    }
    let elapsed_ns = start.elapsed().as_nanos(); let after = counters.map_or([0;4],|f|f(false));
    assert_eq!(known,if expected_known {iterations}else{0});
    let requests=after[0]-before[0]; let requested_bytes=after[1]-before[1];
    let live_delta=after[2] as i128-before[2] as i128;
    let peak_delta=after[3].saturating_sub(before[2]);
    let mode=if counters.is_some() {"allocation"} else {"cpu"};
    println!("{{\"mode\":\"{mode}\",\"variant\":\"{variant}\",\"case\":{case},\"width\":{width},\"lifecycle\":\"{lifecycle}\",\"iterations\":{iterations},\"known\":{known},\"elapsed_ns\":{elapsed_ns},\"requests\":{requests},\"requested_bytes\":{requested_bytes},\"live_delta\":{live_delta},\"peak_delta\":{peak_delta}}}");
}
