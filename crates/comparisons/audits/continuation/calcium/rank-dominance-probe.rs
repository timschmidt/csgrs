use hyperreal::{CertifiedRealSign, Real, RealSign};
use hypersolve::{Constraint, ExactAffineRankStatus, Expr, Problem, SymbolId, analyze_exact_affine_rank};

fn scalar(kind: usize) -> Real {
    match kind {
        0 => Real::one(),
        1 => Real::from(2).sqrt().unwrap(),
        2 => Real::from(2).ln().unwrap(),
        3 => {
            let [_, upper] = Real::pi().certified_dyadic_interval(-768).unwrap();
            Real::from(upper) - Real::pi()
        }
        4 => {
            let s = Real::one().sin(); let c = Real::one().cos();
            &s * &s + &c * &c - Real::one()
        }
        5 => Real::zero(),
        _ => unreachable!(),
    }
}

fn problem(matrix: &[Vec<Real>]) -> Problem {
    let mut p = Problem::default();
    for j in 0..matrix[0].len() { p.add_variable(format!("x{j}"), Real::zero()); }
    for (i, row) in matrix.iter().enumerate() {
        let e = row.iter().enumerate().fold(Expr::int(0), |sum,(j,c)|
            sum + Expr::real(c.clone()) * Expr::symbol(SymbolId(j as u32),format!("x{j}")));
        p.add_constraint(Constraint::equality(format!("row{i}"),e));
    }
    p
}

fn check(family: &str, kind: usize, width: usize, position: usize, floor: i32,
         matrix: &[Vec<Real>], expected_rank: usize) {
    let p = problem(matrix);
    let report = analyze_exact_affine_rank(&p.analyze(),floor);
    let status = match report.status {
        ExactAffineRankStatus::Certified => {
            assert_eq!(report.coefficient_rank,Some(expected_rank));
            assert_eq!(report.augmented_rank,Some(expected_rank));
            assert_eq!(report.degrees_of_freedom,Some(matrix[0].len()-expected_rank));
            assert!(report.error.is_none()); "Certified"
        }
        ExactAffineRankStatus::Undecided => {
            assert!(report.coefficient_rank.is_none() && report.augmented_rank.is_none());
            assert!(report.error.is_some()); "Undecided"
        }
        _ => panic!("incorrect rank status {report:?}"),
    };
    println!("{family},{kind},{width},{position},{floor},{status},{expected_rank}");
}

fn main() {
    println!("family,kind,width,position,floor,status,expected_rank");
    let mut queries=0;
    for floor in [-32,-128,-512] { for kind in 0..6 {
        let u=scalar(kind);
        let sign=u.certified_sign_until(floor);
        if kind==3 || kind==4 { assert!(matches!(sign,CertifiedRealSign::Unknown{..}),"kind{kind}: {sign:?}"); }
        else { assert!(matches!(sign,CertifiedRealSign::Known{sign:RealSign::Positive|RealSign::Zero,..})); }
        for width in [2,3,4,6,8] { for position in 0..width {
            let mut row=vec![u.clone();width]; row[position]=Real::one();
            check("row",kind,width,position,floor,&[row],1); queries+=1;
        } }
        for width in [2,3,4,6] { for position in 0..width {
            let mut rows=vec![vec![u.clone()];width]; rows[position][0]=Real::one();
            check("column",kind,width,position,floor,&rows,1); queries+=1;
        } }
        for width in [3,4,5,6,7] {
            let mut a=vec![vec![Real::zero();width];2];
            for x in &mut a[0][..width-2] { *x=u.clone(); }
            a[0][width-2]=Real::one(); a[1][width-1]=Real::one();
            check("rank2",kind,width,0,floor,&a,2); queries+=1;
        }
        let nonzero=kind<=3;
        for (position,a,rank) in [
            (0,vec![vec![u.clone(),Real::zero()],vec![Real::zero(),Real::one()]],if nonzero {2}else{1}),
            (1,vec![vec![u.clone(),Real::zero()],vec![Real::zero(),Real::zero()]],usize::from(nonzero)),
            (2,vec![vec![u.clone(),Real::zero(),Real::zero()],vec![Real::zero(),Real::one(),Real::zero()],vec![Real::zero();3]],if nonzero {2}else{1}),
        ] {
            check("blocked",kind,a.len(),position,floor,&a,rank); queries+=1;
        }
    } }
    println!("{{\"suite\":\"rank-dominance\",\"queries\":{queries}}}");
}
