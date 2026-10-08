use hypercurve::{CurveContext, Point2, RationalBezier2, Real};
use std::{hint::black_box, time::{Duration, Instant}};

fn curve(kind: &str) -> RationalBezier2 {
    match kind {
        "line" => RationalBezier2::try_new(
            vec![Point2::new(0.into(), 0.into()), Point2::new(1.into(), 2.into())],
            vec![2.into(), 3.into()],
        ).unwrap(),
        "cubic" => RationalBezier2::try_new(
            vec![Point2::new(0.into(), 1.into()), Point2::new(2.into(), 4.into()),
                 Point2::new(5.into(), (-1).into()), Point2::new(7.into(), 3.into())],
            vec![2.into(), 3.into(), 5.into(), 7.into()],
        ).unwrap(),
        "dense8" | "dense32" => {
            let degree=if kind=="dense8" {8} else {32};
            let mut weight=Real::one();
            let mut points=Vec::new(); let mut weights=Vec::new();
            for _ in 0..=degree {
                points.push(Point2::new((Real::one()/&weight).unwrap(),Real::zero()));
                weights.push(weight.clone()); weight*=Real::from(2);
            }
            RationalBezier2::try_new(points,weights).unwrap()
        }
        _ => panic!("unknown workload"),
    }
}

fn main() {
    let args:Vec<_>=std::env::args().skip(1).collect();
    let kind=&args[0];let order=args[1].parse::<usize>().unwrap();let phase=&args[2];
    let once=args.get(3).is_some_and(|x|x=="once");
    let parameter=(Real::one()/Real::from(3)).unwrap();
    let retained=curve(kind);
    let warm=retained.derivatives_at(&parameter,order,&CurveContext::STRICT).unwrap();
    assert_eq!(warm.len(),order);
    drop(warm);
    let run=|| {
        let cold;
        let input=if phase=="cold" {cold=curve(black_box(kind));&cold} else {&retained};
        let ds=input.derivatives_at(black_box(&parameter),black_box(order),&CurveContext::STRICT).unwrap();
        assert_eq!(ds.len(),order);
        black_box(ds);
    };
    if once {run();return;}
    let start=Instant::now();let mut iterations=0_u64;
    loop {
        run(); iterations+=1;
        if start.elapsed()>=Duration::from_millis(200) {break;}
    }
    println!("{kind}\t{order}\t{phase}\t{iterations}\t{:.3}",start.elapsed().as_nanos() as f64/iterations as f64);
}
