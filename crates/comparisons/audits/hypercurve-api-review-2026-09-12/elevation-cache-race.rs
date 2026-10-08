use hypercurve::{Point2, RationalBezier2, Real};
use std::sync::Barrier;
fn main() {
    for attempt in 0..16 {
        let source = RationalBezier2::try_new(
            vec![Point2::from_values(0,0), Point2::from_values(1,1), Point2::from_values(2,0)],
            vec![Real::one(), Real::one(), Real::one()],
        ).unwrap();
        let barrier = Barrier::new(8);
        let degrees = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..8).map(|_| scope.spawn(|| {
                barrier.wait();
                source.elevated_to_degree(10).unwrap().degree()
            })).collect();
            handles.into_iter().map(|handle| handle.join().unwrap()).collect::<Vec<_>>()
        });
        println!("attempt={attempt} requested=10 actual={degrees:?}");
        if degrees.iter().any(|degree| *degree != 10) { std::process::exit(1); }
    }
}
