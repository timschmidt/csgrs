#[path = "../../../allocations.rs"]
mod allocations;
use hyperlattice_interval::IntervalVector;
use hyperlattice_interval::hyperinterval::{F32Enclosure, F64Enclosure, FixedInterval};
use interval_layout_conformance::Soa;
use interval_layout_conformance::workload::{Fixture, inputs, native_chain, soa_chain};
use std::{fmt::Debug, hint::black_box, mem::size_of, time::Instant};

fn run<I: Fixture + Eq + Debug, const N: usize>(name: &str) {
    let (a, b) = inputs::<I, N>(7);
    let x = Soa::from_vector(&a);
    let y = Soa::from_vector(&b);
    println!(
        "size,{name},{N},{},{}",
        size_of::<IntervalVector<I, N>>(),
        size_of::<Soa<I, N>>()
    );
    for operation in 0..8 {
        // Check full materialized outputs before timing; checksums alone are
        // insufficient. Operation 7 is the separately reported retained path.
        if operation < 5 {
            let native = match operation {
                0 => a,
                1 => a.add(&b, &mut ()).unwrap(),
                2 => a.sub(&b, &mut ()).unwrap(),
                3 => a.hull(&b, &mut ()).unwrap(),
                _ => a.intersection(&b, &mut ()).unwrap(),
            };
            let soa = match operation {
                0 => x,
                1 => x.add(&y).unwrap(),
                2 => x.sub(&y).unwrap(),
                3 => x.hull(&y).unwrap(),
                _ => x.intersection(&y).unwrap(),
            }
            .to_vector()
            .unwrap();
            assert_eq!(native, soa);
        } else if operation == 5 {
            assert_eq!(a.dot(&b, &mut ()), x.dot(&y));
        } else {
            assert_eq!(native_chain(&a, &b), soa_chain(&x, &y));
        }
        let repetitions = (256 / (N + 1)).max(4);
        let execute = |soa: bool| {
            if soa {
                if operation == 7 {
                    black_box(soa_chain(black_box(&x), black_box(&y)).unwrap());
                    return;
                }
                let (x, y) = (
                    Soa::from_vector(black_box(&a)),
                    Soa::from_vector(black_box(&b)),
                );
                match operation {
                    0 => {
                        black_box(x.to_vector().unwrap());
                    }
                    1 => {
                        black_box(x.add(&y).unwrap().to_vector().unwrap());
                    }
                    2 => {
                        black_box(x.sub(&y).unwrap().to_vector().unwrap());
                    }
                    3 => {
                        black_box(x.hull(&y).unwrap().to_vector().unwrap());
                    }
                    4 => {
                        black_box(x.intersection(&y).unwrap().to_vector().unwrap());
                    }
                    5 => {
                        black_box(x.dot(&y).unwrap());
                    }
                    _ => {
                        black_box(soa_chain(&x, &y).unwrap());
                    }
                }
            } else {
                let (a, b) = (black_box(&a), black_box(&b));
                match operation {
                    0 => {
                        black_box(*a);
                    }
                    1 => {
                        black_box(a.add(b, &mut ()).unwrap());
                    }
                    2 => {
                        black_box(a.sub(b, &mut ()).unwrap());
                    }
                    3 => {
                        black_box(a.hull(b, &mut ()).unwrap());
                    }
                    4 => {
                        black_box(a.intersection(b, &mut ()).unwrap());
                    }
                    5 => {
                        black_box(a.dot(b, &mut ()).unwrap());
                    }
                    _ => {
                        black_box(native_chain(a, b).unwrap());
                    }
                }
            }
        };
        for _ in 0..16 {
            execute(false);
            execute(true);
        }
        for sample in 0..12 {
            for soa in if sample % 2 == 0 {
                [false, true]
            } else {
                [true, false]
            } {
                let (elapsed, count, bytes) = allocations::measure(|| {
                    let start = Instant::now();
                    for _ in 0..repetitions {
                        execute(soa);
                    }
                    start.elapsed().as_nanos()
                });
                assert_eq!((count, bytes), (0, 0));
                println!(
                    "sample,{name},{N},{operation},{sample},{soa},{repetitions},{elapsed},{count},{bytes}"
                );
            }
        }
    }
}

fn main() {
    macro_rules! family {
        ($i:ty, $name:literal) => {
            run::<$i, 0>($name);
            run::<$i, 1>($name);
            run::<$i, 2>($name);
            run::<$i, 3>($name);
            run::<$i, 4>($name);
            run::<$i, 16>($name);
            run::<$i, 64>($name);
            run::<$i, 256>($name);
            run::<$i, 1024>($name);
        };
    }
    family!(FixedInterval<i32, 8>, "i32q8");
    family!(FixedInterval<i64, 32>, "i64q32");
    family!(F32Enclosure, "f32");
    family!(F64Enclosure, "f64");
}
