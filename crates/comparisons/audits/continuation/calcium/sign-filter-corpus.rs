use hyperlimit::{Point2, PredicatePolicy, PredicateOutcome, Sign, ring_area_sign};
use hyperreal::{Rational, Real};
use std::hint::black_box;
use std::time::Instant;

fn points(fixture: &str, n: usize) -> Vec<Point2> {
    let left = if fixture == "rational" { Real::from(3) } else { Real::pi() };
    let right = match fixture {
        "rational" => Real::from(2),
        "pi-far" => Real::from(3),
        "pi-near" | "pi-reversed" => Real::new(Rational::fraction(103_993, 33_102).unwrap()),
        _ => panic!("unknown fixture"),
    };
    let mut points = (0..n-1).map(|i| {
        let t = Real::new(Rational::fraction(i as i64, (n-2) as u64).unwrap());
        Point2::new(&left * &t, t)
    }).collect::<Vec<_>>();
    points.push(Point2::new(right, Real::one()));
    if fixture == "pi-reversed" { points.reverse(); }
    points
}

fn query(points: &[Point2], lifecycle: &str) -> PredicateOutcome<Sign> {
    if lifecycle == "retained" {
        ring_area_sign(black_box(points), PredicatePolicy::STRICT)
    } else {
        assert_eq!(lifecycle, "cloned-ring");
        let cloned = points.to_vec();
        ring_area_sign(black_box(&cloned), PredicatePolicy::STRICT)
    }
}

fn run(memory: Option<fn(bool) -> [usize; 4]>) {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    assert_eq!(args.len(), 5);
    let (variant, fixture, lifecycle) = (&args[0], &args[1], &args[3]);
    let vertices = args[2].parse::<usize>().unwrap();
    let iterations = args[4].parse::<usize>().unwrap();
    assert!([3,4,8,16,32,128].contains(&vertices) && iterations > 0 && iterations <= 20_000);
    let input = points(fixture, vertices);
    let expected = query(&input, lifecycle);
    assert_eq!(expected.value(), Some(if fixture == "pi-reversed" { Sign::Negative } else { Sign::Positive }));
    assert!(format!("{expected:?}").contains("certainty: Exact"));
    for _ in 0..16 { assert_eq!(query(&input, lifecycle), expected); }
    let before = memory.map_or([0;4], |m| m(true));
    let start = Instant::now();
    for _ in 0..iterations { assert_eq!(black_box(query(&input, lifecycle)), expected); }
    let elapsed = start.elapsed().as_nanos();
    let after = memory.map_or([0;4], |m| m(false));
    println!("{}", serde_json::json!({"mode":if memory.is_some(){"allocation"}else{"cpu"},
        "variant":variant,"fixture":fixture,"vertices":vertices,"lifecycle":lifecycle,"iterations":iterations,
        "elapsed_ns":elapsed,"requests":after[0]-before[0],"requested_bytes":after[1]-before[1],
        "live_delta":after[2] as i64-before[2] as i64,"peak_delta":after[3]-before[2],
        "outcome":format!("{expected:?}")}));
}
