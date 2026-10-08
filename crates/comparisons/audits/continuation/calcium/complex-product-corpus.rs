use hyperlattice::Complex;
use hyperreal::{Rational, Real};
use num::{BigInt, BigRational, BigUint, Integer, One, Zero};
use std::hint::black_box;
use std::time::Instant;

pub const FAMILIES: [&str; 9] = [
    "dense", "real-cancel", "imag-cancel", "near-cancel", "sum-zero",
    "unbalanced", "mixed-scale", "sparse", "zero",
];

pub struct Fixture {
    pub nums: [BigInt; 4],
    pub dens: [BigUint; 4],
}

pub fn fixture(bits: usize, family: &str, scale: &str, mask: u8) -> Fixture {
    assert!(bits >= 8 && bits <= 65_536);
    assert!(FAMILIES.contains(&family));
    assert!(mask < 16);
    let mut state = 0x8317_9821_61aa_7383_u64 ^ bits as u64;
    let mut dense = || {
        let digits: Vec<u32> = (0..bits.div_ceil(32)).map(|_| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state as u32
        }).collect();
        (BigUint::new(digits) & ((BigUint::one() << bits) - 1_u8))
            | (BigUint::one() << (bits - 1)) | BigUint::one()
    };
    let t = dense();
    let nums = match family {
        "dense" | "mixed-scale" => [t, dense(), dense(), dense()],
        "real-cancel" => { let u = dense(); [t.clone(), u.clone(), u, t] },
        "imag-cancel" => { let u = dense(); [t.clone(), u.clone(), t, u] },
        "near-cancel" => [t.clone(), &t + 2_u8, &t + 4_u8, &t + 2_u8],
        "sum-zero" => [t.clone(), t.clone(), &t + 4_u8, &t + 6_u8],
        "unbalanced" => [t.clone(), BigUint::from(3_u8), &t + 4_u8, BigUint::from(5_u8)],
        "sparse" => std::array::from_fn(|i| (BigUint::one() << (bits - 1)) + (2 * i + 1)),
        "zero" => [t.clone(), BigUint::zero(), &t + 4_u8, &t + 6_u8],
        _ => unreachable!(),
    };
    let choose_odd = |pair: &[BigUint], seed: u32| {
        let mut d = BigUint::from(seed);
        while pair.iter().filter(|n| !n.is_zero()).any(|n| !n.gcd(&d).is_one()) { d += 2_u8; }
        d
    };
    let (left, right) = match scale {
        "integer" => (BigUint::one(), BigUint::one()),
        "dyadic" => (BigUint::one() << (bits / 2 + 7), BigUint::one() << (bits / 3 + 9)),
        "odd" => (choose_odd(&nums[..2], 65_537), choose_odd(&nums[2..], 65_539)),
        _ => panic!("unknown scale"),
    };
    let mut dens = [left.clone(), left, right.clone(), right];
    if family == "mixed-scale" { dens[1] += 2_u8; dens[3] += 4_u8; }
    Fixture {
        nums: std::array::from_fn(|i| BigInt::from_biguint(
            if mask & (1 << i) == 0 { num::bigint::Sign::Plus } else { num::bigint::Sign::Minus },
            nums[i].clone(),
        )),
        dens,
    }
}

pub struct Inputs {
    pub r: [Rational; 4],
    pub z: [Complex; 2],
}

impl Fixture {
    pub fn inputs(&self) -> Inputs {
        let r: [Rational; 4] = std::array::from_fn(|i|
            Rational::from_bigint_fraction(self.nums[i].clone(), self.dens[i].clone()).unwrap());
        let z = std::array::from_fn(|i| Complex::new(Real::new(r[2*i].clone()), Real::new(r[2*i+1].clone())));
        Inputs { r, z }
    }
    pub fn expected(&self, quotient: bool) -> [BigRational; 2] {
        let [a,b,c,d] = std::array::from_fn(|i|
            BigRational::new(self.nums[i].clone(), self.dens[i].clone().into()));
        if quotient {
            let norm = &c * &c + &d * &d;
            [(&a * &c + &b * &d) / &norm, (&b * &c - &a * &d) / norm]
        } else { [&a * &c - &b * &d, &a * &d + &b * &c] }
    }
}

pub fn calculate(input: &Inputs, route: &str) -> (Rational, Rational) {
    let [a,b,c,d] = &input.r;
    match route {
        "product" => Rational::complex_product_components([a,b],[c,d]),
        "quotient" => Rational::complex_quotient_components([a,b],[c,d]).unwrap(),
        "lattice" => {
            let z = &input.z[0] * &input.z[1];
            (z.re.exact_rational().unwrap(), z.im.exact_rational().unwrap())
        },
        _ => panic!("unknown route"),
    }
}

pub fn as_ratio(x: &Rational) -> BigRational {
    BigRational::new(BigInt::from_biguint(x.sign(), x.numerator().clone()), x.denominator().clone().into())
}

pub fn run(snapshot: Option<fn(bool) -> [usize; 4]>) {
    let args: Vec<String> = std::env::args().collect();
    assert_eq!(args.len(), 8, "bits family scale mask route lifecycle iterations");
    let bits: usize = args[1].parse().unwrap();
    let mask: u8 = args[4].parse().unwrap();
    let count: usize = args[7].parse().unwrap();
    assert!((1..=4096).contains(&count));
    let f = fixture(bits, &args[2], &args[3], mask);
    let expected = f.expected(args[5] == "quotient");
    let reference = calculate(&f.inputs(), &args[5]);
    assert_eq!([as_ratio(&reference.0), as_ratio(&reference.1)], expected);
    drop(reference);
    let fresh = match args[6].as_str() { "fresh" => true, "reused" => false, _ => panic!("lifecycle") };
    // Cold kernel inputs are separately constructed before the timed window.
    // This excludes construction, pool lifetime and all oracle work from cost.
    let inputs: Vec<Inputs> = (0..if fresh {count} else {1}).map(|_| f.inputs()).collect();
    if !fresh { for _ in 0..4 { drop(black_box(calculate(&inputs[0], &args[5]))); } }
    let before = snapshot.map(|s| s(true));
    let start = Instant::now();
    for i in 0..count { drop(black_box(calculate(black_box(&inputs[if fresh {i} else {0}]), &args[5]))); }
    let elapsed = start.elapsed().as_nanos();
    let after = snapshot.map(|s| s(false));
    let allocation = match (before, after) {
        (Some(a),Some(b)) => format!("[{}, {}, {}, {}]", b[0]-a[0], b[1]-a[1], b[2] as i128-a[2] as i128, b[3].saturating_sub(a[2])),
        _ => "null".into(),
    };
    println!("{{\"bits\":{bits},\"family\":\"{}\",\"scale\":\"{}\",\"mask\":{mask},\"route\":\"{}\",\"lifecycle\":\"{}\",\"iterations\":{count},\"ns\":{elapsed},\"allocation\":{allocation},\"preflight\":true}}",args[2],args[3],args[5],args[6]);
}
