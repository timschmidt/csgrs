//! Common, unsupported, nonzero and newly provable scalar query costs.
use hyperreal::{Rational, Real, RealSign};
use std::hint::black_box;
use std::time::Instant;

#[cfg(feature = "count-alloc")]
mod allocation {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
    pub struct Counted;
    static LIVE: AtomicUsize = AtomicUsize::new(0);
    static PEAK: AtomicUsize = AtomicUsize::new(0);
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    static BYTES: AtomicUsize = AtomicUsize::new(0);
    fn allocated(bytes: usize) {
        CALLS.fetch_add(1, Relaxed);
        BYTES.fetch_add(bytes, Relaxed);
        PEAK.fetch_max(LIVE.fetch_add(bytes, Relaxed) + bytes, Relaxed);
    }
    unsafe impl GlobalAlloc for Counted {
        unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
            let result = unsafe { System.alloc(layout) };
            if !result.is_null() {
                allocated(layout.size());
            }
            result
        }
        unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
            LIVE.fetch_sub(layout.size(), Relaxed);
            unsafe { System.dealloc(pointer, layout) };
        }
        unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
            let result = unsafe { System.realloc(pointer, layout, size) };
            if !result.is_null() {
                LIVE.fetch_sub(layout.size(), Relaxed);
                allocated(size);
            }
            result
        }
    }
    pub fn start() -> usize {
        let initial = LIVE.load(Relaxed);
        PEAK.store(initial, Relaxed);
        CALLS.store(0, Relaxed);
        BYTES.store(0, Relaxed);
        initial
    }
    pub fn finish(initial: usize) -> (usize, usize, i128, usize) {
        (
            CALLS.load(Relaxed),
            BYTES.load(Relaxed),
            LIVE.load(Relaxed) as i128 - initial as i128,
            PEAK.load(Relaxed) - initial,
        )
    }
}
#[cfg(feature = "count-alloc")]
#[global_allocator]
static ALLOCATOR: allocation::Counted = allocation::Counted;

enum Request {
    Normal(Real),
    Affine(Box<[[Real; 3]; 4]>),
}

enum Expected {
    Unknown,
    Rational(Rational),
    Sign(RealSign),
}

struct Recipe {
    name: &'static str,
    request: Request,
    expected: Expected,
    require: bool,
    repetitions: usize,
}

impl Recipe {
    fn query(&self) -> bool {
        match &self.request {
            Request::Normal(value) => {
                let answer = black_box(value).exact_rational_normal_form();
                if let Some(answer) = answer {
                    let Expected::Rational(expected) = &self.expected else {
                        panic!("unexpected rational certificate: {}", self.name);
                    };
                    assert_eq!(&answer, expected, "{}", self.name);
                    true
                } else {
                    false
                }
            },
            Request::Affine(points) => {
                let [a, b, c, d] = black_box(points.as_ref());
                let answer = Real::certified_affine_det3_sign(
                    a.each_ref(),
                    b.each_ref(),
                    c.each_ref(),
                    d.each_ref(),
                );
                if let Some(answer) = answer {
                    let Expected::Sign(expected) = &self.expected else {
                        panic!("unexpected orientation certificate: {}", self.name);
                    };
                    assert_eq!(&answer, expected, "{}", self.name);
                    true
                } else {
                    false
                }
            },
        }
    }
}

fn measure(recipe: &Recipe, phase: &str, count: usize, mut known: bool) -> bool {
    #[cfg(feature = "count-alloc")]
    let initial = allocation::start();
    let started = Instant::now();
    let mut certified = 0;
    for _ in 0..count {
        let answer = black_box(recipe.query());
        assert!(!known || answer, "available fact weakened: {}", recipe.name);
        assert!(
            !recipe.require || answer,
            "required fact absent: {}",
            recipe.name
        );
        known |= answer;
        certified += usize::from(answer);
    }
    let nanos = started.elapsed().as_nanos();
    #[cfg(feature = "count-alloc")]
    let (calls, bytes, retained, peak) = allocation::finish(initial);
    #[cfg(not(feature = "count-alloc"))]
    let (calls, bytes, retained, peak) = (0, 0, 0, 0);
    println!(
        "{{\"case\":\"{}\",\"phase\":\"{phase}\",\"iterations\":{count},\"certified\":{certified},\"nanos\":{nanos},\"allocation_accounting\":{},\"allocation_calls\":{calls},\"allocated_bytes\":{bytes},\"retained_bytes_delta\":{retained},\"peak_bytes_delta\":{peak}}}",
        recipe.name,
        cfg!(feature = "count-alloc")
    );
    known
}

fn fraction(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn tetrahedron(x: Real) -> [[Real; 3]; 4] {
    [
        [x, Real::zero(), Real::zero()],
        [Real::zero(), Real::one(), Real::zero()],
        [Real::zero(), Real::zero(), Real::one()],
        [Real::zero(), Real::zero(), Real::zero()],
    ]
}

fn sphere_point(index: usize) -> [Real; 3] {
    if index == 0 {
        return [Real::zero(), Real::one(), Real::zero()];
    }
    let longitude = (index - 1) / 15;
    let latitude = (index - 1) % 15 + 1;
    let theta = fraction(i64::try_from(2 * longitude).unwrap(), 32);
    let phi = fraction(i64::try_from(latitude).unwrap(), 16);
    [
        theta.clone().cos_pi() * phi.clone().sin_pi(),
        phi.clone().cos_pi(),
        theta.sin_pi() * phi.sin_pi(),
    ]
}

fn main() {
    let group = std::env::args().nth(1).expect("baseline, late or image");
    assert!(matches!(group.as_str(), "baseline" | "late" | "image"));
    let sine = fraction(1, 7).sin_pi();
    let cosine = fraction(1, 7).cos_pi();
    let pythagorean = &sine * &sine + &cosine * &cosine - Real::one();
    let double = fraction(2, 7).sin_pi() - Real::from(2) * &sine * &cosine;
    let mut recipes = vec![
        Recipe {
            name: "normal_rational",
            request: Request::Normal(fraction(1, 3)),
            expected: Expected::Rational(Rational::fraction(1, 3).unwrap()),
            require: true,
            repetitions: 262_144,
        },
        Recipe {
            name: "normal_pi_plus_e",
            request: Request::Normal(Real::pi() + Real::e()),
            expected: Expected::Unknown,
            require: false,
            repetitions: 16_384,
        },
        Recipe {
            name: "normal_sqrt5",
            request: Request::Normal(Real::from(5).sqrt().unwrap()),
            expected: Expected::Unknown,
            require: false,
            repetitions: 16_384,
        },
        Recipe {
            name: "normal_trig_nonzero",
            request: Request::Normal(sine.clone()),
            expected: Expected::Unknown,
            require: false,
            repetitions: 4_096,
        },
        Recipe {
            name: "normal_unsupported_order",
            request: Request::Normal(fraction(1, 257).sin_pi()),
            expected: Expected::Unknown,
            require: false,
            repetitions: 4_096,
        },
        Recipe {
            name: "normal_pythagorean_zero",
            request: Request::Normal(pythagorean),
            expected: Expected::Rational(Rational::zero()),
            require: group != "baseline",
            repetitions: 1_024,
        },
        Recipe {
            name: "normal_double_angle_zero",
            request: Request::Normal(double),
            expected: Expected::Rational(Rational::zero()),
            require: group != "baseline",
            repetitions: 1_024,
        },
        Recipe {
            name: "affine_dyadic",
            request: Request::Affine(Box::new(tetrahedron(Real::one()))),
            expected: Expected::Sign(RealSign::Positive),
            require: true,
            repetitions: 262_144,
        },
        Recipe {
            name: "affine_nondyadic_rational",
            request: Request::Affine(Box::new(tetrahedron(fraction(1, 3)))),
            expected: Expected::Unknown,
            require: false,
            repetitions: 262_144,
        },
        Recipe {
            name: "affine_unsupported_pi",
            request: Request::Affine(Box::new(tetrahedron(Real::pi()))),
            expected: Expected::Unknown,
            require: false,
            repetitions: 16_384,
        },
        Recipe {
            name: "affine_unsupported_sqrt5",
            request: Request::Affine(Box::new(tetrahedron(Real::from(5).sqrt().unwrap()))),
            expected: Expected::Unknown,
            require: false,
            repetitions: 16_384,
        },
        Recipe {
            name: "affine_trig_nonzero",
            request: Request::Affine(Box::new(tetrahedron(sine))),
            expected: Expected::Unknown,
            require: false,
            repetitions: 4_096,
        },
        Recipe {
            name: "affine_unsupported_order",
            request: Request::Affine(Box::new(tetrahedron(fraction(1, 257).sin_pi()))),
            expected: Expected::Unknown,
            require: false,
            repetitions: 4_096,
        },
        Recipe {
            name: "affine_original_zero",
            request: Request::Affine(Box::new([0, 1, 368, 256].map(sphere_point))),
            expected: Expected::Sign(RealSign::Zero),
            require: group == "image",
            repetitions: 1_024,
        },
    ];
    println!("scalar_group={group} cases={}", recipes.len());
    for recipe in &mut recipes {
        if cfg!(feature = "count-alloc") {
            recipe.repetitions = 128;
        }
        let known = measure(recipe, "first", 1, false);
        measure(recipe, "repeated", recipe.repetitions, known);
        // Structural metadata is printed after measurement so it cannot warm
        // a first-query cache. This coarse category is not a dispatch trace.
        match &recipe.request {
            Request::Normal(value) => println!(
                "kind case={} {:?}",
                recipe.name,
                value.detailed_facts().symbolic.kind
            ),
            Request::Affine(points) => println!(
                "kind case={} {:?}",
                recipe.name,
                points
                    .iter()
                    .flatten()
                    .map(|v| v.detailed_facts().symbolic.kind)
                    .collect::<Vec<_>>()
            ),
        }
    }
}
