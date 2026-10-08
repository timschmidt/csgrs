use hyperlimit::{PredicatePolicy, compare_reals};
use hyperreal::{Rational, Real};
use hypersolve::square_free_part;
use std::cmp::Ordering;
use std::sync::{Arc, Barrier, atomic::{AtomicBool, Ordering as AtomicOrdering}};

fn multiply(p: &mut Vec<Real>, root: &Real) {
    let mut next = vec![Real::zero(); p.len() + 1];
    for (i, c) in p.iter().enumerate() { next[i] -= c * root; next[i+1] += c; }
    *p = next;
}

fn inputs(kind: usize, code: usize, scale: Real, repeated: usize) -> (Vec<Real>, Vec<Real>, Vec<Real>) {
    let pair = match kind {
        0 => [Real::one(), Real::from(2)],
        1 => { let r = Real::from(2).sqrt().unwrap(); [r.clone(), -r] },
        2 => { let r = Real::from(2).ln().unwrap(); [r.clone(), r + Real::one()] },
        _ => unreachable!(),
    };
    let mut p = vec![scale]; let mut expected = p.clone(); let mut selected = Vec::new();
    let mut digits = code;
    for root in [Real::zero(), pair[0].clone(), pair[1].clone()] {
        let digit = digits % 3; digits /= 3;
        let exponent = if digit == 2 { repeated } else { digit };
        for _ in 0..exponent { multiply(&mut p, &root); }
        if exponent != 0 { multiply(&mut expected, &root); selected.push(root); }
    }
    (p, expected, selected)
}

fn status(a: &Real, b: &Real) -> &'static str {
    match compare_reals(a, b, PredicatePolicy::STRICT).value() {
        Some(Ordering::Equal) => "Equal",
        Some(_) => panic!("certified unequal values in a required mathematical identity: {a:?} versus {b:?}"),
        None => "Unknown",
    }
}

fn check(p: &[Real], expected: &[Real], roots: &[Real]) -> serde_json::Value {
    let Some(actual) = square_free_part(p.to_vec(), PredicatePolicy::STRICT) else {
        return serde_json::json!({"outcome":"Unknown"});
    };
    assert_eq!(actual.len(), expected.len());
    let coordinate: Vec<_> = actual.iter().zip(expected).map(|(a,b)| status(a,b)).collect();
    let projective: Vec<_> = actual.iter().zip(expected).map(|(a,b)| {
        status(&(a * expected.last().unwrap()), &(b * actual.last().unwrap()))
    }).collect();
    let residual: Vec<_> = roots.iter().map(|r| status(&Real::eval_poly(&actual,r), &Real::zero())).collect();
    let encoded = serde_json::to_string(&actual).unwrap();
    let decoded: Vec<Real> = serde_json::from_str(&encoded).unwrap();
    assert_eq!(serde_json::to_string(&decoded).unwrap(), encoded);
    let roundtrip: Vec<_> = actual.iter().zip(&decoded).map(|(a,b)| status(a,b)).collect();
    serde_json::json!({"outcome":"Known", "degree":actual.len()-1,
        "coordinate":coordinate,"projective":projective,"residual":residual,"output_roundtrip":roundtrip})
}

fn emit(kind: usize, code: usize, state: &str, result: serde_json::Value) {
    println!("{}", serde_json::json!({"kind":kind,"code":code,"state":state,"result":result}));
}

fn main() {
    let mut queries = 0;
    let mut unchanged_inputs = 0;
    for kind in 0..3 { for code in 0..27 {
        let (p, expected, roots) = inputs(kind, code, Real::from(if code % 2 == 0 { 2 } else { -2 }), 2);
        let encoded = serde_json::to_string(&p).unwrap();
        emit(kind,code,"cold",check(&p,&expected,&roots)); queries += 1;
        let decoded: Vec<Real> = serde_json::from_str(&encoded).unwrap();
        emit(kind,code,"input-roundtrip",check(&decoded,&expected,&roots)); queries += 1;
        for floor in [-32, -128, -512] {
            for c in &p { let _ = c.certified_sign_until(floor); }
            emit(kind,code,&format!("warm{floor}"),check(&p,&expected,&roots)); queries += 1;
        }
        let signal = Arc::new(AtomicBool::new(true));
        let mut aborted = p.clone();
        for c in &mut aborted {
            c.abort(signal.clone());
            // Abort permits invalid numeric observations: discard these and
            // check only the later, uncancelled exact query.
            let _ = c.certified_sign_until(-128);
        }
        signal.store(false, AtomicOrdering::Relaxed);
        emit(kind,code,"after-abort",check(&aborted,&expected,&roots)); queries += 1;
        let barrier = Barrier::new(4);
        let results = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..4).map(|worker| {
                let (p, expected, roots, encoded, barrier) = (&p,&expected,&roots,&encoded,&barrier);
                scope.spawn(move || {
                    barrier.wait();
                    (0..4).map(|iteration| {
                        let result = if iteration % 2 == 0 {
                            let local: Vec<Real> = serde_json::from_str(encoded).unwrap();
                            check(&local,expected,roots)
                        } else { check(p,expected,roots) };
                        (format!("worker{worker}-{iteration}"),result)
                    }).collect::<Vec<_>>()
                })
            }).collect();
            handles.into_iter().flat_map(|h| h.join().unwrap()).collect::<Vec<_>>()
        });
        for (state, result) in results { emit(kind,code,&state,result); queries += 1; }
        assert_eq!(serde_json::to_string(&p).unwrap(), encoded); unchanged_inputs += 1;
    } }
    // Larger multiplicities and rational scalings are separate from the
    // lifecycle corpus. In particular, the square-free output need not retain
    // the input scale: projective identities are the oracle here.
    for kind in 0..2 { for repeated in [3, 4, 8] { for code in [2, 5, 11, 14, 26] {
        for (scale_id, scale) in [Real::one(), Real::from(-7),
            Real::from(Rational::fraction(3,7).unwrap()),
            Real::from(2).powi_i64(-300).unwrap(), Real::from(2).powi_i64(300).unwrap()]
            .into_iter().enumerate() {
            let (p, expected, roots) = inputs(kind,code,scale,repeated);
            emit(kind,code,&format!("expanded{repeated}-scale{scale_id}"),check(&p,&expected,&roots));
            queries += 1;
        }
    } } }
    println!("{}", serde_json::json!({"suite":"monic-state", "queries":queries,
        "unchanged_inputs":unchanged_inputs,"workers_per_case":4}));
}
