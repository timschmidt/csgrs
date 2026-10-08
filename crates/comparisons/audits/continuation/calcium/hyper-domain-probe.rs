use hyperreal::{Problem, Rational, Real};

fn main() {
    let mut inputs: Vec<(Real, bool)> = [-4, -2, -1, 0, 1, 2, 4]
        .into_iter()
        .map(|n| (Real::new(Rational::fraction(n, 2).unwrap()), n.abs() <= 2))
        .collect();
    let epsilon = Real::new(Rational::fraction(1, 1_048_576).unwrap());
    for sign in [-1, 1] {
        inputs.push((Real::from(sign) * (Real::one() - &epsilon), true));
        inputs.push((Real::from(sign) * (Real::one() + &epsilon), false));
    }
    let radical = Real::from(2).sqrt().unwrap();
    let half = Real::new(Rational::fraction(1, 2).unwrap());
    for sign in [-1, 1] {
        inputs.push((Real::from(sign) * &radical, false));
        inputs.push((Real::from(sign) * &radical * &half, true));
    }
    inputs.push((Real::pi(), false));
    inputs.push((-Real::pi(), false));
    assert_eq!(inputs.len(), 17);
    let mut rows = 0;
    println!("input,state,operation,expected_valid,result");
    for (index, (original, valid)) in inputs.into_iter().enumerate() {
        for state in 0..3 {
            let value = match state {
                0 => original.clone(),
                1 => serde_json::from_str::<Real>(&serde_json::to_string(&original).unwrap()).unwrap(),
                _ => {
                    let warm = original.clone();
                    warm.certified_dyadic_interval(-128).unwrap();
                    warm
                }
            };
            for operation in ["asin", "acos"] {
                let result = if operation == "asin" { value.clone().asin() } else { value.clone().acos() };
                let outcome = match result {
                    Ok(answer) => {
                        assert!(valid, "out-of-domain success {index}/{state}/{operation}");
                        answer.certified_dyadic_interval(-80).unwrap();
                        "Ok"
                    }
                    Err(Problem::NotANumber) => {
                        assert!(!valid, "in-domain rejection {index}/{state}/{operation}");
                        "NotANumber"
                    }
                    Err(e) => panic!("unexpected error {index}/{state}/{operation}: {e:?}"),
                };
                println!("{index},{state},{operation},{valid},{outcome}");
                rows += 1;
            }
        }
    }
    assert_eq!(rows, 102);
    println!("{{\"suite\":\"hyper-domain\",\"rows\":{rows}}}");
}
