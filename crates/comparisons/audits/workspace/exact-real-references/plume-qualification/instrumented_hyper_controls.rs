use hyperreal::{Computable, Rational};
use rug::{Integer, Rational as Q};

fn decimal(text: &str) -> Q {
    let (negative, body) = text.strip_prefix('-').map_or((false, text), |s| (true, s));
    let (whole, fraction) = body.split_once('.').unwrap_or((body, ""));
    assert!(!whole.is_empty() && whole.bytes().chain(fraction.bytes()).all(|x| x.is_ascii_digit()));
    let mut numerator = Integer::from_str_radix(&format!("{whole}{fraction}"), 10).unwrap();
    if negative { numerator = -numerator; }
    Q::from((numerator, Integer::from(10_u64.pow(fraction.len() as u32))))
}

fn main() {
    let mut checks = 0;
    for a in -32..=32 {
        for d in [1, 10, 100] {
            let expected = Q::from((a, d));
            for places in [1_usize, 4, 9] {
                for warmed in [false, true] {
                    let input: Rational = expected.to_string().parse().unwrap();
                    let x = Computable::rational(input);
                    if warmed {
                        let _ = x.approx(-512);
                        let _ = x.approx(-2);
                    }
                    let text = format!("{x:.places$}");
                    // Parse the produced decimal directly with independent
                    // integer arithmetic, not Hyper's own decimal parser.
                    let actual = decimal(&text);
                    let radius = Q::from((1, Integer::from(10_u64.pow(places as u32))));
                    assert!((actual - &expected).abs() <= radius,
                        "a={a} d={d} places={places} warmed={warmed} output={text}");
                    checks += 1;
                }
            }
        }
    }
    println!("PASS exact decimal output controls={checks}; 195 inputs, 3 places, 2 cache histories");
}
