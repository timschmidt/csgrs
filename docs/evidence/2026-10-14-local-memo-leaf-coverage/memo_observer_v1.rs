fn record_memo_payload<const RETAIN: bool>(
    approximation: &Approximation,
    value: &Option<PiLaurentPolynomial>,
    event: crate::memo_leaf_trace::Event,
) {
    use crate::memo_leaf_trace::{Payload, Shape};
    if !crate::memo_leaf_trace::active() {
        return;
    }
    let kind = match approximation {
        Approximation::Int(_) => 0,
        Approximation::One => 1,
        Approximation::Ratio(_) => 2,
        Approximation::Constant(constant) => match constant {
            SharedConstant::Pi => 3,
            SharedConstant::InvPi => 4,
            SharedConstant::Tau => 5,
            SharedConstant::Sqrt2 => 6,
            SharedConstant::Sqrt3 => 7,
            _ => 8,
        },
        _ => 9,
    };
    let payload = match value {
        None => Payload {
            shape: Shape::None,
            terms: 0,
            radicals: 0,
            atoms: 0,
        },
        Some(polynomial) => {
            let terms = polynomial.terms.len();
            let mut radicals = 0;
            let mut atoms = 0;
            for (_, coefficient) in &polynomial.terms {
                radicals += coefficient.radicals.len();
                atoms += coefficient.atoms.len();
            }
            let shape = if atoms != 0 {
                Shape::Opaque
            } else if terms == 0 {
                Shape::Empty
            } else if terms == 1 && radicals == 0 {
                Shape::OneRationalTerm
            } else if terms == 1 && radicals == 1 {
                Shape::OneRadicalTerm
            } else {
                Shape::OtherClosed
            };
            Payload {
                shape,
                terms,
                radicals,
                atoms,
            }
        }
    };
    crate::memo_leaf_trace::record(kind, RETAIN, event, payload);
}
