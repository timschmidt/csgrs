use hyperreal::{Rational, Real};
use num::{BigInt, BigUint};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Input {
    pub bucket: String,
    pub genome: [u8; 8],
}

#[derive(Clone, Copy, Debug, Serialize)]
pub enum OffsetFamily {
    PiSubtract,
    ESubtract,
    ProductAdd,
    InverseProductAdd,
}

#[derive(Clone, Debug, Serialize)]
pub struct Recipe {
    pub input: Input,
    pub scale: String,
    pub prime: u32,
    pub other_prime: u32,
    pub exponent_numerator: u32,
    pub exponent_denominator: u32,
    pub angle_numerator: u32,
    pub angle_denominator: u32,
    pub pi_power: u32,
    pub pi_offset: String,
    pub offset_family: OffsetFamily,
    pub constant_offset: String,
    pub generic_near_pi: bool,
}

impl Recipe {
    pub fn new(input: Input) -> Self {
        const PRIMES: [u32; 8] = [2, 3, 5, 7, 11, 13, 17, 19];
        const ODD_PRIMES: [u32; 8] = [5, 7, 11, 13, 17, 19, 23, 29];
        const PI_DIGITS: &str =
            "314159265358979323846264338327950288419716939937510582097494459230781640628620899";
        let g = input.genome;
        let numerator = BigInt::from(2 * u32::from(g[3]) + 1) << usize::from(g[2]);
        let numerator = if g[5] & 1 == 0 { numerator } else { -numerator };
        let denominator = BigUint::from(2 * u32::from(g[4]) + 1) << usize::from(g[6]);
        let places = 1 + usize::from(g[7]) % 78;
        let mut offset: BigUint = PI_DIGITS[..=places].parse().expect("decimal pi prefix");
        if g[5] & 2 != 0 {
            offset += 1_u8;
        }
        let angle_denominator = ODD_PRIMES[usize::from(g[0]) % ODD_PRIMES.len()];
        // ConstOffset's inner value must have a cheap positive certificate.
        // Keep near-pi cancellation in the generic Irrational population;
        // these four families exercise the actual ConstOffset constructor.
        let offset_family = match (g[5] >> 3) % 4 {
            0 => OffsetFamily::PiSubtract,
            1 => OffsetFamily::ESubtract,
            2 => OffsetFamily::ProductAdd,
            _ => OffsetFamily::InverseProductAdd,
        };
        let offset_denominator = 1 + u32::from(g[1]) + u32::from(g[7]);
        let constant_offset = match offset_family {
            OffsetFamily::PiSubtract | OffsetFamily::ESubtract => {
                let bound = if matches!(offset_family, OffsetFamily::PiSubtract) {
                    3
                } else {
                    2
                };
                let numerator = bound * (1 + u32::from(g[1]));
                format!("-{numerator}/{offset_denominator}")
            }
            OffsetFamily::ProductAdd | OffsetFamily::InverseProductAdd => {
                let numerator = BigUint::from(2 * u32::from(g[1]) + 1) << usize::from(g[7]);
                format!("{numerator}/{offset_denominator}")
            }
        };
        Self {
            scale: format!("{numerator}/{denominator}"),
            prime: PRIMES[usize::from(g[0]) % PRIMES.len()],
            other_prime: PRIMES[(usize::from(g[0]) + 1) % PRIMES.len()],
            exponent_numerator: 1 + u32::from(g[1]) % 16,
            exponent_denominator: 2 + u32::from(g[0]) % 17,
            angle_numerator: 1 + u32::from(g[1]) % ((angle_denominator - 1) / 2),
            angle_denominator,
            pi_power: 2 + u32::from(g[6]) % 5,
            pi_offset: format!("{offset}/1{}", "0".repeat(places)),
            offset_family,
            constant_offset,
            generic_near_pi: g[5] & 4 == 0,
            input,
        }
    }

    pub fn real(&self) -> Result<Real, String> {
        let rational = |n: u32, d: u32| {
            Real::new(Rational::fraction(i64::from(n), u64::from(d)).expect("positive denominator"))
        };
        let exponent = || rational(self.exponent_numerator, self.exponent_denominator);
        let angle = || rational(self.angle_numerator, self.angle_denominator);
        let pi_power = || {
            let pi = Real::pi();
            (1..self.pi_power).fold(pi.clone(), |value, _| value * &pi)
        };
        let exponential = || exponent().exp().expect("finite positive exponential input");
        let root = || Real::from(self.prime).sqrt().expect("positive prime");
        let log = || Real::from(self.prime).ln().expect("positive prime");
        let value = match self.input.bucket.as_str() {
            "One" => Real::one(),
            "Pi" => Real::pi(),
            "PiPow" => pi_power(),
            "PiInv" => Real::pi().inverse().expect("pi is nonzero"),
            "PiExp" => Real::pi() * exponential(),
            "PiInvExp" => (exponential() / Real::pi()).expect("pi is nonzero"),
            "PiSqrt" => Real::pi() * root(),
            "ConstProduct" => pi_power() * exponential(),
            "ConstOffset" => {
                let base = match self.offset_family {
                    OffsetFamily::PiSubtract => Real::pi(),
                    OffsetFamily::ESubtract => Real::e(),
                    OffsetFamily::ProductAdd => pi_power() * exponential(),
                    OffsetFamily::InverseProductAdd => {
                        (exponential() / Real::pi()).expect("pi is nonzero")
                    }
                };
                base + self
                    .constant_offset
                    .parse::<Real>()
                    .expect("exact constant offset")
            }
            "ConstProductSqrt" => pi_power() * exponential() * root(),
            "Sqrt" => root(),
            "Exp" => exponential(),
            "Ln" => log(),
            "LnAffine" => (Real::from(self.prime) * exponential())
                .ln()
                .expect("positive logarithm input"),
            "LnProduct" => log() * Real::from(self.other_prime).ln().expect("positive prime"),
            "Log10" => Real::from(self.prime).log10().expect("positive prime"),
            "Log2" => Real::from(self.angle_denominator)
                .log2()
                .expect("odd prime"),
            "Pow10" => angle().exp10().expect("finite exponent"),
            "Pow2" => angle().exp2().expect("finite exponent"),
            "SinPi" => angle().sin_pi(),
            "TanPi" => angle()
                .tan_pi()
                .expect("angle is strictly between zero and one half"),
            "Irrational" if self.generic_near_pi => {
                Real::pi() - self.pi_offset.parse::<Real>().expect("exact pi offset")
            }
            "Irrational" => angle().sin(),
            other => return Err(format!("unsupported representation: {other}")),
        };
        Ok(value
            * self
                .scale
                .parse::<Real>()
                .expect("nonzero exact rational scale"))
    }
}

pub fn private_class(value: &Real) -> Result<String, String> {
    let serialized = serde_json::to_value(value).map_err(|error| error.to_string())?;
    match &serialized["class"] {
        serde_json::Value::String(name) => Ok(name.clone()),
        serde_json::Value::Object(fields) if fields.len() == 1 => {
            Ok(fields.keys().next().unwrap().clone())
        }
        _ => Err("unexpected private-class serialization".into()),
    }
}
