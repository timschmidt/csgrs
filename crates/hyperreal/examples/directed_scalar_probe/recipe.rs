//! Deterministic exact recipe grammar shared by the probe and enclosure benchmark.
//!
//! Eight input bytes select rational scales, primes, angles and constant offsets;
//! the bucket chooses a construction family. Derived recipe fields retain exact
//! fraction text and the original input. Construction uses public Real operations
//! and may simplify, evaluate predicates or share constant caches. A requested
//! bucket is not proof of the resulting private class: consumers inspect it.
//!
//! Input serde validates shape and byte ranges, not bucket membership. Recipe fields
//! are public for these development tools; callers who mutate them must re-establish
//! the generated domain invariants. This is a finite adversarial grammar, not all
//! exact reals or all parameters of every certificate family.

use hyperreal::{Rational, Real};
use num::{BigInt, BigUint};
use serde::{Deserialize, Serialize};

/// Wire request selecting a construction bucket and eight-byte genome.
/// Derived serde rejects unknown fields and requires exactly eight u8 values, but
/// accepts arbitrary bucket strings; unknown buckets fail later during evaluation.
/// Clone copies the owned request and Debug displays it without constructing a scalar.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Input {
    /// Requested case-sensitive certificate label; retained verbatim and not validated by deserialization.
    pub bucket: String,
    /// Deterministic parameter bytes; several bits influence more than one recipe field.
    pub genome: [u8; 8],
}

/// Choice of a positive inner constant-offset expression before the outer signed scale.
/// Derived Clone/Copy duplicate the selector; Debug and Serialize expose its variant
/// name. There is no derived Deserialize or proof of positivity on this enum itself.
#[derive(Clone, Copy, Debug, Serialize)]
pub enum OffsetFamily {
    /// pi minus a positive rational at most three.
    PiSubtract,
    /// e minus a positive rational at most two.
    ESubtract,
    /// pi^n * exp(q) plus a positive rational.
    ProductAdd,
    /// exp(q)/pi plus a positive rational.
    InverseProductAdd,
}

/// Expanded exact construction record, normally produced by new from the input genome.
/// Derived Clone duplicates fields and strings; Debug/Serialize expose the record
/// without evaluating a Real. No deserializer or automatic field validation is derived.
/// The public fields can be changed inconsistently with input; evaluators trust them.
#[derive(Clone, Debug, Serialize)]
pub struct Recipe {
    /// Original request retained for identity and bucket selection; not recomputed from derived fields.
    pub input: Input,
    /// Nonzero signed ratio `((2*g[3]+1)*2^g[2])/((2*g[4]+1)*2^g[6])`; `g[5]` bit 0 selects minus.
    pub scale: String,
    /// One of 2, 3, 5, 7, 11, 13, 17, 19 selected by `g[0]` modulo eight.
    pub prime: u32,
    /// The next prime in the same cyclic table, used for the second logarithm.
    pub other_prime: u32,
    /// Positive exponential numerator `1 + (g[1] % 16)`, in 1..=16.
    pub exponent_numerator: u32,
    /// Exponential denominator `2 + (g[0] % 17)`, in 2..=18.
    pub exponent_denominator: u32,
    /// Numerator `1 + (g[1] % ((angle_denominator-1)/2))`; gives 0 < angle < 1/2.
    pub angle_numerator: u32,
    /// Odd prime from 5, 7, 11, 13, 17, 19, 23, 29 selected by `g[0]` modulo eight.
    pub angle_denominator: u32,
    /// Exponent `2 + (g[6] % 5)`, in 2..=6.
    pub pi_power: u32,
    /// One-to-78-place decimal prefix of pi, optionally increased by one decimal unit using `g[5]` bit 1.
    pub pi_offset: String,
    /// Constant-offset construction selected by `g[5]` bits 3 and 4.
    pub offset_family: OffsetFamily,
    /// Exact signed rational added to the chosen constant product; new ensures its inner result is positive.
    pub constant_offset: String,
    /// True when `g[5]` bit 2 is clear: opaque bucket uses pi-pi_offset; otherwise it uses sin(angle).
    pub generic_near_pi: bool,
}

/// Expand the byte grammar and construct its exact scalar definition without replacing it by samples.
impl Recipe {
    /// Decode the eight bytes with finite tables, shifts and exact fraction strings.
    /// The scale sign uses `g[5]` bit 0; the decimal pi side uses bit 1, opaque family
    /// bit 2, and constant-offset family bits 3..4. The same bytes also set other
    /// parameters, so selections are correlated rather than independent samples.
    ///
    /// For subtraction families, the offset is `-b*(1+g[1])/(1+g[1]+g[7])`, with b=3
    /// for pi and b=2 for e, leaving a positive inner expression. Addition families
    /// use `(2*g[1]+1)*2^g[7]/(1+g[1]+g[7])`. Preserve numerator/denominator text even
    /// when reducible. Bucket membership is not checked. Fixed byte ranges bound these
    /// integer constructions, not subsequent Real evaluation; allocation failure is
    /// not a typed outcome.
    pub fn new(input: Input) -> Self {
        /// Positive prime radicands/log bases; cyclic adjacency chooses distinct logarithm bases.
        const PRIMES: [u32; 8] = [2, 3, 5, 7, 11, 13, 17, 19];
        /// Angle denominators above three, avoiding rational pi-angle special cases in this grammar.
        const ODD_PRIMES: [u32; 8] = [5, 7, 11, 13, 17, 19, 23, 29];
        /// Decimal digits of pi without a point, used only as exact rational offsets with up to 78 places.
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

    /// Build the bucket's exact expression and multiply by scale last. Rational
    /// parameters enter as ratios, not host floats. Repeated pi multiplication, roots,
    /// logs, exponentials and trig operations may normalize or refine during construction;
    /// returned representation tags must be checked separately by consumers.
    ///
    /// Assumes fields produced by new: nonzero fraction denominators, positive primes,
    /// pi_power >= 2, and 0 < angle < 1/2. The opaque branch is either pi minus the
    /// exact decimal offset or sin(angle) in radians. Return a string for an unknown
    /// bucket; expected parse/domain operations panic if manually changed fields break
    /// the grammar. There is no total-work budget, cancellation or cached-result reset.
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

/// Serialize a Real and extract its class name from a string or single-key object.
/// Propagate serde errors and reject unexpected class shapes. This depends on the
/// private wire representation; the returned diagnostic name is neither a scalar
/// equality certificate nor validation that the value matches a requested recipe.
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
