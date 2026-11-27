//! Deterministic, completed-work fixtures shared by tests and cost/size probes.
//!
//! The usual probe window is 64 indices, but sample construction depends on the
//! two residues modulo 33 and five. Dispatch retains complete domain wrappers and
//! each transform's failure boundary. Native/shared agreement can reuse the same
//! scalar kernel and is separate from an independent arithmetic enclosure oracle.
use alumina_foc::{AlphaBeta, DqInterval, FocError, Phase3, Q30, Q30Interval, Rotation};

/// Phase, stationary-frame and rotating-frame sources with a validated rotation.
/// Public fields also permit caller-authored combinations, so provider calls remain
/// fallible even though the generated fixture corpus is intended to complete.
#[derive(Clone, Copy)]
pub struct Sample {
    /// Phase source; generated samples have residual bounds of minus/plus three raw units.
    pub phases: Phase3,
    /// Stationary-frame input.
    pub stationary: AlphaBeta,
    /// Rotating-frame input.
    pub rotating: DqInterval,
    /// Validated sine/cosine and norm evidence, reused by both directions of Park.
    pub rotation: Rotation,
}

/// The admitted zero-sequence tolerance for the sample phases.
/// Three raw Q30 units, i.e. 3/2^30; equality at the residual endpoints is accepted.
pub const BALANCE: Q30 = Q30::from_bits(3);

/// Construct bounded transform fixtures from index%33 and index%5 (165 residue combinations).
///
/// Set n=index%33-16 and U=2^24 raw Q30 units (real 1/64). Phase centers are
/// n*U, -(n/2)*U and their negated sum, with integer n/2 truncated toward zero.
/// Each phase/frame lane has radius one raw unit. Stationary centers are
/// (n*U,(16-abs(n))*U); rotating centers are (n*U,(n+8)*U/2).
///
/// Rotation choices are (sine,cosine)=(0,1),(1,0),(-1,0),(0,-1), or outward
/// brackets of (3/5,4/5). Validate them with a maximum squared-norm error of
/// sixteen raw units. Constructors unwrap these bounded invariants. The function
/// accepts any u32 index; the common 0..64 probe window is only part of its domain.
pub fn sample(index: u32) -> Sample {
    let n = (index % 33) as i32 - 16;
    let unit = 1 << 24;
    let interval =
        |center| Q30Interval::new(Q30::from_bits(center - 1), Q30::from_bits(center + 1)).unwrap();
    let a = n * unit;
    let b = -(n / 2) * unit;
    let phases = Phase3 {
        a: interval(a),
        b: interval(b),
        c: interval(-a - b),
    };
    let stationary = AlphaBeta {
        alpha: interval(a),
        beta: interval((16 - n.abs()) * unit),
    };
    let rotating = DqInterval {
        d: interval(a),
        q: interval((n + 8) * unit / 2),
    };
    let (sine, cosine) = match index % 5 {
        0 => (Q30Interval::ZERO, Q30Interval::ONE),
        1 => (Q30Interval::ONE, Q30Interval::ZERO),
        2 => (Q30Interval::point(Q30::NEG_ONE), Q30Interval::ZERO),
        3 => (Q30Interval::ZERO, Q30Interval::point(Q30::NEG_ONE)),
        _ => (
            Q30Interval::from_ratio(3, 5).unwrap(),
            Q30Interval::from_ratio(4, 5).unwrap(),
        ),
    };
    let rotation = Rotation::new(sine, cosine, Q30::from_bits(16)).unwrap();
    Sample {
        phases,
        stationary,
        rotating,
        rotation,
    }
}

/// Named completed workload, identical for both implementations.
/// Single transforms use their corresponding Sample field. Chain starts from phases
/// and feeds each result to the next operation, reusing the supplied rotation.
#[derive(Clone, Copy, Debug)]
pub enum Action {
    /// Forward Clarke on phases with BALANCE; return a stationary-frame output.
    Clarke,
    /// Forward Park on stationary with rotation; return a rotating-frame output.
    Park,
    /// Inverse Park on rotating with rotation; return a stationary-frame output.
    InversePark,
    /// Inverse Clarke on stationary; return three phase intervals.
    InverseClarke,
    /// Clarke, Park, inverse Park, inverse Clarke, and Clarke again.
    /// First Clarke uses BALANCE; the final call admits 2^20 raw units of residual
    /// after intermediate widening. This is a composition workload, not an identity assertion.
    Chain,
}

/// All actions in deterministic reporting order.
/// Clarke, Park, inverse Park, inverse Clarke, then the five-transform chain.
pub const ACTIONS: [Action; 5] = [
    Action::Clarke,
    Action::Park,
    Action::InversePark,
    Action::InverseClarke,
    Action::Chain,
];

/// Complete output retaining the domain wrapper.
/// Derived equality compares tags and all stored interval endpoints. Digests are
/// supplementary diagnostics and cannot replace full equality or enclosure checks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Output {
    /// Stationary frame.
    Stationary(
        /// Complete alpha/beta result in the stationary frame.
        AlphaBeta,
    ),
    /// Rotating frame.
    Rotating(
        /// Complete direct/quadrature result in the rotating frame.
        DqInterval,
    ),
    /// Three phases.
    Phases(
        /// Complete a/b/c phase result.
        Phase3,
    ),
}

/// Stable conformance tags and endpoint hashing for complete transform results.
impl Output {
    /// Output tag and ordered Q30 lanes used by the conformance ABI.
    /// Two-axis frames have a zero third lane.
    /// Tags are 1 for (alpha,beta,zero), 2 for (d,q,zero), and 3 for (a,b,c).
    /// Padding is an exact zero interval, not an extra physical phase or failure sentinel.
    pub fn tagged_lanes(self) -> (u8, [Q30Interval; 3]) {
        match self {
            Self::Stationary(v) => (1, [v.alpha, v.beta, Q30Interval::ZERO]),
            Self::Rotating(v) => (2, [v.d, v.q, Q30Interval::ZERO]),
            Self::Phases(v) => (3, [v.a, v.b, v.c]),
        }
    }

    /// Deterministic digest of every endpoint and the output tag; equality tests
    /// compare the original outputs separately, not just this checksum.
    /// Starting from the caller's hash, process the one-byte tag followed by six
    /// signed i32 endpoints in lane order, lower before upper, each little-endian.
    /// XOR each byte then multiply by the FNV-1a prime modulo 2^64. The caller
    /// chooses the initial seed; this digest is neither injective nor cryptographic.
    pub fn digest(self, mut hash: u64) -> u64 {
        let (tag, lanes) = self.tagged_lanes();
        for byte in core::iter::once(tag).chain(
            lanes
                .into_iter()
                .flat_map(|v| [v.lower().bits(), v.upper().bits()])
                .flat_map(i32::to_le_bytes),
        ) {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
        hash
    }
}

/// Generate identical action dispatch/composition for one provider without duplicating formulas.
macro_rules! provider {
    ($name:ident, $ops:ident, $doc:literal) => {
        #[doc = $doc]
        /// Dispatch the selected workload and preserve its complete wrapper or first FocError.
        /// Single actions use the corresponding source field. Chain uses phases and
        /// rotation, passes BALANCE to its first Clarke, and 2^20 raw units to its
        /// final Clarke after widening. No later operation runs after an error;
        /// no intermediate output is returned. Caller-authored Sample values need
        /// not satisfy the generated corpus's balance or range conditions.
        pub fn $name(action: Action, sample: Sample) -> Result<Output, FocError> {
            match action {
                Action::Clarke => $ops::clarke(sample.phases, BALANCE).map(Output::Stationary),
                Action::Park => {
                    $ops::park(sample.stationary, sample.rotation).map(Output::Rotating)
                }
                Action::InversePark => {
                    $ops::inverse_park(sample.rotating, sample.rotation).map(Output::Stationary)
                }
                Action::InverseClarke => {
                    $ops::inverse_clarke(sample.stationary).map(Output::Phases)
                }
                Action::Chain => {
                    let a = $ops::clarke(sample.phases, BALANCE)?;
                    let dq = $ops::park(a, sample.rotation)?;
                    let b = $ops::inverse_park(dq, sample.rotation)?;
                    let phases = $ops::inverse_clarke(b)?;
                    $ops::clarke(phases, Q30::from_bits(1 << 20)).map(Output::Stationary)
                }
            }
        }
    };
}
provider!(
    native,
    alumina_foc,
    "Runs the current firmware implementation."
);
provider!(
    shared,
    crate,
    "Runs the shared-kernel conformance implementation."
);
