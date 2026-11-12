//! Deterministic, completed-work fixtures shared by tests and cost/size probes.
use alumina_foc::{AlphaBeta, DqInterval, FocError, Phase3, Q30, Q30Interval, Rotation};

/// One balanced phase source, two frame sources and validated rotation.
#[derive(Clone, Copy)]
pub struct Sample {
    /// Phase inputs with a zero-sequence range of [-3,3] lattice units.
    pub phases: Phase3,
    /// Stationary-frame input.
    pub stationary: AlphaBeta,
    /// Rotating-frame input.
    pub rotating: DqInterval,
    /// Validated angle evidence.
    pub rotation: Rotation,
}

/// The admitted zero-sequence tolerance for the sample phases.
pub const BALANCE: Q30 = Q30::from_bits(3);

/// Generates one of 64 small, nontrivial successful transform/chain workloads.
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
#[derive(Clone, Copy, Debug)]
pub enum Action {
    /// Forward Clarke.
    Clarke,
    /// Forward Park.
    Park,
    /// Inverse Park.
    InversePark,
    /// Inverse Clarke.
    InverseClarke,
    /// Clarke, Park, inverse Park, inverse Clarke, and Clarke again.
    Chain,
}

/// All actions in deterministic reporting order.
pub const ACTIONS: [Action; 5] = [
    Action::Clarke,
    Action::Park,
    Action::InversePark,
    Action::InverseClarke,
    Action::Chain,
];

/// Complete output retaining the domain wrapper.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Output {
    /// Stationary frame.
    Stationary(AlphaBeta),
    /// Rotating frame.
    Rotating(DqInterval),
    /// Three phases.
    Phases(Phase3),
}

impl Output {
    /// Deterministic digest of every endpoint and the output tag; equality tests
    /// compare the original outputs separately, not just this checksum.
    pub fn digest(self, mut hash: u64) -> u64 {
        let (tag, lanes) = match self {
            Self::Stationary(v) => (1, [v.alpha, v.beta, Q30Interval::ZERO]),
            Self::Rotating(v) => (2, [v.d, v.q, Q30Interval::ZERO]),
            Self::Phases(v) => (3, [v.a, v.b, v.c]),
        };
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

// The macro shares only the identical composition client, not any arithmetic.
macro_rules! provider {
    ($name:ident, $ops:ident, $doc:literal) => {
        #[doc = $doc]
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
