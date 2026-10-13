//! Diagnostic-only observation of existing exact certificates.
//!
//! Temporary radicand clones and cache read guards are released before the
//! original query. Counters retain only integers, never expressions or proofs.
use super::*;
use std::cell::RefCell;

const OPERATIONS: usize = 13;
const ROOT_KINDS: usize = 5;
const CHILD_KINDS: usize = 7;
const LANES: usize = 9;
const ROWS: usize = OPERATIONS * ROOT_KINDS * CHILD_KINDS * CHILD_KINDS * LANES * 2;

// Child kinds: unavailable, intrinsic rational leaf, retained rational,
// retained quadratic, retained multiradical, retained Laurent, absent operand.
struct Snapshot {
    kind: usize,
    radicand: Option<Rational>,
}

impl Snapshot {
    fn absent() -> Self {
        Self {
            kind: 6,
            radicand: None,
        }
    }

    fn retained(&self) -> bool {
        matches!(self.kind, 2..=5)
    }

    fn rational(&self) -> bool {
        matches!(self.kind, 1 | 2)
    }

    fn covered(&self) -> bool {
        matches!(self.kind, 1..=3)
    }
}

fn certificate(value: &Computable) -> Snapshot {
    let missing = || Snapshot {
        kind: 0,
        radicand: None,
    };
    let Some(cell) = value.internal.cache.cell() else {
        return missing();
    };
    let guard = cell.read().unwrap_or_else(|error| error.into_inner());
    let Some(polynomial) = guard.pi_laurent_polynomial.as_deref() else {
        return missing();
    };
    match polynomial.terms.as_slice() {
        [] => Snapshot {
            kind: 2,
            radicand: None,
        },
        [(0, coefficient)] => match coefficient.radicals.as_slice() {
            [] => Snapshot {
                kind: 2,
                radicand: None,
            },
            [(radicand, _)] => Snapshot {
                kind: 3,
                radicand: Some(radicand.clone()),
            },
            _ => Snapshot {
                kind: 4,
                radicand: None,
            },
        },
        _ => Snapshot {
            kind: 5,
            radicand: None,
        },
    }
}

fn operand(value: &Computable) -> Snapshot {
    let mut snapshot = certificate(value);
    if snapshot.kind == 0
        && matches!(
            value.internal.approximation,
            Approximation::One | Approximation::Int(_) | Approximation::Ratio(_)
        )
    {
        snapshot.kind = 1;
    }
    snapshot
}

#[derive(Clone, Copy)]
pub(super) struct Observation {
    operation: usize,
    root: usize,
    left: usize,
    right: usize,
    lane: usize,
    retained_mask: usize,
}

/// Classify one root and its immediate operands without recognizing a subtree.
pub(super) fn classify(value: &Computable) -> Observation {
    let root_snapshot = certificate(value);
    let root = match root_snapshot.kind {
        2..=5 => root_snapshot.kind - 1,
        _ => 0,
    };
    let mut left = Snapshot::absent();
    let mut right = Snapshot::absent();
    let mut retained_mask = 0;
    let operation = match &value.internal.approximation {
        Approximation::One => 1,
        Approximation::Int(_) => 2,
        Approximation::Ratio(_) => 3,
        Approximation::Constant(_) => 4,
        Approximation::Add(a, b) | Approximation::Multiply(a, b) => {
            left = operand(a);
            right = operand(b);
            if matches!(value.internal.approximation, Approximation::Add(_, _)) {
                5
            } else {
                6
            }
        },
        Approximation::Negate(child)
        | Approximation::Offset(child, _)
        | Approximation::Inverse(child)
        | Approximation::Square(child)
        | Approximation::Sqrt(child) => {
            left = operand(child);
            match value.internal.approximation {
                Approximation::Negate(_) => 7,
                Approximation::Offset(_, _) => 8,
                Approximation::Inverse(_) => 9,
                Approximation::Square(_) => 10,
                _ => 11,
            }
        },
        Approximation::LinearCombination3(combination) => {
            for (index, child) in combination
                .coefficients
                .iter()
                .chain(combination.values.iter())
                .enumerate()
            {
                if certificate(child).retained() {
                    retained_mask |= 1 << index;
                }
            }
            12
        },
        _ => 0,
    };
    if operation != 12 {
        retained_mask = usize::from(left.retained()) | (usize::from(right.retained()) << 1);
    }
    let mut lane = match operation {
        5 if left.rational() && right.rational() => 2,
        5 if (left.kind == 3 && right.rational()) || (right.kind == 3 && left.rational()) => 3,
        5 if left.kind == 3 && right.kind == 3 && left.radicand == right.radicand => 4,
        6 if (left.rational() && right.covered()) || (right.rational() && left.covered()) => 7,
        7 if left.covered() => 5,
        8 if left.covered()
            && matches!(
                value.internal.approximation,
                Approximation::Offset(_, -64..=64)
            ) =>
        {
            6
        },
        _ => 0,
    };
    if lane != 0 && retained_mask == 0 {
        lane = 8;
    }
    if matches!(root, 1 | 2) {
        lane = 1;
    } else if root != 0 {
        // A wider retained form is a decline from this narrow consumer,
        // not a negative value fact. The original query still runs below.
        lane = 0;
    }
    Observation {
        operation,
        root,
        left: left.kind,
        right: right.kind,
        lane,
        retained_mask,
    }
}

struct Counters {
    calls: Vec<u64>,
    nanos: Vec<u128>,
    masks: Vec<u64>,
}

impl Default for Counters {
    fn default() -> Self {
        Self {
            calls: vec![0; ROWS],
            nanos: vec![0; ROWS],
            masks: vec![0; OPERATIONS * 64 * 2],
        }
    }
}

impl Drop for Counters {
    fn drop(&mut self) {
        for (index, &calls) in self
            .calls
            .iter()
            .enumerate()
            .filter(|(_, count)| **count != 0)
        {
            let success = index % 2;
            let rest = index / 2;
            let lane = rest % LANES;
            let rest = rest / LANES;
            let right = rest % CHILD_KINDS;
            let rest = rest / CHILD_KINDS;
            let left = rest % CHILD_KINDS;
            let rest = rest / CHILD_KINDS;
            let root = rest % ROOT_KINDS;
            let operation = rest / ROOT_KINDS;
            eprintln!(
                "QUADRATIC_SHAPE_DIAGNOSTIC {operation} {root} {left} {right} {lane} {success} {calls} {}",
                self.nanos[index],
            );
        }
        for (index, &calls) in self
            .masks
            .iter()
            .enumerate()
            .filter(|(_, count)| **count != 0)
        {
            eprintln!(
                "QUADRATIC_CHILD_MASK_DIAGNOSTIC {} {} {} {calls}",
                index / (64 * 2),
                (index / 2) % 64,
                index % 2,
            );
        }
    }
}

thread_local! {
    static COUNTERS: RefCell<Counters> = RefCell::new(Counters::default());
}

pub(super) fn record(observation: Observation, success: bool, nanos: u128) {
    let Observation {
        operation,
        root,
        left,
        right,
        lane,
        retained_mask,
    } = observation;
    let success = usize::from(success);
    let index = (((((operation * ROOT_KINDS + root) * CHILD_KINDS + left) * CHILD_KINDS
        + right)
        * LANES
        + lane)
        * 2)
        + success;
    COUNTERS.with(|counters| {
        let mut counters = counters.borrow_mut();
        counters.calls[index] += 1;
        counters.nanos[index] += nanos;
        counters.masks[(operation * 64 + retained_mask) * 2 + success] += 1;
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw(approximation: Approximation) -> Computable {
        Computable {
            internal: Arc::new(Node::new(
                approximation,
                BoundCache::Invalid,
                ExactSignCache::Unknown,
            )),
            signal: None,
        }
    }

    fn warm_root(radicand: i64) -> Computable {
        let root = raw(Approximation::Sqrt(Computable::rational(Rational::new(
            radicand,
        ))));
        let clone = root.clone();
        assert!(clone.exact_quadratic_surd_parts().is_some());
        root
    }

    #[test]
    fn observation_is_shallow_and_releases_all_owners() {
        let root = warm_root(13);
        let other = warm_root(17);
        let rows = [
            (raw(Approximation::Add(root.clone(), root.clone())), 4, 3),
            (raw(Approximation::Add(root.clone(), other.clone())), 0, 3),
            (raw(Approximation::Add(Computable::one(), root.clone())), 3, 2),
            (
                raw(Approximation::Multiply(Computable::one(), root.clone())),
                7,
                2,
            ),
            (raw(Approximation::Negate(root.clone())), 5, 1),
            (raw(Approximation::Offset(root.clone(), 64)), 6, 1),
            (raw(Approximation::Offset(root.clone(), 65)), 0, 1),
            (
                raw(Approximation::Add(Computable::one(), Computable::one())),
                8,
                0,
            ),
        ];
        for (value, lane, mask) in rows {
            let root_owners = Arc::strong_count(&root.internal);
            let value_owners = Arc::strong_count(&value.internal);
            let observed = classify(&value);
            assert_eq!(observed.root, 0);
            assert_eq!((observed.lane, observed.retained_mask), (lane, mask));
            assert_eq!(Arc::strong_count(&root.internal), root_owners);
            assert_eq!(Arc::strong_count(&value.internal), value_owners);
            assert!(value.internal.cache.pi_laurent_polynomial().is_none());
            assert!(value.cached().is_none());
            if lane != 0 {
                assert!(value.exact_quadratic_surd_parts().is_some());
            }
        }
        assert_eq!(classify(&root).lane, 1);
        let weak = Arc::downgrade(&root.internal);
        drop(root);
        assert!(weak.upgrade().is_none());
    }
}
