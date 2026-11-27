//! Shared fixed-point wire and action oracle for deterministic tests and fuzzing.
//!
//! Small rectangular shapes include empty and zero-dimensional unit products and
//! dimensions above Serde's traditional small-array implementations. Expected wire
//! status comes from deliberate edits, while endpoint calculations use i64 integer
//! arithmetic independently of the interval kernels. Quarter-grid product rounding
//! and prefix overflow are part of the expected operation sequence.
//!
//! The second action consumes a replayed Cartesian enclosure: correlations lost
//! by the first action are not restored. Checks assert and panic on disagreement;
//! this allocating test harness is neither a production validator nor a resource
//! budget. Its byte decoder bounds shape size, not the caller's input allocation.

use hyperlattice_interval::hyperinterval::{FixedInterval, IntervalError};
use hyperlattice_interval::{IntervalLatticeError, IntervalMatrix, IntervalVector};
use serde_json::{Value, json};

/// Closed real intervals with signed i32 endpoints measured in quarter units.
type Scalar = FixedInterval<i32, 2>;
/// Ordered lower/upper raw numerators; `[a,b]` denotes `[a/4,b/4]`.
type Bounds = [i32; 2];

/// Expected decoding outcome derived from the authored wire edit, before replay.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Status {
    /// Valid inhabited product, including a zero-dimensional unit.
    Nonempty,
    /// Valid empty Cartesian set, serialized canonically as null after admission.
    Empty,
    /// Malformed shape or scalar bounds that must fail rather than become empty.
    Invalid,
}

/// Mutate an otherwise valid fixed-length lane array and predict its admission.
/// Mode zero preserves it; mode one inserts null; modes two through four remove
/// a lane, write reversed bounds after null, or write an out-of-i32 endpoint.
/// Other cases append a lane, including edits that need a lane when none exists.
/// In the one-lane mode-three case, reversed bounds overwrite the same null lane.
/// Selection wraps modulo the nonzero length; status never consults the decoder.
fn edit_lanes(lanes: &mut Vec<Value>, mode: u8, index: usize) -> Status {
    match mode {
        0 => return Status::Nonempty,
        1 if !lanes.is_empty() => {
            let i = index % lanes.len();
            lanes[i] = Value::Null;
            return Status::Empty;
        }
        2 if !lanes.is_empty() => {
            lanes.pop();
        }
        3 if !lanes.is_empty() => {
            // A later invalid endpoint must survive an earlier empty lane.
            lanes[0] = Value::Null;
            *lanes.last_mut().unwrap() = json!([1, -1]);
        }
        4 if !lanes.is_empty() => {
            let i = index % lanes.len();
            lanes[i] = json!([0, 2147483648_i64]);
        }
        _ => lanes.push(json!([0, 0])),
    }
    Status::Invalid
}

/// Encode valid rows, then preserve them (zero), insert outer null (six), remove
/// a row (seven), edit a selected row (one through five), or append an extra row.
/// An attempted row removal/edit on no rows appends instead and is invalid.
/// Inner empty edits collapse the entire matrix only after all entries validate.
/// The same selector chooses the row and its lane, modulo their respective sizes.
fn encode_rows(rows: &[Vec<Bounds>], mode: u8, index: usize) -> (Value, Status) {
    let mut wire: Vec<Value> = rows.iter().map(|row| json!(row)).collect();
    match mode {
        0 => (wire.into(), Status::Nonempty),
        6 => (Value::Null, Status::Empty),
        7 if !wire.is_empty() => {
            wire.pop();
            (wire.into(), Status::Invalid)
        }
        1..=5 if !wire.is_empty() => {
            let r = index % wire.len();
            let status = edit_lanes(wire[r].as_array_mut().unwrap(), mode, index);
            (wire.into(), status)
        }
        _ => {
            wire.push(json!([]));
            (wire.into(), Status::Invalid)
        }
    }
}

/// Encode outer null for mode six; otherwise apply the shared lane edit.
/// Empty zero-length vectors remain distinct from inhabited zero-length arrays.
fn encode_vector(lanes: &[Bounds], mode: u8, index: usize) -> (Value, Status) {
    if mode == 6 {
        return (Value::Null, Status::Empty);
    }
    let mut wire: Vec<Value> = lanes.iter().map(|lane| json!(lane)).collect();
    let status = edit_lanes(&mut wire, mode, index);
    (wire.into(), status)
}

/// Predict an ordered dot-product enclosure on the quarter grid using integers.
/// Callers supply equally long, ordered interval slices with independent lanes.
/// Four endpoint products give each term's extrema; divide by four with floor
/// and ceiling before accumulating raw numerators. Reject either an out-of-i32
/// term or prefix sum immediately, even if later terms would cancel it. No terms
/// yields `[0,0]`. Slice zip would silently truncate mismatched lengths, so shape
/// agreement is a caller invariant rather than validation performed here.
///
/// Products of two i32 endpoints fit i64. Every preceding accepted prefix fits
/// i32, so adding one rounded product is also safe in i64. The reference compares
/// overflow outcomes, not the lower/upper endpoint ordering of kernel checks.
fn dot(row: &[Bounds], vector: &[Bounds]) -> Result<Bounds, IntervalLatticeError> {
    let mut bounds = [0_i64; 2];
    for (a, b) in row.iter().zip(vector) {
        let corners = a.map(|x| b.map(|y| i64::from(x) * i64::from(y)));
        let endpoints = corners.iter().flatten().copied();
        let lo = endpoints.clone().min().unwrap();
        let hi = endpoints.max().unwrap();
        let term = [lo.div_euclid(4), -(-hi).div_euclid(4)];
        for (sum, x) in bounds.iter_mut().zip(term) {
            *sum += x;
            if i32::try_from(x).is_err() || i32::try_from(*sum).is_err() {
                return Err(IntervalError::Overflow.into());
            }
        }
    }
    Ok(bounds.map(|v| i32::try_from(v).unwrap()))
}

/// Apply the independent dot oracle row by row, stopping at the first overflow.
/// None denotes an empty operand/image; Some with no rows denotes a unit output.
/// For inhabited inputs, callers provide a common row/vector length. Allocation
/// and partial-result cleanup use ordinary Vec/Result collection semantics.
fn product(
    rows: &[Vec<Bounds>],
    lanes: Option<&[Bounds]>,
) -> Result<Option<Vec<Bounds>>, IntervalLatticeError> {
    lanes
        .map(|v| rows.iter().map(|row| dot(row, v)).collect())
        .transpose()
}

/// Select the first invalid index, usize::MAX, or a reduced small index.
/// Residues zero/one modulo four choose the two invalid cases; the others use
/// byte/4 modulo max(dimension,1). Dimension zero therefore never yields a valid
/// index, and dimensions larger than 64 are not exhaustively addressed.
fn index(byte: u8, dimension: usize) -> usize {
    match byte % 4 {
        0 => dimension,
        1 => usize::MAX,
        _ => usize::from(byte / 4) % dimension.max(1),
    }
}

/// Decode a nonempty byte slice into one fixed rectangular qualification case.
/// All byte reads cycle through the payload. The top tag bits select signed-byte
/// endpoints, signed bytes with 127/128 mapped to i32 extrema, or little-endian
/// i32 endpoints (tag mode two); mode three uses ordinary signed bytes. Endpoint
/// pairs are sorted before wire edits. Matrix/vector edit bytes below 128 reduce
/// modulo nine/seven; bytes at least 128 leave those carriers unedited.
///
/// Decode both carriers and check predicted admission and canonical JSON. Invalid
/// matrices stop further checks after vector replay; invalid vectors still allow
/// matrix transpose/projection checks. Invalid indices fail even on empty matrices.
/// For valid operands, compare the product with independent per-term rounding
/// and overflow expectations, serialize/reload a successful result, and consume
/// it in the transposed matrix action. The second action treats its input lanes
/// as independent, so it does not assert an exact correlated A-transpose*A image.
/// Caller-enforced nonempty input prevents a modulo-zero panic in byte lookup.
fn shape<const R: usize, const C: usize>(data: &[u8]) {
    let byte = |i: usize| data[i % data.len()];
    let endpoint = |i: usize| {
        let value = byte(4 + i);
        match byte(0) >> 6 {
            1 if value == 127 => i32::MAX,
            1 if value == 128 => i32::MIN,
            2 => i32::from_le_bytes(core::array::from_fn(|j| byte(4 + 4 * i + j))),
            _ => i32::from(value as i8),
        }
    };
    let lane = |i: usize| {
        let a = endpoint(2 * i);
        let b = endpoint(2 * i + 1);
        [a.min(b), a.max(b)]
    };
    let rows: Vec<Vec<_>> = (0..R)
        .map(|r| (0..C).map(|c| lane(r * C + c)).collect())
        .collect();
    let lanes: Vec<_> = (0..C).map(|c| lane(R * C + c)).collect();
    let selector = usize::from(byte(3));
    let matrix_mode = if byte(1) < 128 { byte(1) % 9 } else { 0 };
    let (wire, matrix_status) = encode_rows(&rows, matrix_mode, selector);
    let matrix =
        serde_json::from_slice::<IntervalMatrix<Scalar, R, C>>(&serde_json::to_vec(&wire).unwrap());
    let vector_mode = if byte(2) < 128 { byte(2) % 7 } else { 0 };
    let (wire, vector_status) = encode_vector(&lanes, vector_mode, selector);
    let vector =
        serde_json::from_slice::<IntervalVector<Scalar, C>>(&serde_json::to_vec(&wire).unwrap());
    assert_eq!(matrix.is_err(), matrix_status == Status::Invalid);
    assert_eq!(vector.is_err(), vector_status == Status::Invalid);

    if let Ok(v) = &vector {
        assert_eq!(
            serde_json::to_value(v).unwrap(),
            if vector_status == Status::Empty {
                Value::Null
            } else {
                json!(lanes)
            }
        );
    }
    let Ok(matrix) = matrix else { return };
    let empty = matrix_status == Status::Empty;
    let canonical = |value: Value| if empty { Value::Null } else { value };
    assert_eq!(
        serde_json::to_value(matrix).unwrap(),
        canonical(json!(rows))
    );
    let columns: Vec<Vec<_>> = (0..C)
        .map(|c| rows.iter().map(|r| r[c]).collect())
        .collect();
    let transpose = matrix.transpose(&mut ()).unwrap();
    assert_eq!(
        serde_json::to_value(transpose).unwrap(),
        canonical(json!(columns))
    );

    let r = index(byte(3), R);
    let c = index(byte(4), C);
    let row = matrix.row(r, &mut ());
    let column = matrix.column(c, &mut ());
    if r < R {
        assert_eq!(
            serde_json::to_value(row.unwrap()).unwrap(),
            canonical(json!(rows[r]))
        );
    } else {
        assert_eq!(
            row.unwrap_err(),
            IntervalLatticeError::IndexOutOfBounds {
                index: r,
                dimension: R
            }
        );
    }
    if c < C {
        assert_eq!(
            serde_json::to_value(column.unwrap()).unwrap(),
            canonical(json!(columns[c]))
        );
    } else {
        assert_eq!(
            column.unwrap_err(),
            IntervalLatticeError::IndexOutOfBounds {
                index: c,
                dimension: C
            }
        );
    }

    if let Ok(vector) = vector {
        let initial = (!empty && vector_status == Status::Nonempty).then_some(lanes.as_slice());
        let image = matrix.mul_vector(&vector, &mut ());
        match product(&rows, initial) {
            Err(error) => assert_eq!(image.unwrap_err(), error),
            Ok(bounds) => {
                let image = image.unwrap();
                assert_eq!(json!(image), json!(bounds));
                let replay: IntervalVector<Scalar, R> =
                    serde_json::from_slice(&serde_json::to_vec(&image).unwrap()).unwrap();
                assert_eq!(image, replay);
                let second = transpose.mul_vector(&replay, &mut ());
                assert_eq!(
                    second.map(|v| json!(v)),
                    product(&columns, bounds.as_deref()).map(|v| json!(v))
                );
            }
        }
    }
}

/// Check one arbitrary byte string; empty input is a no-op.
/// The low six tag bits modulo seven select 0x0, 0x3, 3x0, 2x3, 3x2, 1x33 or
/// 33x1. The remaining tag bits select endpoint encoding inside shape. This
/// bounds generated dimensions independently of payload length. All malformed
/// cases must match the predicted failure; assertion panics signal regressions.
pub fn check(data: &[u8]) {
    let Some(&tag) = data.first() else { return };
    match (tag & 63) % 7 {
        0 => shape::<0, 0>(data),
        1 => shape::<0, 3>(data),
        2 => shape::<3, 0>(data),
        3 => shape::<2, 3>(data),
        4 => shape::<3, 2>(data),
        5 => shape::<1, 33>(data),
        _ => shape::<33, 1>(data),
    }
}
