use super::rectangular_cover::exactly_covers;
use crate::{BuildError, GeometryError};
use hyperreal::Problem;

fn cells(width: usize, height: usize, rectangles: &[[usize; 4]]) -> bool {
    (0..width).all(|x| {
        (0..height).all(|y| {
            rectangles
                .iter()
                .filter(|&&[a, b, c, d]| a <= x && x < b && c <= y && y < d)
                .count()
                == 1
        })
    })
}

#[test]
fn sweep_matches_independent_cell_counts_for_every_two_by_two_subset() {
    let mut choices = Vec::new();
    for a in 0..2 {
        for b in a + 1..=2 {
            for c in 0..2 {
                for d in c + 1..=2 {
                    choices.push([a, b, c, d]);
                }
            }
        }
    }
    for bits in 0..1usize << choices.len() {
        let rectangles = choices
            .iter()
            .enumerate()
            .filter_map(|(i, r)| (bits & (1 << i) != 0).then_some(*r))
            .collect::<Vec<_>>();
        assert_eq!(
            exactly_covers(2, 2, &rectangles).unwrap(),
            cells(2, 2, &rectangles),
            "{rectangles:?}"
        );
    }
    assert!(!exactly_covers(2, 2, &[[0, 2, 0, 2], [0, 2, 0, 2]]).unwrap());
    assert!(!exactly_covers(2, 2, &[[0, 2, 0, 1], [0, 2, 0, 1]]).unwrap()); // equal total area, overlap and gap
}

#[test]
fn event_order_and_invalid_ranges_never_hide_gaps_or_overlap() {
    let mut seed = 0x51d41cc50u64;
    let mut random = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed as usize
    };
    for _ in 0..2000 {
        let mut rectangles = Vec::new();
        for _ in 0..random() % 12 {
            let a = random() % 3;
            let b = a + 1 + random() % (3 - a);
            let c = random() % 3;
            let d = c + 1 + random() % (3 - c);
            rectangles.push([a, b, c, d]);
        }
        let expected = cells(3, 3, &rectangles);
        assert_eq!(exactly_covers(3, 3, &rectangles).unwrap(), expected);
        rectangles.reverse();
        assert_eq!(exactly_covers(3, 3, &rectangles).unwrap(), expected);
    }
    for rect in [
        [0, 0, 0, 1],
        [1, 0, 0, 1],
        [0, 3, 0, 1],
        [0, 1, 1, 0],
        [0, 1, 0, 3],
    ] {
        assert!(!exactly_covers(2, 2, &[rect]).unwrap());
    }
    assert!(matches!(
        exactly_covers(usize::MAX, 1, &[[0, usize::MAX, 0, 1]]),
        Err(BuildError::Geometry(GeometryError::Numeric(
            Problem::OutOfRange
        )))
    ));
    assert!(exactly_covers(1, usize::MAX, &[[0, 1, 0, usize::MAX]]).unwrap());
}

#[test]
fn a_hundred_million_implicit_cells_need_only_linear_storage() {
    let n = 10_000;
    let mut rectangles = Vec::new();
    for i in 0..n {
        rectangles.push([i, n, i, i + 1]);
        if i + 1 < n {
            rectangles.push([i, i + 1, i + 1, n]);
        }
    }
    assert!(exactly_covers(n, n, &rectangles).unwrap());
    rectangles.pop();
    assert!(!exactly_covers(n, n, &rectangles).unwrap());
}
