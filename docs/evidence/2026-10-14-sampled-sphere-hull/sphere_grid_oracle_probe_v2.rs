//! Additional independently proven coplanarity witnesses on original sphere coordinates.
use csgrs::{Real, solid};
use hypermesh::{
    MeshCertainty, MeshContext, PredicatePolicy, convex_hull_with_coplanar_groups,
    convex_hull_with_retained_facts,
};
use std::collections::BTreeMap;

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

// A signed product of canonical sin(pi * rational) factors, with the common
// radius omitted. This proves equality only; it never replaces coordinates.
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct Product(i8, Vec<(usize, usize)>);

fn gcd(mut a: usize, mut b: usize) -> usize {
    while b != 0 {
        (a, b) = (b, a % b);
    }
    a
}

fn sine(mut numerator: usize, denominator: usize) -> Product {
    numerator %= 2 * denominator;
    let mut sign = 1;
    if numerator >= denominator {
        sign = -1;
        numerator -= denominator;
    }
    numerator = numerator.min(denominator - numerator);
    if numerator == 0 {
        return Product(0, Vec::new());
    }
    if 2 * numerator == denominator {
        return Product(sign, Vec::new());
    }
    let divisor = gcd(numerator, denominator);
    Product(sign, vec![(numerator / divisor, denominator / divisor)])
}

fn product(mut left: Product, right: Product) -> Product {
    if left.0 == 0 || right.0 == 0 {
        return Product(0, Vec::new());
    }
    left.0 *= right.0;
    left.1.extend(right.1);
    left.1.sort_unstable();
    left
}

fn coordinate_ids(segments: usize, stacks: usize) -> Vec<[u64; 5]> {
    let mut table = BTreeMap::<Product, u64>::new();
    let mut ids = Vec::new();
    let mut append = |coordinates: [Product; 3]| {
        let coordinates = coordinates.map(|value| {
            let next = table.len() as u64;
            *table.entry(value).or_insert(next)
        });
        // Distinct surface IDs disable the unrelated ruled-surface shortcut.
        ids.push([
            coordinates[0],
            coordinates[1],
            coordinates[2],
            ids.len() as u64,
            0,
        ]);
    };
    append([Product(0, vec![]), Product(1, vec![]), Product(0, vec![])]);
    for longitude in 0..segments {
        let longitude_sin = sine(2 * longitude, segments);
        let longitude_cos = sine(4 * longitude + segments, 2 * segments);
        for latitude in 1..stacks {
            let radial = sine(latitude, stacks);
            append([
                product(longitude_cos.clone(), radial.clone()),
                sine(2 * latitude + stacks, 2 * stacks),
                product(longitude_sin.clone(), radial),
            ]);
        }
    }
    append([Product(0, vec![]), Product(-1, vec![]), Product(0, vec![])]);
    ids
}

fn groups(segments: usize, stacks: usize, full: bool) -> Vec<Vec<usize>> {
    let slot = |longitude: usize, latitude: usize| 1 + longitude * (stacks - 1) + latitude - 1;
    let mut groups = Vec::new();
    if !full {
        for longitude in 0..segments {
            let next = (longitude + 1) % segments;
            for latitude in 1..stacks - 1 {
                groups.push(vec![
                    slot(longitude, latitude),
                    slot(next, latitude),
                    slot(longitude, latitude + 1),
                    slot(next, latitude + 1),
                ]);
            }
        }
        return groups;
    }
    for first in 0..segments {
        for second in first + 1..segments {
            for upper in 1..stacks {
                for lower in upper + 1..stacks {
                    groups.push(vec![
                        slot(first, upper),
                        slot(second, upper),
                        slot(first, lower),
                        slot(second, lower),
                    ]);
                }
            }
        }
    }
    for latitude in 1..stacks {
        groups.push(
            (0..segments)
                .map(|longitude| slot(longitude, latitude))
                .collect(),
        );
    }
    let even = segments.is_multiple_of(2);
    let meridians = if even { segments / 2 } else { segments };
    for longitude in 0..meridians {
        let mut group = vec![0, segments * (stacks - 1) + 1];
        group.extend((1..stacks).map(|latitude| slot(longitude, latitude)));
        if even {
            group.extend((1..stacks).map(|latitude| slot(longitude + meridians, latitude)));
        }
        groups.push(group);
    }
    groups
}

fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let case = &args[1];
    let mode = &args[2];
    let (radius, segments, stacks) = match case.as_str() {
        "fifteen_degrees" => (8_u8, 24, 12),
        "original_medium" => (10, 32, 16),
        "fifth_turn" => (8, 10, 7),
        "seventh_turn" => (8, 7, 5),
        _ => panic!("unknown frozen case"),
    };
    assert!(["adjacent", "all_rectangles", "coordinate_symmetries"].contains(&mode.as_str()));
    solid::cache::clear();
    let mesh = solid::sphere(Real::from(radius), segments, stacks);
    assert_eq!(mesh.positions.len(), segments * (stacks - 1) + 2);
    assert_eq!(mesh.triangles.len(), 2 * mesh.positions.len() - 4);
    assert!(mesh.is_closed_manifold());
    let mut groups = groups(segments, stacks, mode != "adjacent");
    if let Some(path) = args.get(3) {
        let text = std::fs::read_to_string(path).unwrap();
        for line in text.lines() {
            let group = line
                .split_whitespace()
                .map(|n| n.parse::<usize>().unwrap())
                .collect::<Vec<_>>();
            assert!(group.len() >= 4);
            assert!(group.iter().all(|&index| index < mesh.positions.len()));
            groups.push(group);
        }
    }
    let ids = coordinate_ids(segments, stacks);
    assert_eq!(ids.len(), mesh.positions.len());
    let reference = if case == "fifteen_degrees" {
        Some(mesh.convex_hull(&STRICT).unwrap())
    } else {
        None
    };
    println!(
        "input case={case} mode={mode} positions={} triangles={} groups={}",
        mesh.positions.len(),
        mesh.triangles.len(),
        groups.len()
    );
    let result = if mode == "coordinate_symmetries" {
        convex_hull_with_retained_facts(&STRICT, &mesh.positions, &groups, &ids)
    } else {
        convex_hull_with_coplanar_groups(&STRICT, &mesh.positions, &groups)
    };
    match result {
        Ok(outcome) => {
            assert_eq!(outcome.certainty, MeshCertainty::Certified);
            let hull = &outcome.value;
            assert_eq!(hull.positions.len(), mesh.positions.len());
            assert_eq!(hull.triangles.len(), 2 * hull.positions.len() - 4);
            assert!(hull.is_closed_manifold());
            assert!(mesh.positions.iter().all(|p| hull.positions.contains(p)));
            assert!(hull.positions.iter().all(|p| mesh.positions.contains(p)));
            if let Some(reference) = &reference {
                assert_eq!(&outcome, reference);
            }
            println!(
                "result case={case} mode={mode} certified=true original_positions_preserved=true prior_certified_rows_check={}",
                if reference.is_some() {
                    "passed"
                } else {
                    "not_applicable"
                }
            );
        },
        Err(error) => {
            println!("result case={case} mode={mode} certified=false error={error:?}")
        },
    }
}
