//! Shared native/WASM semantic cases. Rectangle expectations use independent
//! integer-cell arithmetic, not another wrapper of the Boolean kernel.

use hyperbrep::*;

type Rect = [i32; 4];

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn half(n: i32) -> Real {
    (Real::from(n) / Real::from(2)).unwrap()
}
fn cuboid(x0: i32, x1: i32) -> BodySet {
    builder::cuboid(point(x0, 0, 0), point(x1, 2, 2)).unwrap()
}
fn rects(rects: &[Rect]) -> BodySet {
    builder::extrude_regions(
        &rects
            .iter()
            .map(|&[x0, y0, x1, y1]| {
                (
                    [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                        .map(|(x, y)| Point2::new(x.into(), y.into()))
                        .to_vec(),
                    Vec::new(),
                )
            })
            .collect::<Vec<_>>(),
        0.into(),
        2.into(),
    )
    .unwrap()
}
fn inside(set: &BodySet, point: &Point3) -> bool {
    set.bodies()
        .any(|s| s.classify_point(point).unwrap() == SolidPointLocation::Inside)
}
fn volume(set: &BodySet) -> Real {
    set.bodies()
        .fold(Real::zero(), |v, s| v + s.volume().unwrap())
}
fn replay(set: &BodySet) -> Model {
    let json = set.model().to_json().unwrap();
    let model = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), json);
    assert_eq!(model.counts(), set.model().counts());
    model
}
fn same_cells(actual: &BodySet, first: &[Rect], second: &[Rect], operation: BooleanOperation) {
    let contains = |rects: &[Rect], x: i32, y: i32| {
        rects
            .iter()
            .any(|&[x0, y0, x1, y1]| x0 <= x && x < x1 && y0 <= y && y < y1)
    };
    let mut expected_cells = 0;
    for x in -2..10 {
        for y in -2..7 {
            let a = contains(first, x, y);
            let b = contains(second, x, y);
            let expected = match operation {
                BooleanOperation::Union => a || b,
                BooleanOperation::Intersection => a && b,
                BooleanOperation::Difference => a && !b,
            };
            assert_eq!(
                inside(
                    actual,
                    &Point3::new(half(2 * x + 1), half(2 * y + 1), Real::one())
                ),
                expected,
                "cell ({x},{y}), {operation:?}"
            );
            expected_cells += i32::from(expected);
        }
    }
    assert_eq!(volume(actual), Real::from(expected_cells * 2));
    assert!(!inside(actual, &point(0, 0, 3)));
    replay(actual);
}

pub fn empty_identities_continue() {
    let a = cuboid(0, 4);
    let empty = BodySet::empty();
    assert!(empty.is_empty());
    assert_eq!(empty.model().counts().solids, 0);
    assert_eq!(empty.model().bounds().unwrap(), None);
    let removed = boolean::difference(&a, &a).unwrap();
    assert_eq!(
        removed.model().to_json().unwrap(),
        empty.model().to_json().unwrap()
    );
    for empty in [empty, removed] {
        for result in [
            boolean::union(&a, &empty).unwrap(),
            boolean::union(&empty, &a).unwrap(),
            boolean::difference(&a, &empty).unwrap(),
        ] {
            assert!(result.model().same_revision(a.model()));
            assert_eq!(volume(&result), Real::from(16));
            let cut = boolean::difference(&result, &cuboid(2, 4)).unwrap();
            assert_eq!(volume(&cut), Real::from(8));
            assert!(inside(&cut, &point(1, 1, 1)));
            assert!(!inside(&cut, &point(3, 1, 1)));
        }
        for result in [
            boolean::intersection(&a, &empty).unwrap(),
            boolean::intersection(&empty, &a).unwrap(),
            boolean::difference(&empty, &a).unwrap(),
            boolean::union(&empty, &empty).unwrap(),
            boolean::difference(&empty, &empty).unwrap(),
        ] {
            assert!(result.is_empty());
            replay(&result);
            let transformed = result
                .transformed(&Matrix4::affine_translation([
                    Real::pi(),
                    0.into(),
                    0.into(),
                ]))
                .unwrap();
            assert!(transformed.is_empty());
            assert_eq!(
                volume(&boolean::union(&transformed, &a).unwrap()),
                Real::from(16)
            );
        }
    }
}

pub fn disconnected_accumulation_removal_and_bridge() {
    let operands = [cuboid(0, 2), cuboid(4, 6), cuboid(8, 9)];
    let mut accumulated = BodySet::empty();
    for operand in &operands {
        accumulated = boolean::union(&accumulated, operand).unwrap();
    }
    assert_eq!(accumulated.len(), 3);
    assert_eq!(volume(&accumulated), Real::from(20));
    replay(&accumulated);
    for operand in &operands {
        accumulated = boolean::difference(&accumulated, operand).unwrap();
    }
    assert!(accumulated.is_empty());
    assert_eq!(
        volume(&boolean::union(&accumulated, &operands[0]).unwrap()),
        Real::from(8)
    );

    // The new operand bridges both separate components. Normalization must
    // not lose material, duplicate a body, or stop after the first merge.
    let disjoint = boolean::union(&cuboid(0, 2), &cuboid(4, 6)).unwrap();
    let bridge = cuboid(1, 5);
    let joined = boolean::union(&disjoint, &bridge).unwrap();
    assert_eq!(joined.len(), 1);
    assert_eq!(volume(&joined), Real::from(24));
    for x in 0..6 {
        assert!(inside(
            &joined,
            &Point3::new(half(2 * x + 1), Real::one(), Real::one())
        ));
    }
    let reverse = boolean::union(&bridge, &disjoint).unwrap();
    assert_eq!(volume(&reverse), Real::from(24));
    let split = boolean::difference(&joined, &cuboid(2, 4)).unwrap();
    assert_eq!(split.len(), 2);
    assert_eq!(volume(&split), Real::from(16));
    assert!(!inside(&split, &point(3, 1, 1)));
    let overlap = boolean::intersection(&joined, &cuboid(2, 4)).unwrap();
    let restored = boolean::union(&split, &overlap).unwrap();
    assert_eq!(restored.len(), 1);
    assert_eq!(volume(&restored), Real::from(24));
    replay(&restored);
}

pub fn overlapping_selections_follow_union_semantics() {
    let cases = [
        (
            vec![[0, 0, 4, 4], [2, 0, 6, 4]],
            vec![[1, 1, 3, 3], [2, 1, 5, 3]],
        ),
        (vec![[0, 0, 2, 4], [4, 0, 6, 4]], vec![[1, 0, 5, 4]]),
        (vec![[0, 0, 6, 4]], vec![[1, 0, 2, 4], [4, 0, 5, 4]]),
        (
            vec![[0, 0, 4, 4], [0, 0, 4, 4]],
            vec![[1, 1, 3, 3], [1, 1, 3, 3]],
        ),
        (
            vec![[0, 0, 2, 2], [4, 0, 6, 2]],
            vec![[0, 0, 2, 2], [4, 0, 6, 2]],
        ),
        (
            vec![[0, 0, 1, 1], [4, 4, 5, 5]],
            vec![[6, 0, 7, 1], [8, 4, 9, 5]],
        ),
    ];
    for (a, b) in cases {
        let first = rects(&a);
        let second = rects(&b);
        for operation in [
            BooleanOperation::Union,
            BooleanOperation::Intersection,
            BooleanOperation::Difference,
        ] {
            let result = boolean::boolean(&first, &second, operation).unwrap();
            same_cells(&result, &a, &b, operation);
            // Consume every result, including complete subtraction and duplicate
            // selections. Exact self-difference must leave no hidden component.
            assert!(boolean::difference(&result, &result).unwrap().is_empty());
        }
        let xor = boolean::symmetric_difference(&first, &second).unwrap();
        let contains = |rs: &[Rect], x: i32, y: i32| {
            rs.iter()
                .any(|&[x0, y0, x1, y1]| x0 <= x && x < x1 && y0 <= y && y < y1)
        };
        let mut cells = 0;
        for x in -2..10 {
            for y in -2..7 {
                let expected = contains(&a, x, y) ^ contains(&b, x, y);
                assert_eq!(
                    inside(
                        &xor,
                        &Point3::new(half(2 * x + 1), half(2 * y + 1), Real::one())
                    ),
                    expected
                );
                cells += i32::from(expected);
            }
        }
        assert_eq!(volume(&xor), Real::from(cells * 2));
        replay(&xor);
        assert!(
            boolean::symmetric_difference(&xor, &xor)
                .unwrap()
                .is_empty()
        );
    }
}

pub fn unselected_bodies_never_reappear() {
    let original = rects(&[[0, 0, 2, 2], [4, 0, 6, 2]]);
    let selected = BodySet::from_solid(original.bodies().nth(1).unwrap());
    let other = BodySet::from_solid(original.bodies().next().unwrap());
    assert!(boolean::intersection(&selected, &other).unwrap().is_empty());
    assert_eq!(
        volume(&boolean::difference(&selected, &other).unwrap()),
        Real::from(8)
    );
    assert!(selected.model().same_revision(original.model()));
    let joined = boolean::union(&selected, &cuboid(8, 9)).unwrap();
    assert_eq!(joined.len(), 2);
    assert_eq!(joined.model().counts().solids, 2);
    assert!(!inside(&joined, &point(1, 1, 1)));
    assert!(inside(&joined, &point(5, 1, 1)));
    assert_eq!(volume(&joined), Real::from(12));
    let loaded = replay(&joined);
    assert_eq!(loaded.solids().count(), 2);
    let cut = boolean::difference(&joined, &selected).unwrap();
    assert_eq!(volume(&cut), Real::from(4));
    assert!(!inside(&cut, &point(5, 1, 1)));
}

pub fn holes_symbolic_coordinates_and_cavities() {
    let outer = cuboid(0, 6);
    let cutter = builder::cuboid(point(2, 0, 0), point(4, 2, 2)).unwrap();
    let split = boolean::difference(&outer, &cutter).unwrap();
    let shift =
        Matrix4::affine_translation([Real::pi(), Real::from(2).sqrt().unwrap(), Real::one()]);
    let shifted = split.transformed(&shift).unwrap();
    let result = boolean::intersection(&shifted, &outer.transformed(&shift).unwrap()).unwrap();
    assert_eq!(result.len(), 2);
    assert_eq!(volume(&result), Real::from(16));
    assert!(inside(
        &result,
        &shift.transform_point3(&point(1, 1, 1)).unwrap()
    ));
    assert!(!inside(
        &result,
        &shift.transform_point3(&point(3, 1, 1)).unwrap()
    ));
    replay(&result);

    let first = builder::cuboid(point(0, 0, 0), point(6, 6, 2)).unwrap();
    let hole = builder::cuboid(point(1, 1, 0), point(5, 5, 2)).unwrap();
    let ring = boolean::difference(&first, &hole).unwrap();
    let extra = builder::cuboid(point(8, 0, 0), point(9, 1, 2)).unwrap();
    let multiple = boolean::union(&ring, &extra).unwrap();
    assert_eq!(volume(&multiple), Real::from(42));
    assert!(!inside(&multiple, &point(3, 3, 1)));
    let restored = boolean::union(&multiple, &hole).unwrap();
    assert_eq!(volume(&restored), Real::from(74));
    assert!(inside(&restored, &point(3, 3, 1)));
    replay(&restored);

    let hollow = builder::sphere_with_voids(
        Real::from(3),
        &[builder::SphereVoid {
            center: Point3::origin(),
            radius: Real::one(),
        }],
    )
    .unwrap();
    let remote = builder::sphere(Real::one())
        .unwrap()
        .transformed(&Matrix4::affine_translation([
            10.into(),
            0.into(),
            0.into(),
        ]))
        .unwrap();
    let combined = boolean::union(&hollow, &remote).unwrap();
    assert_eq!(
        combined.bodies().map(|s| s.voids().count()).sum::<usize>(),
        1
    );
    assert!(!inside(&combined, &Point3::origin()));
    assert_eq!(volume(&combined), Real::from(36) * Real::pi());
    replay(&combined);
}

pub fn selected_parameters_survive_collection() {
    let curve = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| {
                Point3::new(
                    (Real::from(n) / Real::from(5)).unwrap(),
                    Real::zero(),
                    Real::zero(),
                )
            })
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut roots) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("five-degree root")
    };
    let root = roots.remove(0);
    assert!(root.try_real().is_none());
    let body = cuboid(0, 2);
    let split = body.model().edges().next().unwrap().split(root).unwrap();
    let selected = BodySet::from_solid(split.model().solids().next().unwrap());
    let unchanged = boolean::union(&BodySet::empty(), &selected).unwrap();
    assert!(unchanged.model().same_revision(split.model()));
    assert_eq!(
        unchanged.model().to_json().unwrap(),
        split.model().to_json().unwrap()
    );
    let combined = boolean::union(&unchanged, &cuboid(5, 6)).unwrap();
    assert_eq!(
        combined
            .model()
            .vertices()
            .filter(|v| v.point().try_coordinates().is_none())
            .count(),
        1
    );
    assert_eq!(volume(&combined), Real::from(12));
    let loaded = replay(&combined);
    let continued = loaded
        .edges()
        .find(|e| e.domain().end().try_real().is_none())
        .unwrap()
        .split(half(1).into())
        .unwrap();
    assert_eq!(continued.model().solids().count(), 2);
}

pub fn failures_are_atomic_and_empty_does_not_admit_raw_models() {
    let empty = BodySet::empty();
    let original = cuboid(0, 2);
    let mut json: serde_json::Value =
        serde_json::from_str(&original.model().to_json().unwrap()).unwrap();
    // Inspect the current archive representation rather than constructing an
    // invalid BodySet via private fields. Invalid input never reaches identities.
    assert!(json["edges"][0]["start"].is_number());
    json["edges"][0]["start"] = serde_json::json!(u32::MAX);
    let raw = RawModel::from_json(&json.to_string(), &ArchiveLimits::default()).unwrap();
    assert!(raw.validate().is_err());
    assert!(builder::cuboid(Point3::origin(), Point3::origin()).is_err());
    assert!(empty.is_empty());
    let sphere = builder::sphere(Real::one()).unwrap();
    let touching = sphere
        .transformed(&Matrix4::affine_translation([2.into(), 0.into(), 0.into()]))
        .unwrap();
    let remote = sphere
        .transformed(&Matrix4::affine_translation([
            10.into(),
            0.into(),
            0.into(),
        ]))
        .unwrap();
    let accumulated = boolean::union(&remote, &sphere).unwrap();
    let before = accumulated.model().to_json().unwrap();
    assert!(matches!(
        boolean::union(&accumulated, &touching),
        Err(BooleanError::FallbackFailed { .. })
    ));
    assert_eq!(accumulated.model().to_json().unwrap(), before);
    assert_eq!(
        volume(&boolean::union(&empty, &accumulated).unwrap()),
        (Real::from(8) * Real::pi() / Real::from(3)).unwrap()
    );
}

/// Unit cubes touching along an edge or at a vertex have a regularized union
/// whose boundary is not a manifold surface. The union keeps both exact
/// solids, which together represent that set, rather than an empty result, a
/// merged shell or an error; later operations consume it.
pub fn edge_and_point_contacts_keep_both_solids() {
    let unit = |x: i32, y: i32, z: i32| {
        builder::cuboid(point(x, y, z), point(x + 1, y + 1, z + 1)).unwrap()
    };
    let center =
        |x: i32, y: i32, z: i32| Point3::new(half(2 * x + 1), half(2 * y + 1), half(2 * z + 1));
    let a = unit(0, 0, 0);
    for (label, corner) in [("edge", [1, 1, 0]), ("point", [1, 1, 1])] {
        let [x, y, z] = corner;
        let b = unit(x, y, z);
        for union in [
            boolean::union(&a, &b).unwrap(),
            boolean::union(&b, &a).unwrap(),
        ] {
            assert_eq!(union.len(), 2, "{label}");
            assert_eq!(volume(&union), Real::from(2), "{label}");
            assert!(inside(&union, &center(0, 0, 0)), "{label}");
            assert!(inside(&union, &center(x, y, z)), "{label}");
            assert!(!inside(&union, &center(1, 0, 0)), "{label}");
            assert!(!inside(&union, &center(0, 1, z)), "{label}");
            replay(&union);
            let rest = boolean::difference(&union, &a).unwrap();
            assert_eq!(rest.len(), 1, "{label}");
            assert_eq!(volume(&rest), Real::one(), "{label}");
            assert!(inside(&rest, &center(x, y, z)), "{label}");
        }
        assert!(boolean::intersection(&a, &b).unwrap().is_empty(), "{label}");
        assert_eq!(
            volume(&boolean::difference(&a, &b).unwrap()),
            Real::one(),
            "{label}"
        );
    }
}

pub fn exact_gap_and_ambient_policy_survive_composition() {
    let tiny: Real = "1e-12".parse().unwrap();
    let first = builder::cuboid(point(0, 0, 0), point(1, 1, 1)).unwrap();
    let second = builder::cuboid(
        Point3::new(Real::one() + &tiny, Real::zero(), Real::zero()),
        point(2, 1, 1),
    )
    .unwrap();
    let outer = hypercurve::provisional(|| {
        let zero = OperationContext::strict(WorkBudget::ZERO);
        let attempt = zero.evaluate(|| {
            let from_empty = boolean::union(&BodySet::empty(), &first).unwrap();
            boolean::union(&from_empty, &second)
        });
        let union = attempt.result.unwrap().unwrap();
        assert_eq!(union.len(), 2);
        assert_eq!(volume(&union), Real::from(2) - &tiny);
        assert_eq!(attempt.usage, WorkUsage::default());
        let gap = Point3::new(
            Real::one() + (&tiny / Real::from(2)).unwrap(),
            half(1),
            half(1),
        );
        assert!(!inside(&union, &gap));
        assert!(boolean::intersection(&first, &second).unwrap().is_empty());
        let right = boolean::difference(&union, &first).unwrap();
        assert_eq!(volume(&right), Real::one() - &tiny);
        replay(&right);
    });
    assert_eq!(outer.certainty(), hypercurve::CurveCertainty::Certified);
    outer.certified().unwrap();
}

pub fn selection_identity_covers_curved_sweeps() {
    let profile = [(0, 0), (2, 0), (2, 2), (0, 2)].map(|(x, y)| Point2::new(x.into(), y.into()));
    let path = Curve3::rational_bezier(
        [(0, 0, 0), (1, 0, 1), (0, 0, 4)]
            .map(|(x, y, z)| Point3::new(x.into(), y.into(), z.into()))
            .to_vec(),
        vec![Real::one(), 2.into(), 3.into()],
    )
    .unwrap();
    let swept = builder::sweep_curve(&profile, Vector3::x(), Vector3::y(), path).unwrap();
    let clone = swept.clone();
    let identity = OperationContext::strict(WorkBudget::ZERO).evaluate(|| {
        for result in [
            boolean::union(&swept, &clone).unwrap(),
            boolean::intersection(&swept, &clone).unwrap(),
        ] {
            assert!(result.model().same_revision(swept.model()));
            assert_eq!(
                result.model().to_json().unwrap(),
                swept.model().to_json().unwrap()
            );
            assert_eq!(volume(&result), Real::from(16));
            replay(&result);
        }
        for empty in [
            boolean::difference(&swept, &clone).unwrap(),
            boolean::symmetric_difference(&swept, &clone).unwrap(),
        ] {
            assert!(empty.is_empty());
            let restored = boolean::union(&empty, &swept).unwrap();
            assert!(restored.model().same_revision(swept.model()));
        }
    });
    identity.result.unwrap();
    assert_eq!(identity.usage, WorkUsage::default());
}
