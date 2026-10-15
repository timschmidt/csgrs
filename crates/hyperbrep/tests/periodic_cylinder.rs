//! The solid proof must depend on the covered domain, not the author's patch count.
#[path = "support/periodic_cylinder.rs"]
mod fixture;
use hyperbrep::*;

#[test]
fn full_period_seam_and_unequal_side_partitions_survive_replay() {
    for rectangles in [
        vec![[0, 4, 0, 1]],
        vec![[0, 2, 0, 1], [2, 4, 0, 1]],
        vec![[0, 1, 0, 1], [1, 4, 0, 1]],
        vec![[0, 1, 0, 1], [1, 2, 0, 1], [2, 4, 0, 1]],
        vec![[0, 1, 0, 1], [1, 2, 0, 1], [2, 3, 0, 1], [3, 4, 0, 1]],
    ] {
        for turns in [0, -1, 2] {
            let model = fixture::build(&rectangles, 1, turns).unwrap();
            assert_eq!(model.counts().faces, rectangles.len() + 2);
            assert_eq!(model.counts().edges, 8 + rectangles.len());
            for model in [&model, &fixture::replay(&model)] {
                fixture::check(model);
                fixture::check(model); // warm proof caches
            }
        }
    }
}

#[test]
fn axial_t_junctions_and_repeated_seams_cover_exactly_once() {
    for rectangles in [
        vec![[0, 4, 0, 1], [0, 4, 1, 2]],
        vec![[0, 1, 0, 2], [1, 4, 0, 1], [1, 4, 1, 2]],
        vec![[0, 2, 0, 1], [2, 4, 0, 2], [0, 1, 1, 2], [1, 2, 1, 2]],
    ] {
        let model = fixture::build(&rectangles, 2, 0).unwrap();
        fixture::check(&model);
        fixture::check(&fixture::replay(&model));
    }
}

#[test]
fn seam_subdivision_reflection_and_boolean_reuse_the_certificate() {
    let model = fixture::build(&[[0, 4, 0, 1]], 1, 0).unwrap();
    let seam = model
        .edges()
        .find(|e| e.curve().kind() == Curve3Kind::Line)
        .unwrap();
    assert_eq!(model.edge_uses().filter(|u| u.edge() == seam).count(), 2);
    let split = seam
        .split((Real::one() / Real::from(2)).unwrap().into())
        .unwrap()
        .into_model();
    assert_eq!(split.counts().edges, 10);
    fixture::check(&fixture::replay(&split));
    let reflected = split
        .transformed(&Matrix4::affine_nonuniform_scale([
            -Real::one(),
            Real::one(),
            Real::one(),
        ]))
        .unwrap();
    fixture::check(&fixture::replay(&reflected));
    let canonical = builder::cylinder(Real::from(2), Real::from(4)).unwrap();
    let a = model.solids().next().unwrap();
    let b = canonical.bodies().next().unwrap();
    fixture::check(boolean::union(a, b).unwrap().model());
    fixture::check(boolean::intersection(a, b).unwrap().model());
    assert_eq!(boolean::difference(a, b).unwrap().bodies().count(), 0);
}

#[test]
fn invalid_side_coverage_and_archived_orientation_are_rejected() {
    for rectangles in [
        vec![[0, 3, 0, 1]],               // missing quarter
        vec![[0, 4, 0, 1], [0, 4, 0, 1]], // double cover
        vec![[0, 2, 0, 1], [1, 3, 0, 1]], // same area, overlap and gap
    ] {
        assert!(fixture::build(&rectangles, 1, 0).is_err());
    }
    let model = fixture::build(&[[0, 4, 0, 1]], 1, 0).unwrap();
    for face in [0, 1, 2] {
        let mut raw: serde_json::Value = serde_json::from_str(&model.to_json().unwrap()).unwrap();
        let old = raw["faces"][face]["orientation"].as_str().unwrap();
        raw["faces"][face]["orientation"] = serde_json::json!(if old == "Forward" {
            "Reversed"
        } else {
            "Forward"
        });
        assert!(
            RawModel::from_json(&raw.to_string(), &ArchiveLimits::default())
                .unwrap()
                .validate()
                .is_err()
        );
    }
}
