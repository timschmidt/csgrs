//! Public BodySet composition of native closed edges, shared with the WASM probe.
use super::fixture;
use hyperbrep::*;

pub fn compose() {
    for reversed in [false, true] {
        let model = fixture::build(Real::zero(), -2, reversed).unwrap();
        let original = model.to_json().unwrap();
        let a = BodySet::from_solid(model.solids().next().unwrap());
        let translated = |x| {
            a.transformed(&Matrix4::affine_translation([
                Real::from(x),
                Real::zero(),
                Real::zero(),
            ]))
            .unwrap()
        };
        let b = translated(10);
        let c = translated(20);
        let pair = boolean::union(&boolean::union(&BodySet::empty(), &a).unwrap(), &b).unwrap();
        assert_eq!(pair.len(), 2);
        let selected = BodySet::from_solid(pair.bodies().next().unwrap());
        // Copy a selected subgraph of a larger model. Its unselected component
        // must not reappear, and closed edges must retain their single vertex.
        let joined = boolean::union(&selected, &c).unwrap();
        assert_eq!(joined.len(), 2);
        for model in [joined.model(), &fixture::replay(joined.model())] {
            let counts = model.counts();
            assert_eq!(
                (
                    counts.vertices,
                    counts.edges,
                    counts.edge_uses,
                    counts.faces
                ),
                (4, 6, 12, 6)
            );
            assert!(model.vertices().all(|v| v.edges().count() == 2));
            assert!(
                model
                    .edges()
                    .filter(|e| e.curve().kind() == Curve3Kind::CircleArc)
                    .all(|e| e.start() == e.end() && e.uses().count() == 2)
            );
            let volume = model
                .solids()
                .map(|s| s.volume().unwrap())
                .fold(Real::zero(), |a, b| a + b);
            assert_eq!(volume, Real::from(32) * Real::pi());
            for (x, expected) in [(0, true), (10, false), (20, true)] {
                let p = Point3::new(x.into(), 0.into(), 2.into());
                assert_eq!(
                    model
                        .solids()
                        .any(|s| s.classify_point(&p).unwrap() == SolidPointLocation::Inside),
                    expected
                );
            }
        }
        let retained = boolean::intersection(&joined, &a).unwrap();
        fixture::check(&fixture::replay(retained.model()));
        let remainder = boolean::difference(&joined, &selected).unwrap();
        assert_eq!(remainder.len(), 1);
        let empty = boolean::difference(&remainder, &c).unwrap();
        assert!(empty.is_empty());
        assert!(fixture::replay(empty.model()).solids().next().is_none());
        let restored = boolean::union(&empty, &a).unwrap();
        fixture::check(&fixture::replay(restored.model()));
        assert_eq!(model.to_json().unwrap(), original);
    }
}
