    #[test]
    fn duplicate_layer_islands_certify_cparti_rounded_pad_pair() {
        // CPArti J1 A5/A6 after the existing finite placement boundary. Their
        // native intersection retains circular algebraic fragments whose area
        // integral was unavailable when this regression was introduced.
        let pad = |center| {
            crate::geometry::rounded_rect_polygon(center, [0.3, 1.15], 0.075, 180.0, 8)
        };
        let first = pad([141.995001, 98.5]);
        let second = pad([141.995, 97.5]);
        let duplicate = pad([141.995, 100.45]);
        for polygons in [
            vec![first.clone(), second.clone()],
            vec![second.clone(), first.clone()],
        ] {
            assert!(
                duplicate_layer_island_readiness(
                    "F.Cu",
                    &region("F.Cu", polygons),
                    &crate::scalar::scalar("1e-9"),
                )
                .is_empty()
            );
        }
        let violations = duplicate_layer_island_readiness(
            "F.Cu",
            &region("F.Cu", vec![first, second, duplicate.clone(), duplicate]),
            &crate::scalar::scalar("1e-9"),
        );
        assert_eq!(violations.len(), 1);
        assert_eq!(violations[0].check, "layer-sanity");
        assert_eq!(violations[0].locations, vec![[141.995, 100.45]]);
    }

    #[test]
    fn duplicate_polygon_overlap_bound_preserves_tolerance_and_touching() {
        let left = rect_polygon([0.0, 0.0], [1.0, 1.0], 0.0);
        // Both sides of the existing 1e-9 area tolerance; no geometric
        // tolerance is introduced by the bounding-box proof.
        for (shift, expected) in [(0.0, true), (2f64.powi(-30), true), (2f64.powi(-29), false), (1.0, false)] {
            let right = rect_polygon([shift, 0.0], [1.0, 1.0], 0.0);
            for (a, b) in [(&left, &right), (&right, &left)] {
                assert_eq!(
                    super::polygons_are_duplicate(
                        a,
                        b,
                        &crate::scalar::scalar("1e-6"),
                        "test",
                        vec!["F.Cu".to_owned()],
                    ).unwrap(),
                    expected,
                );
            }
        }
    }
