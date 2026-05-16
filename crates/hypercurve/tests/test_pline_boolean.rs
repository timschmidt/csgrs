mod test_utils;

use cavalier_contours::polyline::{
    BooleanOp, BooleanPlineSlice, BooleanResult, BooleanResultInfo, BooleanResultPline,
    PlineBooleanOptions, PlineCreation, PlineSource, PlineSourceMut, Polyline,
};
use test_utils::{
    ModifiedPlineSet, ModifiedPlineSetVisitor, ModifiedPlineState, PlineProperties,
    create_property_set, property_sets_match, property_sets_match_abs_a,
};

fn create_boolean_property_set(polylines: &[BooleanResultPline<Polyline>]) -> Vec<PlineProperties> {
    for r in polylines {
        assert!(
            r.pline
                .remove_repeat_pos(PlineProperties::POS_EQ_EPS)
                .is_none(),
            "boolean result should not have repeat positioned vertexes"
        );
    }
    create_property_set(polylines.iter().map(|p| &p.pline), false)
}

fn run_same_boolean_test(
    self1: &Polyline<f64>,
    self2: &Polyline<f64>,
    self1_state: &ModifiedPlineState,
    self2_state: &ModifiedPlineState,
    input_properties: &PlineProperties,
) {
    use cavalier_contours::polyline::BooleanOp::*;
    // test same polyline
    for &op in [Or, And].iter() {
        let result = self1.boolean(self2, op);
        assert!(matches!(result.result_info, BooleanResultInfo::Overlapping));
        let mut passed = result.pos_plines.len() == 1 && result.neg_plines.is_empty();
        if passed {
            let result_properties = PlineProperties::from_pline(
                &result.pos_plines[0].pline,
                self2_state.inverted_direction,
            );
            passed =
                property_sets_match(&[result_properties], std::slice::from_ref(input_properties));
        }

        assert!(
            passed,
            "boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    for &op in [Not, Xor].iter() {
        let result = self1.boolean(self2, op);
        assert!(matches!(result.result_info, BooleanResultInfo::Overlapping));
        let passed = result.pos_plines.is_empty() && result.neg_plines.is_empty();
        assert!(
            passed,
            "boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // test same polyline disjoint by translating
    let extents = self1.extents().unwrap();
    let disjoint1 = {
        let mut c = self1.clone();
        c.translate_mut(1.0 + extents.max_x - extents.min_x, 0.0);
        c
    };

    let disjoint1_properties = PlineProperties::from_pline(&disjoint1, false);

    // disjoint OR
    {
        let op = Or;
        let expected = &[disjoint1_properties.clone(), input_properties.clone()];
        let result = disjoint1.boolean(self2, op);
        assert!(matches!(result.result_info, BooleanResultInfo::Disjoint));
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // disjoint AND
    {
        let op = And;
        let result = disjoint1.boolean(self2, op);
        assert!(matches!(result.result_info, BooleanResultInfo::Disjoint));
        let passed = result.pos_plines.is_empty() && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // disjoint NOT
    {
        let op = Not;
        let expected = std::slice::from_ref(&disjoint1_properties);
        let result = disjoint1.boolean(self2, op);
        assert!(matches!(result.result_info, BooleanResultInfo::Disjoint));
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // disjoint XOR
    {
        let op = Xor;
        let expected = &[disjoint1_properties.clone(), input_properties.clone()];
        let result = disjoint1.boolean(self2, op);
        assert!(matches!(result.result_info, BooleanResultInfo::Disjoint));
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // test same polyline but offset one of them to be fully enclosed by the other
    let offset = if self1.area() < 0.0 { -0.2 } else { 0.2 };
    let self1_inward_offset = self1.parallel_offset(offset).remove(0);

    let offset_properties = &[PlineProperties::from_pline(&self1_inward_offset, false)];

    // enclosed OR
    {
        let op = Or;
        let expected = std::slice::from_ref(input_properties);
        let result = self2.boolean(&self1_inward_offset, op);
        assert!(matches!(
            result.result_info,
            BooleanResultInfo::Pline2InsidePline1
        ));
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "enclosed test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // enclosed AND
    {
        let op = And;
        let expected = offset_properties;
        let result = self2.boolean(&self1_inward_offset, op);
        assert!(matches!(
            result.result_info,
            BooleanResultInfo::Pline2InsidePline1
        ));
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "enclosed test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // enclosed self2 NOT self1_offset
    {
        let op = Not;
        let pos_expected = std::slice::from_ref(input_properties);
        let neg_expected = offset_properties;
        let result = self2.boolean(&self1_inward_offset, op);
        assert!(matches!(
            result.result_info,
            BooleanResultInfo::Pline2InsidePline1
        ));
        let pos_pline_result_properties = create_boolean_property_set(&result.pos_plines);
        let neg_pline_result_properties = create_boolean_property_set(&result.neg_plines);
        let passed = property_sets_match_abs_a(&pos_pline_result_properties, pos_expected)
            && property_sets_match_abs_a(&neg_pline_result_properties, neg_expected);
        assert!(
            passed,
            "enclosed test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // enclosed self1_offset NOT self2
    {
        let op = Not;
        let result = self1_inward_offset.boolean(self2, op);
        assert!(matches!(
            result.result_info,
            BooleanResultInfo::Pline1InsidePline2
        ));
        let passed = result.pos_plines.is_empty() && result.neg_plines.is_empty();
        assert!(
            passed,
            "enclosed test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }

    // enclosed XOR
    {
        let op = Xor;
        let pos_expected = std::slice::from_ref(input_properties);
        let neg_expected = offset_properties;
        let result = self2.boolean(&self1_inward_offset, op);
        assert!(matches!(
            result.result_info,
            BooleanResultInfo::Pline2InsidePline1
        ));
        let pos_pline_result_properties = create_boolean_property_set(&result.pos_plines);
        let neg_pline_result_properties = create_boolean_property_set(&result.neg_plines);
        let passed = property_sets_match_abs_a(&pos_pline_result_properties, pos_expected)
            && property_sets_match_abs_a(&neg_pline_result_properties, neg_expected);
        assert!(
            passed,
            "enclosed test failed, boolean op: {op:?}, modified state1: {self1_state:?}, modified state2: {self2_state:?}"
        );
    }
}

struct SameBooleanTestVisitor<'a> {
    input_properties: &'a PlineProperties,
    other_set: ModifiedPlineSet<'a>,
}

impl ModifiedPlineSetVisitor for SameBooleanTestVisitor<'_> {
    fn visit(&mut self, modified_pline: Polyline<f64>, pline_state: ModifiedPlineState) {
        // test every combination of direction and index position cycle
        self.other_set
            .accept_closure(&mut |modified_pline2, pline_state2| {
                run_same_boolean_test(
                    &modified_pline,
                    &modified_pline2,
                    &pline_state,
                    &pline_state2,
                    self.input_properties,
                )
            });
    }
}

fn run_same_boolean_tests(input: &Polyline<f64>) {
    let pline_properties = PlineProperties::from_pline(input, false);
    let other_modified_set = ModifiedPlineSet::new(input, true, true);
    let mut visitor = SameBooleanTestVisitor {
        input_properties: &pline_properties,
        other_set: other_modified_set,
    };

    let test_set = ModifiedPlineSet::new(input, true, true);

    test_set.accept(&mut visitor);
}

macro_rules! declare_same_boolean_tests {
    ($($name:ident { $($pline:expr),+ $(,)? })*) => {
        $(
            #[test]
            fn $name() {
                $(
                    run_same_boolean_tests(&$pline);
                )+
            }
        )+
    };
}

mod test_same {
    use super::*;
    use cavalier_contours::pline_closed;

    declare_same_boolean_tests!(
        origin_circle {
            pline_closed![ (-1.0, 0.0, 1.0), (1.0, 0.0, 1.0) ]
        }
        origin_circle2 {
            pline_closed![ (0.0, -1.0, 1.0), (0.0, 1.0, 1.0) ]
        }
        rectangle {
            pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ]
        }
        diamond {
            pline_closed![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0) ]
        }
        case1 {
            pline_closed![(27.804688, 1.0, 0.0),
                           (28.46842055794889, 0.3429054695163245, 0.0),
                           (32.34577133994935, 0.9269762697003898, 0.0),
                           (32.38116957207762, 1.451312562563487, 0.0),
                           (31.5, 1.0, -0.31783751349740424),
                           (30.79289310940682, 1.5, 0.0),
                           (29.20710689059337, 1.5, -0.31783754777018053),
                           (28.49999981323106, 1.00000000000007, 0.0)]
        }
        case2 {
            pline_closed![(27.804688, 1.0, 0.0),
                           (27.804688, 0.75, 0.0),
                           (32.195313, 0.75, 0.0),
                           (32.195313, 1.0, 0.0),
                           (31.5, 1.0, -0.3178375134974),
                           (30.792893109407, 1.5, 0.0),
                           (29.207106890593, 1.5, -0.31783754777018),
                           (28.499999813231, 1.0000000000001, 0.0)]
        }
        case3 {
            pline_closed![ (0.0, 0.0, 0.0), (120.0, 0.0, 0.0), (120.0, 40.0, 0.0), (0.0, 40.0, 0.0) ]
        }
    );
}

fn verify_slice_set(
    result_pline: &BooleanResultPline<Polyline>,
    pline1: &Polyline,
    pline2: &Polyline,
) {
    if result_pline.subslices.is_empty() {
        return;
    }

    let slice_to_pline = |s: &BooleanPlineSlice| {
        let source = if s.source_is_pline1 { pline1 } else { pline2 };
        Polyline::create_from_remove_repeat(&s.view(source), PlineProperties::POS_EQ_EPS)
    };

    let stitch_slice_onto = |s: &BooleanPlineSlice, target: &mut Polyline| {
        let source = if s.source_is_pline1 { pline1 } else { pline2 };
        target.extend_remove_repeat(&s.view(source), PlineProperties::POS_EQ_EPS)
    };

    let first_slice = &result_pline.subslices[0];
    let mut stitched = slice_to_pline(first_slice);

    for s in result_pline.subslices.iter().skip(1) {
        stitched.remove_last();
        stitch_slice_onto(s, &mut stitched);
    }
    assert!(
        stitched[0].pos().fuzzy_eq(stitched.last().unwrap().pos()),
        "start does not connect with end when stitching slices together"
    );
    stitched.remove_last();
    stitched.set_is_closed(true);

    let expected_properties = PlineProperties::from_pline(&result_pline.pline, false);
    let stitched_properties = PlineProperties::from_pline(&stitched, false);

    assert!(
        expected_properties.fuzzy_eq_eps(&stitched_properties, PlineProperties::PROP_CMP_EPS),
        "slices stitched together do not match result polyline, expected: {expected_properties:?}, actual: {stitched_properties:?}"
    );
}

fn verify_all_slices(
    pline1: &Polyline,
    pline2: &Polyline,
    boolean_result: &BooleanResult<Polyline>,
) {
    for result_pline in boolean_result
        .pos_plines
        .iter()
        .chain(boolean_result.neg_plines.iter())
    {
        verify_slice_set(result_pline, pline1, pline2);
    }
}

fn run_pline_boolean_tests(
    pline1: &Polyline<f64>,
    pline2: &Polyline<f64>,
    cases: &[(BooleanOp, &[PlineProperties], &[PlineProperties])],
) {
    let test_set1 = ModifiedPlineSet::new(pline1, true, true);
    let test_set2 = ModifiedPlineSet::new(pline2, true, true);

    let boolean_options = PlineBooleanOptions {
        // NOTE: we prune collapsed areas for testing as there can be inconsistencies due to float
        // thresholding when inverting direction or cycling vertex index positions
        collapsed_area_eps: Some(1e-5),
        ..Default::default()
    };

    // test every combination of direction and index position cycle between the two polylines
    test_set1.accept_closure(&mut |modified_pline1, state1| {
        test_set2.accept_closure(&mut |modified_pline2, state2| {
            for &(op, pos_set_expected, neg_set_expected) in cases {
                let result = modified_pline1.boolean_opt(&modified_pline2, op, &boolean_options);
                let pos_set_result = create_boolean_property_set(&result.pos_plines);
                let neg_set_result = create_boolean_property_set(&result.neg_plines);
                let passed = property_sets_match_abs_a(&pos_set_result, pos_set_expected)
                    && property_sets_match_abs_a(&neg_set_result, neg_set_expected);
                assert!(
                    passed,
                    "property sets do not match\nop: {op:?}\nstate1: {state1:?}\nstate2: {state2:?}"
                );

                println!("{op:?}");
                verify_all_slices(&modified_pline1, &modified_pline2, &result);
            }
        });
    });
}

macro_rules! declare_boolean_tests {
    ($($name:ident { $($value:expr => $expected:expr),+ $(,)? })*) => {
        $(
            #[test]
            fn $name() {
                $(
                    run_pline_boolean_tests(&$value.0, &$value.1, &$expected);
                )+
            }
        )+
    };
}

mod test_simple {
    use cavalier_contours::pline_closed_userdata;

    use super::*;
    declare_boolean_tests! {
        rectangle_slicing_circle {
            (
                pline_closed_userdata![[4], (0.0, 1.0, 1.0),(10.0, 1.0, 1.0)],
                pline_closed_userdata![[117], (3.0, -10.0, 0.0),(6.0, -10.0, 0.0),(6.0, 10.0, 0.0),(3.0, 10.0, 0.0)])
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(8, 109.15381629282, 52.324068506275, 0.0, -10.0, 10.0, 10.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(2, 29.336980664548, 23.492343031178, 6.0, -3.8989794855664, 10.0, 5.8989794855664, vec![4, 117]), PlineProperties::new(2, 19.816835628274, 20.757946197186, 0.0, -3.5825756949558, 3.0, 5.5825756949558, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 29.386000046923, 25.091858029623, 3.0, -4.0, 6.0, 6.0, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(2, 29.336980664548, 23.492343031178, 6.0, -3.8989794855664, 10.0, 5.8989794855664, vec![4, 117]), PlineProperties::new(2, 19.816835628274, 20.757946197186, -8.8817841970013e-16, -3.5825756949558, 3.0, 5.5825756949558, vec![4, 117]), PlineProperties::new(4, -18.306999976538, 18.582818653767, 3.0, -10.0, 6.0, -3.5825756949558, vec![4, 117]), PlineProperties::new(4, -12.306999976538, 14.582818653767, 3.0, 5.5825756949558, 6.0, 10.0, vec![4, 117]), ], &[])
            ]
        }
        rectangle_over_half_of_circle {
            (
                pline_closed_userdata![[4], (-50.0, 0.0, 1.0), (50.0, 0.0, 1.0)],
                pline_closed_userdata![[117], (-50.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (-50.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 8926.990816987241, 357.0796326794897, -50.0, -50.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(2, 3926.9908169872415, 257.0796326794897, -50.0, 0.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(2, -3926.9908169872415, 257.0796326794897, -50.0, -50.0, 50.0, 0.0, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(2, -3926.9908169872415, 257.0796326794897, -50.0, -50.0, 50.0, 0.0, vec![4, 117]), PlineProperties::new(3, 536.504591506379, 178.53981633974485, 0.0, 0.0, 50.0, 50.0, vec![4, 117]), PlineProperties::new(3, 536.504591506379, 178.53981633974485, -50.0, 0.0, 0.0, 50.0, vec![4, 117])], &[])
            ]
        }
        rectangle_in_rectangle_one_edge_overlap {
            (
                pline_closed_userdata![[4], (0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)],
                pline_closed_userdata![[117], (10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 40.0, 0.0), (10.0, 40.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1200.0, 140.0, 10.0, 10.0, 50.0, 40.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(8, -1300.0, 280.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(8, -1300.0, 280.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[])
            ]
        }
        rectangle_in_rectangle_one_edge_overlap_flipped_order {
            (
                pline_closed_userdata![[4], (10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 40.0, 0.0), (10.0, 40.0, 0.0)],
                pline_closed_userdata![[117], (0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1200.0, 140.0, 10.0, 10.0, 50.0, 40.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(8, 1300.0, 280.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[])
            ]
        }
        rectangle_in_rectangle_two_edge_overlap {
            (
                pline_closed_userdata![[4], (0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)],
                pline_closed_userdata![[117], (10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 50.0, 0.0), (10.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1600.0, 160.0, 10.0, 10.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(6, -900.0, 200.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(6, -900.0, 200.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[])
            ]
        }
        rectangle_in_rectangle_two_edge_overlap_flipped_order {
            (
                pline_closed_userdata![[4], (10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 50.0, 0.0), (10.0, 50.0, 0.0)],
                pline_closed_userdata![[117], (0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1600.0, 160.0, 10.0, 10.0, 50.0, 50.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(6, 900.0, 200.0, 0.0, 0.0, 50.0, 50.0, vec![4, 117])], &[])
            ]
        }
    }
}

mod test_specific {
    use cavalier_contours::pline_closed_userdata;

    use super::*;
    declare_boolean_tests! {
        mostly_overlapping_case1 {
            (
                pline_closed_userdata![[4], (100.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)],
                pline_closed_userdata![[117], (30.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(8, 49966.081474155624, 1146.9875028214367, 30.0, -45.0, 404.41586988912127, 210.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(8, 44023.79002726299, 1033.0222984555098, 80.0, -45.0, 404.41586988912127, 210.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, 5942.291446892632, 585.653349720364, 30.0, 88.52457514062631, 280.0, 172.99920254980566, vec![4, 117])], &[])
            ]
        }
        mostly_overlapping_case2 {
            (
                pline_closed_userdata![[4], (100.0, 100.0, -0.5), (30.0, 100.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)],
                pline_closed_userdata![[117], (30.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 54254.07433137387, 1140.0400125191409, 29.99999999999998, -45.0, 404.41586988912127, 210.0, vec![4, 117])], &[PlineProperties::new(3, -884.615153413562, 120.820138445405, 29.99999999999998, 82.5, 81.20071955870648, 108.1188719599549, vec![4, 117])]),
                (BooleanOp::And, &[PlineProperties::new(8, 44035.3323189534, 1026.5255074451134, 81.20071955870648, -45.0, 404.41586988912127, 210.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(3, -3403.377703804679, 449.8260354415685, 29.99999999999998, -3.4440267329434846, 210.0, 99.99999999999999, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(3, -3403.377703804679, 449.8260354415685, 29.99999999999998, -3.4440267329434846, 210.0, 99.99999999999999, vec![4, 117]), PlineProperties::new(5, 5930.749155202231, 588.2593437968856, 29.99999999999998, 85.61012812790688, 280.0, 172.99920254980566, vec![4, 117])], &[])
            ]
        }
        pill_shapes_overlapping_at_ends_90_deg {
            (
                pline_closed_userdata![[4], (3.0, 7.0, 0.0), (3.0, 4.0, 1.0), (5.0, 4.0, 0.0), (5.0, 7.0, 1.0)],
                pline_closed_userdata![[117], (4.0, 3.0, 0.0), (9.0, 3.0, 1.0), (9.0, 5.0, 0.0), (4.0, 5.0, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 18.926990816987242, 21.853981633974485, 3.0, 3.0, 10.0, 8.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 3.3561944901923457, 6.71238898038469, 3.0, 3.0, 5.0, 5.0, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -5.785398163397448, 10.71238898038469, 3.0, 4.0, 5.0, 8.0, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -5.785398163397448, 10.71238898038469, 3.0, 4.0, 5.0, 8.0, vec![4, 117]), PlineProperties::new(5, 9.785398163397446, 14.71238898038469, 4.0, 3.0, 10.0, 5.0, vec![4, 117])], &[])
            ]
        }
        pill_shapes_overlapping_at_ends_at_acute_angle {
            (
                pline_closed_userdata![[4], (3.0, 7.0, 0.0), (3.0, 4.0, 1.0), (5.0, 4.0, 0.0), (5.0, 7.0, 1.0)],
                pline_closed_userdata![[117], (4.31623, 3.05132, 0.0), (10.3162, 5.05132, 1.0), (9.68377, 6.94868, 0.0), (3.68377, 4.94868, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 21.349440304029333, 24.049941151993778, 3.0, 3.000002084139782, 10.999977834286828, 8.0, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 3.5827364210362647, 7.165481520754011, 3.0, 3.000002084139782, 5.0, 5.3874233333333335, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -5.558852427532134, 10.39063865910795, 3.0, 4.0, 5.0, 8.0, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -5.558852427532134, 10.39063865910795, 3.0, 4.0, 5.0, 8.0, vec![4, 117]), PlineProperties::new(5, 12.207851455460933, 17.039688511108828, 4.31623, 3.05132, 10.999977834286828, 6.999992834286827, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported {
            // reported case which failed, caused by numerical stability problem in
            // line_circle_intr function
            (
                pline_closed_userdata![[4], (161.29, 113.665, 0.0), (167.64, 113.665, 0.0), (167.64, 114.935, 0.0), (161.29, 114.935, 1.0)],
                pline_closed_userdata![[117], (155.575, 113.665, 0.0), (161.29, 113.665, 1.0), (161.29, 114.935, 0.0), (155.575, 114.935, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 15.322550000000774, 26.669999999999987, 155.575, 113.665, 167.64, 114.935, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(2, 1.2667686977437822, 3.989822670059025, 160.655, 113.665, 161.92499999999998, 114.935, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(4, -7.43111565112854, 15.964911335029496, 161.29, 113.665, 167.64, 114.935, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, -7.43111565112854, 15.964911335029496, 161.29, 113.665, 167.64, 114.935, vec![4, 117]), PlineProperties::new(4, 6.624665651128453, 14.694911335029516, 155.575, 113.665, 161.29, 114.935, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported2 {
            // reported case which failed, caused by inconsistent epsilon values used across
            // functions and not scaling parametric t values and angles by lengths/radii for
            // fuzzy comparing with epsilon values
            (
                pline_closed_userdata![[4], (152.874687837651, 123.98368333641, 0.0), (160.791017837651, 113.90835333641, 1.0), (161.789642162349, 114.69298666359, 0.0), (153.873312162349, 124.76831666359, 1.0)],
                pline_closed_userdata![[117], (160.7910178773358, 113.9083532859021, 0.0), (166.8446878773358, 106.2036832859021, 1.0), (167.8433121226642, 106.9883167140979, 0.0), (161.7896421226642, 114.6929867140979, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 29.98363677157613, 49.21323695956188, 152.739, 105.961, 167.979, 125.01100000000002, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(2, 1.2667686977438057, 3.9898226700590773, 160.65533001984238, 113.66566997474605, 161.92533001984242, 114.93566997474606, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(4, -16.27288600510112, 29.616414804078143, 152.739, 113.9083532859021, 161.789642162349, 125.01100000000002, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, -16.27288600510112, 29.616414804078143, 152.739, 113.9083532859021, 161.789642162349, 125.01100000000002, vec![4, 117]), PlineProperties::new(4, 12.443982068731202, 23.586644825542812, 160.7910178773358, 105.961, 167.979, 114.69298666359, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported3 {
            // reported case which failed, caused by fuzzy compare between squared numbers which are
            //  not to scale with epsilon value in line_circle_intr function
            (
                pline_closed_userdata![[4], (113.1450199999994, 99.04090098302, 0.0), (113.1449999999994, 114.30000098302, 1.0), (111.6450000000006, 114.29999901698, 0.0), (111.6450200000006, 99.04089901698, 1.0)],
                pline_closed_userdata![[117], (113.145, 114.3, 0.0), (113.145, 117.475, 1.0), (111.645, 117.475, 0.0), (111.645, 114.3, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 29.418295867659253, 41.58058898041103, 111.645, 98.29089999999995, 113.14502000000005, 118.225, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(2, 1.7671458676433858, 4.712388980383754, 111.64500000000044, 113.54999950849015, 113.14500000000015, 115.04999950848986, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(4, -22.88864926275236, 35.230587997390565, 111.6450000000006, 98.29089999999995, 113.14502000000005, 114.3, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, -22.88864926275236, 35.230587997390565, 111.6450000000006, 98.29089999999995, 113.14502000000005, 114.3, vec![4, 117]), PlineProperties::new(4, 4.762500737263508, 11.062389963404218, 111.645, 114.29999901698, 113.145, 118.225, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported4 {
            // reported case which failed with large epsilon values (which were just too large)
            // did not fail with reasonable epsilon values, added as just an additional case to test
            (
                pline_closed_userdata![[4], (152.2788842579442, 58.514514467757, 0.0), (151.6438842579442, 53.434514467757, 1.0), (153.1561157420558, 53.245485532243, 0.0), (153.7911157420558, 58.325485532243, 1.0)],
                pline_closed_userdata![[117], (153.162, 52.705, 0.0), (153.162, 53.34, 1.0), (151.638, 53.34, 0.0), (151.638, 52.705, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 10.59400964081351, 16.296732241847607, 151.638, 51.943, 153.797, 59.18199999999999, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1.8241935220269396, 4.787909506633163, 151.638, 52.57800000000002, 153.162, 54.102000000000004, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -7.802122716062558, 14.932096038564826, 151.6438842579442, 53.29255959579674, 153.797, 59.18199999999999, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -7.802122716062558, 14.932096038564826, 151.6438842579442, 53.29255959579674, 153.797, 59.18199999999999, vec![4, 117]), PlineProperties::new(5, 0.9676934027233983, 5.963028698226644, 151.638, 51.943, 153.162, 53.34, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported5 {
            // reported case which failed, caused by not making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (10.2548112191951, 4.4473618027979, 0.0), (5.8085712191951, 8.0043518027979, 1.0), (5.0152087808049, 7.0126481972021, 0.0), (9.4614487808049, 3.4556581972021, 1.0)],
                pline_closed_userdata![[117], (5.8089199887361, 8.0040725860499, -0.2276528214202), (5.09501, 9.4907900625929, 1.0), (3.82501, 9.4907899374071, 0.2276528214202), (5.0148600112639, 7.0129274139501, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 11.38632932320666, 19.926138990954986, 3.825009999999997, 3.31650999999997, 10.49313000000003, 10.125790000000004, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.2667686977282067, 3.9898226700509536, 4.77688999999997, 6.873499999999976, 6.046889999999988, 8.143500000000031, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -7.231334499089485, 15.377751015048563, 5.0152087808049, 3.31650999999997, 10.49313000000003, 8.0043518027979, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -7.231334499089485, 15.377751015048563, 5.0152087808049, 3.31650999999997, 10.49313000000003, 8.0043518027979, vec![4, 117]), PlineProperties::new(4, 2.888226126389008, 8.537317108731827, 3.825009999999997, 7.0129274139501, 5.8085712191951, 10.125790000000004, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported5_modified {
            // found by modifying previous case slightly, caused by not making arc-arc intersect
            // case sticky to end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (10.2548112191951, 4.4473618027979, 0.0), (5.8085712191951, 8.0043518027979, 1.0), (5.0152087808049, 7.0126481972021, 0.0), (9.4614487808049, 3.4556581972021, 1.0)],
                pline_closed_userdata![[117], (5.80888046975325, 8.00410413311421, -0.2276528214202), (5.09501, 9.4907900625929, 1.0), (3.82501, 9.4907899374071, 0.2276528214202), (5.0148600112639, 7.0129274139501, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 11.386322805194794, 19.92614241253084, 3.825009999999997, 3.31650999999997, 10.49313000000003, 10.125790000000004, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.2667365182740837, 3.9897719937466536, 4.77688999999997, 6.873515801569765, 6.046870199969008, 8.143500000000031, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -7.231366678543611, 15.377700338744262, 5.0152087808049, 3.31650999999997, 10.49313000000003, 8.0043518027979, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -7.231366678543611, 15.377700338744262, 5.0152087808049, 3.31650999999997, 10.49313000000003, 8.0043518027979, vec![4, 117]), PlineProperties::new(4, 2.88821960837715, 8.537320530307678, 3.825009999999997, 7.0129274139501, 5.8085712191951, 10.125790000000004, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported6 {
            // reported case which failed, caused by not making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (34.5520701659868, 13.58, -0.3443278085713), (39.2580061973296, 9.9033180119262, 1.0), (42.1688938026704, 10.6290819880738, 0.3443278085713), (34.5520698340132, 16.58, 1.0)],
                pline_closed_userdata![[117], (43.505213750218, 5.2838034375545, 0.0), (42.168663750218, 10.6300034375545, 1.0), (39.258236249782, 9.9023965624455, 0.0), (40.594786249782, 4.5561965624455, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 48.869685074349405, 37.29217971282884, 33.05206999999999, 3.4199999999999973, 43.55, 16.58000000000003, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 7.068583470538565, 9.424777960754131, 39.21345, 8.766199999999992, 42.213450000000016, 11.766200000000001, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(4, -25.2688911320144, 26.2688065984268, 33.05206999999999, 9.9033180119262, 42.168663750218, 16.58000000000003, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, -25.2688911320144, 26.2688065984268, 33.05206999999999, 9.9033180119262, 42.168663750218, 16.58000000000003, vec![4, 117]), PlineProperties::new(5, 16.53221047179639, 20.446251608592696, 39.258236249782, 3.4199999999999973, 43.55, 10.6300034375545, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported7 {
            // reported case which failed with large epsilon values (which were just too large)
            // did not fail with reasonable epsilon values, added as just an additional case to test
            (
                pline_closed_userdata![[4], (24.2719330436524, 4.8163319805663, 0.0), (16.1052530436524, 14.7344219805663, 1.0), (15.3209269563476, 14.0885980194337, 0.0), (23.4876069563476, 4.1705080194337, 1.0)],
                pline_closed_userdata![[117], (11.7209392690379, 15.8420000000005, -0.2299490931599), (15.3138743717012, 14.0973589501497, 1.0), (16.1123056282988, 14.7256610498503, 0.2299490931599), (11.7209407309621, 16.8579999999995, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 18.530198218309717, 38.07269700558251, 11.212939999999975, 3.985420000000036, 24.387769999999964, 16.85800000000003, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 0.8107320777579569, 3.1918585636697134, 15.20508999999996, 13.90351000000301, 16.22108999999764, 14.919509999999981, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -13.053254238933325, 28.87626153226048, 15.3209269563476, 3.985420000000036, 24.387769999999964, 14.730183990954652, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -13.053254238933325, 28.87626153226048, 15.3209269563476, 3.985420000000036, 24.387769999999964, 14.730183990954652, vec![4, 117]), PlineProperties::new(5, 4.666211901614574, 12.365799768897254, 11.212939999999975, 14.0973589501497, 16.108742657580247, 16.85800000000003, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported8 {
            // reported case which failed, caused by not making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (9.205577051997063, 7.029369955609887, 0.0), (18.23161705199706, 2.014909955609888, 1.0), (18.84838294800294, 3.125090044390112, 0.0), (9.82234294800294, 8.139550044390113, 1.0)],
                pline_closed_userdata![[117], (9.822782733308527, 8.139305491458524, -0.207545228447529), (7.056947779266055, 11.750330431457192, 1.0), (5.824672220733945, 11.443089568542808, 0.207545228447529), (9.205137266691475, 7.029614508541476, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 20.98132045896099, 35.0363617834408, 5.805810000000001, 1.9349999999999996, 19.175, 12.23171, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.2667686977251513, 3.9898226700460335, 8.878960000000001, 6.94946, 10.14896, 8.21946, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -13.113277803438073, 24.64065385653342, 9.205577051997063, 1.9349999999999996, 19.175, 8.139550044390113, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -13.113277803438073, 24.64065385653342, 9.205577051997063, 1.9349999999999996, 19.175, 8.139550044390113, vec![4, 117]), PlineProperties::new(4, 6.601273957797762, 14.384524182993633, 5.805810000000001, 7.029614508541476, 9.82234294800294, 12.23171, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported9 {
            // reported case which failed, fixed after making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (6.012782733308527, 6.869305491458524, -0.207545228447529), (3.246947779266055, 10.480330431457192, 1.0), (2.014672220733944, 10.173089568542808, 0.207545228447529), (5.395137266691473, 5.759614508541476, 1.0)],
                pline_closed_userdata![[117], (1.413958758496091, 12.575992893779793, 0.0), (2.014768758496091, 10.172702893779793, 1.0), (3.246851241503909, 10.480717106220208, 0.0), (2.646041241503909, 12.884007106220208, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 11.014152378234503, 19.340033655873068, 1.3950000000000002, 5.67946, 6.338960000000001, 13.365, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.26676869773619, 3.9898226700525505, 1.9958099999999992, 9.69171, 3.2658099999999997, 10.96171, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(4, -6.601274150926578, 14.384733372719799, 2.014768758496091, 5.67946, 6.338960000000001, 10.480330431457192, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, -6.601274150926578, 14.384733372719799, 2.014768758496091, 5.67946, 6.338960000000001, 10.480330431457192, vec![4, 117]), PlineProperties::new(5, 3.1461095295717385, 8.94432586621662, 1.3950000000000002, 10.172702893779793, 3.246851241503909, 13.365, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported10 {
            // reported case which failed, fixed after making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (13.015577051997061, 9.569369955609886, 0.0), (22.04161705199706, 4.554909955609888, 1.0), (22.65838294800294, 5.665090044390113, 0.0), (13.632342948002938, 10.679550044390112, 1.0)],
                pline_closed_userdata![[117], (13.632782733308526, 10.679305491458523, -0.20754522844753), (10.866947779266056, 14.29033043145719, 1.0), (9.634672220733943, 13.98308956854281, 0.20754522844753), (13.015137266691474, 9.569614508541475, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 20.98132045896099, 35.03636178344082, 9.61581, 4.4750000000000005, 22.985000000000003, 14.77171, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.2667686977251869, 3.989822670046036, 12.68896, 9.489459999999998, 13.95896, 10.75946, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -13.113277803438049, 24.640653856533426, 13.015577051997061, 4.4750000000000005, 22.985000000000003, 10.679550044390112, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -13.113277803438049, 24.640653856533426, 13.015577051997061, 4.4750000000000005, 22.985000000000003, 10.679550044390112, vec![4, 117]), PlineProperties::new(4, 6.601273957797755, 14.384524182993642, 9.61581, 9.569614508541475, 13.632342948002938, 14.77171, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported11 {
            // reported case which failed, fixed after making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (9.033958758496091, 16.38599289377979, 0.0), (9.63476875849609, 13.982702893779793, 1.0), (10.866851241503909, 14.290717106220209, 0.0), (10.26604124150391, 16.69400710622021, 1.0)],
                pline_closed_userdata![[117], (13.632782733308526, 10.679305491458523, -0.20754522844753), (10.866947779266056, 14.29033043145719, 1.0), (9.634672220733943, 13.98308956854281, 0.20754522844753), (13.015137266691474, 9.569614508541475, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 11.01415237823448, 19.340033655873075, 9.014999999999999, 9.48946, 13.95896, 17.175, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.2667686977361845, 3.9898226700525523, 9.61581, 13.501710000000001, 10.88581, 14.77171, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -3.146109529571715, 8.944325866216621, 9.014999999999999, 13.982702893779793, 10.866851241503909, 17.175, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -3.146109529571715, 8.944325866216621, 9.014999999999999, 13.982702893779793, 10.866851241503909, 17.175, vec![4, 117]), PlineProperties::new(4, 6.601274150926551, 14.384733372719804, 9.63476875849609, 9.48946, 13.95896, 14.29033043145719, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported12 {
            // reported case which failed, fixed after making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (25.604811219195092, 15.257361802797867, 0.0), (21.158571219195093, 18.81435180279787, 1.0), (20.36520878080491, 17.82264819720213, 0.0), (24.81144878080491, 14.265658197202132, 1.0)],
                pline_closed_userdata![[117], (21.158919988736113, 18.81407258604992, -0.227652821420176), (20.445009999999996, 20.300790062592878, 1.0), (19.17501, 20.30078993740712, 0.227652821420176), (20.36486001126389, 17.82292741395008, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 11.386329323206176, 19.926138990954883, 19.175009999999997, 14.12651, 25.84313, 20.93579, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.2667686977281605, 3.989822670050854, 20.12689, 17.6835, 21.396890000000003, 18.953500000000002, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -7.231334499089066, 15.377751015048462, 20.36520878080491, 14.12651, 25.84313, 18.81435180279787, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -7.231334499089066, 15.377751015048462, 20.36520878080491, 14.12651, 25.84313, 18.81435180279787, vec![4, 117]), PlineProperties::new(4, 2.888226126388971, 8.53731710873184, 19.175009999999997, 17.82292741395008, 21.158571219195093, 20.93579, vec![4, 117])], &[])
            ]
        }
        overlapping_pill_shaped_ends_reported13 {
            // reported case which failed, fixed after making arc-arc intersect case sticky to
            // end points the same as line-arc intersect case
            (
                pline_closed_userdata![[4], (31.425213750217996, 14.3638034375545, 0.0), (30.088663750217997, 19.710003437554498, 1.0), (27.178236249782003, 18.9823965624455, 0.0), (28.514786249782002, 13.6361965624455, 1.0)],
                pline_closed_userdata![[117], (22.47207016598675, 22.66000000000001, -0.344327808571312), (27.17800619732963, 18.983318011926176, 1.0), (30.08889380267037, 19.709081988073823, 0.344327808571312), (22.472069834013247, 25.65999999999999, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 48.86968507434945, 37.29217971282889, 20.97207, 12.500000000000002, 31.47, 25.660000000000025, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 7.068583470538498, 9.424777960754085, 27.13345, 17.8462, 30.13345, 20.8462, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(5, -16.53221047179636, 20.446251608592654, 27.178236249782003, 12.500000000000002, 31.47, 19.710003437554498, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(5, -16.53221047179636, 20.446251608592654, 27.178236249782003, 12.500000000000002, 31.47, 19.710003437554498, vec![4, 117]), PlineProperties::new(4, 25.268891132014584, 26.268806598426877, 20.97207, 18.983318011926176, 30.088663750217997, 25.660000000000025, vec![4, 117])], &[])
            ]
        }
        parametric_from_point_debug_assert_reported1 {
            // reported case triggered debug assert in parametric_from_point (unexpected loss in float accuracy)
            (
                pline_closed_userdata![[4], (11.664990000000303, 18.442909378846142, 0.0), (11.665000000000305, 8.219999378846142, 1.0), (12.934999999999697, 8.22000062115386, 0.0), (12.934989999999695, 18.44291062115386, 1.0)],
                pline_closed_userdata![[117], (13.062000000000001, 8.22, 0.0), (13.062000000000001, 10.125, 1.0), (11.538, 10.125, 0.0), (11.538, 8.22, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(8, 15.085029101721778, 24.884787008560057, 11.538, 7.458, 13.062000000000001, 19.077910000000003, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 3.8922022207111775, 8.148642865490364, 11.664997724514617, 7.585000000000001, 12.935, 10.886999999964068, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(4, -10.357662177038755, 19.289617472302368, 11.66499, 10.54620791786424, 12.934997724508513, 19.077910000000003, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, -10.357662177038755, 19.289617472302368, 11.66499, 10.54620791786424, 12.934997724508513, 19.077910000000003, vec![4, 117]), PlineProperties::new(8, 0.8351647039718321, 13.743812401748059, 11.538, 7.458, 13.062000000000001, 10.546214778732427, vec![4, 117])], &[])
            ]
        }
        parametric_from_point_debug_assert_reported2 {
            // reported case triggered debug assert in parametric_from_point (unexpected loss in float accuracy)
            (
                pline_closed_userdata![[4], (15.460744844799105, 20.44135329367296, 0.2839154127006927), (14.548990000000272, 18.965099544964527, 0.9999999999999999), (15.310989999999729, 18.965100455035472, -0.28391541270069265), (15.801935155200894, 19.76000670632704, 0.9999999999999999)],
                pline_closed_userdata![[117], (14.548990000025455, 18.96509559588487, 0.0), (14.549000000025455, 18.09999559588487, 1.0), (15.310999999974545, 18.100004404115133, 0.0), (15.310989999974545, 18.96510440411513, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(6, 2.1860886410435683, 6.934715005959068, 14.548990000020739, 17.719, 16.012340000000002, 20.48168, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(2, 0.4560367311877418, 2.3938936020354182, 14.54899, 18.5841, 15.31099, 19.3461, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(4, -1.0708457098118014, 5.2045150058434775, 14.54899, 18.96509559588487, 16.012340000000002, 20.48168, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, -1.0708457098118014, 5.2045150058434775, 14.54899, 18.96509559588487, 16.012340000000002, 20.48168, vec![4, 117]), PlineProperties::new(4, 0.6592062000440251, 4.124093602151008, 14.548990000025455, 17.719, 15.311, 18.96510440411513, vec![4, 117])], &[])
            ]
        }
        debug_assert_stitching_slices_reported {
            // reported case triggered debug assert while closing stitched slices together into closed pline (unexpected loss in float accuracy)
            (
                pline_closed_userdata![[4], (71.44735180279787, 41.015208780804905, 0.0), (75.00434180279787, 45.46144878080491, 1.0), (74.01263819720212, 46.2548112191951, 0.0), (70.45564819720212, 41.808571219195095, -0.22759115259754015), (68.96920993740713, 41.095009999999995, 0.9999999999999999), (68.96921006259288, 39.82501, 0.22765282142017604)],
                pline_closed_userdata![[117], (62.570000992309986, 39.82500000000078, 0.0), (68.96921099231, 39.82501000000077, 1.0), (68.96920900769001, 41.09500999999922, 0.0), (62.569999007690015, 41.094999999999224, 1.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(9, 19.513229882549645, 32.72452861196257, 61.935, 39.825, 75.14349, 46.49313000000001, vec![4, 117])], &[]),
                (BooleanOp::And, &[PlineProperties::new(3, 1.2667686977434385, 3.9898226700588975, 68.33421, 39.82501, 69.60421000000001, 41.09500999999999, vec![4, 117])], &[]),
                (BooleanOp::Not, &[PlineProperties::new(6, -10.119464484796188, 19.925886964359684, 68.96920900769001, 39.82501000984475, 75.14349, 46.49313000000001, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(6, -10.119464484796188, 19.925886964359684, 68.96920900769001, 39.82501000984475, 75.14349, 46.49313000000001, vec![4, 117]), PlineProperties::new(4, 8.12699670000984, 16.788242670074666, 61.935, 39.825, 68.96921099231, 41.09500999999922, vec![4, 117])], &[])
            ]
        }
        opposite_direction_arc_arc_endpoints_intersect_bug_reported {
            // Test case for issue: https://github.com/jbuckmccready/cavalier_contours/issues/42
            // Caused by arc-arc seg intersect bug
            (
                pline_closed_userdata![[4], (-188.500000000023, -166.831646988729, 0.0), (-188.5, -195.881478300073, 0.0), (-189.0, -196.91384910249, 0.553407781718062), (-170.999999999999, -225.631646989572, -0.553407781718061), (-153.0, -196.91384910249, -0.553407781718095), (-153.5, -195.881478300072, 0.0), (-153.5, -166.831646988778, 0.0), (-153.5, -166.820646988779, 0.0), (-188.500000000023, -166.820646988729, 0.0)],
                pline_closed_userdata![[117], (412.0, -246.331646989572, 0.0), (412.0, -156.831646989572, 0.0), (319.0, -156.831646989571, 0.0), (319.0, -193.831646989571, 0.0), (317.0, -193.831646989571, 0.0), (317.0, -156.831646989572, 0.0), (-153.5, -156.831646989572, 0.0), (-153.5, -195.881478300072, 0.0), (-153.0, -196.91384910249, -0.553407781718061), (-171.0, -225.631646989571, -0.553407781718061), (-189.0, -196.91384910249, 0.0), (-188.5, -195.881478300073, 0.0), (-188.5, -156.831646989571, 0.0), (-498.0, -156.831646989571, 0.0), (-498.0, -193.831646989572, 0.0), (-448.0, -193.831646989572, 0.0), (-448.0, -228.831646989571, 0.0), (-538.0, -228.831646989571, 0.0), (-538.0, -193.831646989572, 0.0), (-500.0, -193.831646989572, 0.0), (-500.0, -156.831646989572, 0.0), (-618.0, -156.831646989572, 0.0), (-618.0, -246.331646989572, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(20, 88537.38500002908, 2652.977999998363, -618.0, -246.331646989572, 412.0, -156.831646989571, vec![4, 117])], &[PlineProperties::new(2, 448.72925543646033, 80.87292554364588, -173.0, -225.631646989571, -151.0, -196.91384910249, vec![4, 117]), PlineProperties::new(2, 0.25699795696345973, 2.515629079078068, -153.60334994589547, -196.91384910249, -153.0, -195.881478300072, vec![4, 117])]),
                (BooleanOp::And, &[], &[]),
                (BooleanOp::Not, &[PlineProperties::new(7, 1570.4416233924637, 176.51021724536002, -191.0, -225.631646989571, -153.0, -166.820646988729, vec![4, 117])], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(7, 1570.4416233924637, 176.51021724536002, -191.0, -225.631646989571, -153.0, -166.820646988729, vec![4, 117]), PlineProperties::new(23, 86517.95712324319, 2759.2667455033516, -618.0, -246.331646989572, 412.0, -156.831646989571, vec![])], &[])
            ]
        }
    }
}
