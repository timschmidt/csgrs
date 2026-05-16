use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

use cavalier_contours::{
    assert_fuzzy_eq,
    core::math::{Vector2, bulge_from_angle, point_on_circle},
    pline_closed, pline_open,
    polyline::{PlineCreation, PlineSource, PlineSourceMut, PlineVertex, PlineViewData, Polyline},
};

const POS_EQ_EPS: f64 = 1e-5;

#[test]
fn from_slice_points_single_seg() {
    let pline = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0)];

    // complete polyline
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // complete polyline (end segment index on top of final vertex)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // slice from start to middle
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.0, 0.0, bulge), (0.5, -0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end (end segment index on top of final vertex)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from first third to second third of the segment
    {
        let start_point = point_on_circle(0.5, Vector2::new(0.5, 0.0), PI + FRAC_PI_3);
        let end_point = point_on_circle(0.5, Vector2::new(0.5, 0.0), PI + 2.0 * FRAC_PI_3);
        let slice =
            PlineViewData::from_slice_points(&pline, start_point, 0, end_point, 0, POS_EQ_EPS)
                .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_3);
        let expected_result = {
            let mut p = Polyline::new();
            p.add_vertex(PlineVertex::from_vector2(start_point, bulge));
            p.add_vertex(PlineVertex::from_vector2(end_point, 0.0));
            p
        };

        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // collapsed slice at start
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.0, 0.0),
            0,
            POS_EQ_EPS,
        );

        assert!(slice.is_none());
    }

    // collapsed slice at end
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        );

        assert!(slice.is_none());
    }

    let closed_pline = pline_closed![
        (0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        (5.0, 5.0, 0.0),
        (0.0, 5.0, 0.0)
    ];

    // collapsed closed polyline (by having start and end point same with same segment index)
    {
        let slice = PlineViewData::from_slice_points(
            &closed_pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.0, 0.0),
            0,
            POS_EQ_EPS,
        );

        assert!(slice.is_none());
    }

    // complete closed polyline (by having end point be at end of last segment)
    {
        let slice = PlineViewData::from_slice_points(
            &closed_pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.0, 0.0),
            3,
            POS_EQ_EPS,
        )
        .unwrap();

        for ((v1, v2), (u1, u2)) in closed_pline
            .iter_segments()
            .zip(slice.view(&closed_pline).iter_segments())
        {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2, u2);
        }
    }
}

#[test]
fn from_slice_points_multi_seg() {
    let pline = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)];
    // complete polyline
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 1.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = {
            let mut pl = pline.clone();
            pl.set_is_closed(false);
            pl
        };
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // complete polyline (end segment index on top of last vertex)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 1.0),
            2,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = {
            let mut pl = pline.clone();
            pl.set_is_closed(false);
            pl
        };
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from start to middle of first segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.0, 0.0, bulge), (0.5, -0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end of first segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from start to second vertex
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from start to middle of second segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from second vertex to middle of second segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            1,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from second vertex to middle of second segment (using previous index for start)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            0,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle of first segment to last vertex position
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 1.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle of end segment to middle of first segment (wrapping)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.5),
            1,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![
            (1.0, 0.5, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 0.0, bulge),
            (0.5, -0.5, 0.0)
        ];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // collapsed slice at start
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.0, 0.0),
            0,
            POS_EQ_EPS,
        );
        assert!(slice.is_none());
    }

    // collapsed slice at midpoint of second segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.5),
            1,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        );
        assert!(slice.is_none());
    }

    // slice from middle of first segment wrapping back to start of first segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(0.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![
            (0.5, -0.5, bulge),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 0.0, 0.0)
        ];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle of second segment wrapping back to middle of first segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.5),
            1,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![
            (1.0, 0.5, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 0.0, bulge),
            (0.5, -0.5, 0.0)
        ];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }
}

#[test]
#[should_panic(
    expected = "start index should be less than or equal to end index if polyline is open"
)]
fn attempting_to_wrap_slice_on_open_pline() {
    let pline = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)];
    let _ = PlineViewData::from_slice_points(
        &pline,
        Vector2::new(1.0, 0.5),
        1,
        Vector2::new(0.5, -0.5),
        0,
        POS_EQ_EPS,
    );
}

#[test]
fn from_new_start() {
    let closed_pline = pline_closed![
        (0.0, 0.0, 0.0),
        (5.0, 0.0, 0.0),
        (5.0, 5.0, 0.0),
        (0.0, 5.0, 0.0)
    ];

    let closed_pline_with_bulges = pline_closed![
        (0.0, 0.0, 0.1),
        (5.0, 0.0, 0.2),
        (5.0, 5.0, 0.3),
        (0.0, 5.0, 0.4)
    ];

    // change start on first segment of closed polyline
    {
        let view_data =
            PlineViewData::from_new_start(&closed_pline, Vector2::new(1.5, 0.0), 0, POS_EQ_EPS)
                .unwrap();

        let expected = pline_closed![
            (1.5, 0.0, 0.0),
            (5.0, 0.0, 0.0),
            (5.0, 5.0, 0.0),
            (0.0, 5.0, 0.0),
            (0.0, 0.0, 0.0),
        ];

        let view = view_data.view(&closed_pline);

        assert_eq!(expected.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in expected.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on top of first vertex of closed polyline (no change)
    {
        let view_data =
            PlineViewData::from_new_start(&closed_pline, Vector2::new(0.0, 0.0), 0, POS_EQ_EPS)
                .unwrap();

        let view = view_data.view(&closed_pline);

        assert_eq!(closed_pline.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in closed_pline.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on top of first vertex of closed polyline with bulge values (no change)
    {
        let view_data = PlineViewData::from_new_start(
            &closed_pline_with_bulges,
            Vector2::new(0.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let view = view_data.view(&closed_pline_with_bulges);

        assert_eq!(
            closed_pline_with_bulges.segment_count(),
            view.segment_count()
        );

        for ((v1, v2), (u1, u2)) in closed_pline_with_bulges
            .iter_segments()
            .zip(view.iter_segments())
        {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on top of first vertex of closed polyline (no change) (using last segment index
    // that puts point on end of segment)
    {
        let view_data =
            PlineViewData::from_new_start(&closed_pline, Vector2::new(0.0, 0.0), 3, POS_EQ_EPS)
                .unwrap();

        let view = view_data.view(&closed_pline);

        assert_eq!(closed_pline.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in closed_pline.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on top of second vertex of closed polyline
    {
        let view_data =
            PlineViewData::from_new_start(&closed_pline, Vector2::new(5.0, 0.0), 1, POS_EQ_EPS)
                .unwrap();

        let expected = pline_closed![
            (5.0, 0.0, 0.0),
            (5.0, 5.0, 0.0),
            (0.0, 5.0, 0.0),
            (0.0, 0.0, 0.0),
        ];

        let view = view_data.view(&closed_pline);

        assert_eq!(expected.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in expected.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on top of first vertex of closed polyline with bulge values (no change)
    {
        let view_data = PlineViewData::from_new_start(
            &closed_pline_with_bulges,
            Vector2::new(5.0, 0.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let expected = pline_closed![
            (5.0, 0.0, 0.2),
            (5.0, 5.0, 0.3),
            (0.0, 5.0, 0.4),
            (0.0, 0.0, 0.1),
        ];

        let view = view_data.view(&closed_pline_with_bulges);

        assert_eq!(expected.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in expected.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on top of second vertex of closed polyline (using last segment index
    // that puts point on end of segment)
    {
        let view_data =
            PlineViewData::from_new_start(&closed_pline, Vector2::new(5.0, 0.0), 0, POS_EQ_EPS)
                .unwrap();

        let expected = pline_closed![
            (5.0, 0.0, 0.0),
            (5.0, 5.0, 0.0),
            (0.0, 5.0, 0.0),
            (0.0, 0.0, 0.0),
        ];

        let view = view_data.view(&closed_pline);

        assert_eq!(expected.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in expected.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on second segment of closed polyline
    {
        let view_data =
            PlineViewData::from_new_start(&closed_pline, Vector2::new(5.0, 2.22), 1, POS_EQ_EPS)
                .unwrap();

        let expected = pline_closed![
            (5.0, 2.22, 0.0),
            (5.0, 5.0, 0.0),
            (0.0, 5.0, 0.0),
            (0.0, 0.0, 0.0),
            (5.0, 0.0, 0.0),
        ];

        let view = view_data.view(&closed_pline);

        assert_eq!(expected.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in expected.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }

    // change start on last segment of closed polyline
    {
        let view_data =
            PlineViewData::from_new_start(&closed_pline, Vector2::new(0.0, 2.22), 3, POS_EQ_EPS)
                .unwrap();

        let expected = pline_closed![
            (0.0, 2.22, 0.0),
            (0.0, 0.0, 0.0),
            (5.0, 0.0, 0.0),
            (5.0, 5.0, 0.0),
            (0.0, 5.0, 0.0),
        ];

        let view = view_data.view(&closed_pline);

        assert_eq!(expected.segment_count(), view.segment_count());

        for ((v1, v2), (u1, u2)) in expected.iter_segments().zip(view.iter_segments()) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2.pos(), u2.pos());
        }
    }
}
