use cavalier_contours::{
    assert_fuzzy_eq,
    core::{
        math::{Vector2, bulge_from_angle},
        traits::FuzzyEq,
    },
    pline_closed, pline_open,
    polyline::{
        PlineCreation, PlineSource, PlineSourceMut, PlineVertex, PlineViewData, Polyline,
        seg_length,
    },
};
use std::f64::consts::{PI, TAU};

#[test]
fn iter_vertexes() {
    fn run_iter_vertexes_tests(is_closed: bool) {
        let mut polyline = Polyline::with_capacity(0, is_closed);
        {
            // empty
            let mut iter = polyline.iter_vertexes();
            assert_eq!(iter.size_hint(), (0, Some(0)));
            assert_eq!(iter.next(), None);
        }

        polyline.add(1.0, 2.0, 0.3);

        {
            // one vertex next
            let mut iter = polyline.iter_vertexes();
            assert_eq!(iter.size_hint(), (1, Some(1)));
            assert_eq!(iter.next(), Some(PlineVertex::new(1.0, 2.0, 0.3)));
            assert_eq!(iter.size_hint(), (0, Some(0)));
            assert_eq!(iter.next(), None);
            assert_eq!(iter.next_back(), None);
        }

        {
            // one vertex next
            let mut iter = polyline.iter_vertexes();
            assert_eq!(iter.size_hint(), (1, Some(1)));
            assert_eq!(iter.next_back(), Some(PlineVertex::new(1.0, 2.0, 0.3)));
            assert_eq!(iter.size_hint(), (0, Some(0)));
            assert_eq!(iter.next_back(), None);
            assert_eq!(iter.next(), None);
        }

        polyline.add(4.0, 5.0, 0.6);

        {
            // two vertex next
            let mut iter = polyline.iter_vertexes();
            assert_eq!(iter.size_hint(), (2, Some(2)));
            assert_eq!(iter.next(), Some(PlineVertex::new(1.0, 2.0, 0.3)));
            assert_eq!(iter.size_hint(), (1, Some(1)));
            assert_eq!(iter.next(), Some(PlineVertex::new(4.0, 5.0, 0.6)));
            assert_eq!(iter.size_hint(), (0, Some(0)));
            assert_eq!(iter.next_back(), None);
            assert_eq!(iter.next(), None);
        }

        {
            // two vertex next_back
            let mut iter = polyline.iter_vertexes();
            assert_eq!(iter.size_hint(), (2, Some(2)));
            assert_eq!(iter.next_back(), Some(PlineVertex::new(4.0, 5.0, 0.6)));
            assert_eq!(iter.size_hint(), (1, Some(1)));
            assert_eq!(iter.next_back(), Some(PlineVertex::new(1.0, 2.0, 0.3)));
            assert_eq!(iter.size_hint(), (0, Some(0)));
            assert_eq!(iter.next_back(), None);
            assert_eq!(iter.next(), None);
        }

        {
            // two vertex next and next_back
            let mut iter = polyline.iter_vertexes();
            assert_eq!(iter.size_hint(), (2, Some(2)));
            assert_eq!(iter.next(), Some(PlineVertex::new(1.0, 2.0, 0.3)));
            assert_eq!(iter.size_hint(), (1, Some(1)));
            assert_eq!(iter.next_back(), Some(PlineVertex::new(4.0, 5.0, 0.6)));
            assert_eq!(iter.size_hint(), (0, Some(0)));
            assert_eq!(iter.next_back(), None);
            assert_eq!(iter.next(), None);
        }
    }

    // should have same results for both open and closed polyline
    run_iter_vertexes_tests(false);
    run_iter_vertexes_tests(true);
}

#[test]
fn iter_segments() {
    let mut polyline = Polyline::<f64>::new();
    assert_eq!(polyline.iter_segments().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segments().collect::<Vec<_>>().len(), 0);

    polyline.add(1.0, 2.0, 0.3);
    assert_eq!(polyline.iter_segments().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segments().collect::<Vec<_>>().len(), 0);

    polyline.add(4.0, 5.0, 0.6);
    assert_eq!(polyline.iter_segments().size_hint(), (1, Some(1)));
    let one_seg = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(one_seg.len(), 1);
    assert_eq!(one_seg[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(one_seg[0].1, PlineVertex::new(4.0, 5.0, 0.6));

    polyline.set_is_closed(true);
    assert_eq!(polyline.iter_segments().size_hint(), (2, Some(2)));
    let two_seg = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(two_seg.len(), 2);
    assert_eq!(two_seg[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(two_seg[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg[1].1, PlineVertex::new(1.0, 2.0, 0.3));

    polyline.add(0.5, 0.5, 0.5);
    assert_eq!(polyline.iter_segments().size_hint(), (3, Some(3)));
    let three_seg = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(three_seg.len(), 3);
    assert_eq!(three_seg[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(three_seg[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(three_seg[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(three_seg[1].1, PlineVertex::new(0.5, 0.5, 0.5));
    assert_eq!(three_seg[2].0, PlineVertex::new(0.5, 0.5, 0.5));
    assert_eq!(three_seg[2].1, PlineVertex::new(1.0, 2.0, 0.3));

    polyline.set_is_closed(false);
    assert_eq!(polyline.iter_segments().size_hint(), (2, Some(2)));
    let two_seg_open = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(two_seg_open.len(), 2);
    assert_eq!(two_seg_open[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(two_seg_open[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg_open[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg_open[1].1, PlineVertex::new(0.5, 0.5, 0.5));
}

#[test]
fn iter_segment_indexes() {
    let mut polyline = Polyline::<f64>::new();
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segment_indexes().collect::<Vec<_>>().len(), 0);

    polyline.add(1.0, 2.0, 0.3);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segment_indexes().collect::<Vec<_>>().len(), 0);

    polyline.add(4.0, 5.0, 0.6);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (1, Some(1)));
    let one_seg = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(one_seg, vec![(0, 1)]);

    polyline.set_is_closed(true);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (2, Some(2)));
    let two_seg = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(two_seg, vec![(0, 1), (1, 0)]);

    polyline.add(0.5, 0.5, 0.5);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (3, Some(3)));
    let three_seg = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(three_seg, vec![(0, 1), (1, 2), (2, 0)]);

    polyline.set_is_closed(false);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (2, Some(2)));
    let two_seg_open = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(two_seg_open, vec![(0, 1), (1, 2)]);
}

#[test]
fn invert_direction_mut() {
    let mut polyline = Polyline::new_closed();
    polyline.add(0.0, 0.0, 0.1);
    polyline.add(2.0, 0.0, 0.2);
    polyline.add(2.0, 2.0, 0.3);
    polyline.add(0.0, 2.0, 0.4);

    polyline.invert_direction_mut();

    assert_fuzzy_eq!(polyline[0], PlineVertex::new(0.0, 2.0, -0.3));
    assert_fuzzy_eq!(polyline[1], PlineVertex::new(2.0, 2.0, -0.2));
    assert_fuzzy_eq!(polyline[2], PlineVertex::new(2.0, 0.0, -0.1));
    assert_fuzzy_eq!(polyline[3], PlineVertex::new(0.0, 0.0, -0.4));
}

#[test]
fn remove_repeat() {
    {
        // empty polyline
        let polyline = Polyline::new_closed();
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(result.is_none());
    }

    {
        // single vertex
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(result.is_none());
    }

    {
        // two repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // two repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // no repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(result.is_none());
    }

    {
        // no repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(4.0, 3.0, 1.0);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(result.is_none());
    }

    {
        // last repeats position on first for closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
    }

    {
        // last repeats position on first for open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(result.is_none());
    }

    {
        // catches case where prev position is updated even when vertex is skipped causing the end
        // result to actually have a repeat position
        let mut polyline = Polyline::new();
        polyline.add(149.75759744152376, 2753.341034622115, 0.0);
        polyline.add(
            149.75761269666256,
            2753.341034955893,
            -0.000000016806842584315973,
        );
        polyline.add(
            149.75760725254852,
            2753.341034836777,
            -0.000000026349436410555433,
        );
        polyline.add(
            149.75759871737387,
            2753.3410346500286,
            -0.0000000059965514775939255,
        );
        polyline.add(
            149.7576044186626,
            2753.341034774772,
            -0.000000017257169693252198,
        );
        polyline.add(
            149.7576208261107,
            2753.3410351337648,
            -0.00000001907759705765955,
        );
        polyline.add(
            149.75762700577317,
            2753.3410352689743,
            -0.0024145466234173404,
        );
        polyline.add(
            176.35224446582103,
            2753.7944419559553,
            -0.000000003667288472897212,
        );
        polyline.add(176.35224565393378, 2753.7944419704727, 0.0);
        polyline.add(176.35227673059205, 2753.794442350188, 0.0);
        polyline.add(176.35229710705553, 2753.794442599162, 0.0);

        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 7);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(
            result[0],
            PlineVertex::new(149.75759744152376, 2753.341034622115, 0.0)
        );
        assert_fuzzy_eq!(
            result[1],
            PlineVertex::new(
                149.75761269666256,
                2753.341034955893,
                -0.000000026349436410555433
            )
        );
        assert_fuzzy_eq!(
            result[2],
            PlineVertex::new(
                149.75759871737387,
                2753.3410346500286,
                -0.000000017257169693252198
            )
        );
        assert_fuzzy_eq!(
            result[3],
            PlineVertex::new(
                149.7576208261107,
                2753.3410351337648,
                -0.0024145466234173404
            )
        );
        assert_fuzzy_eq!(
            result[4],
            PlineVertex::new(176.35224446582103, 2753.7944419559553, 0.0)
        );
        assert_fuzzy_eq!(
            result[5],
            PlineVertex::new(176.35227673059205, 2753.794442350188, 0.0)
        );
        assert_fuzzy_eq!(
            result[6],
            PlineVertex::new(176.35229710705553, 2753.794442599162, 0.0)
        );
    }
}

#[test]
fn remove_redundant_removes_repeat_pos() {
    {
        // empty polyline
        let polyline = Polyline::new_closed();
        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // single vertex
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // two repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // two repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // no repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // no repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(4.0, 3.0, 1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // last repeats position on first for closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
    }

    {
        // last repeats position on first for open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // catches case where prev position is updated even when vertex is skipped causing the end
        // result to actually have a repeat position
        let mut polyline = Polyline::new();
        polyline.add(149.75759744152376, 2753.341034622115, 0.0);
        polyline.add(
            149.75761269666256,
            2753.341034955893,
            -0.000000016806842584315973,
        );
        polyline.add(
            149.75760725254852,
            2753.341034836777,
            -0.000000026349436410555433,
        );
        polyline.add(
            149.75759871737387,
            2753.3410346500286,
            -0.0000000059965514775939255,
        );
        polyline.add(
            149.7576044186626,
            2753.341034774772,
            -0.000000017257169693252198,
        );
        polyline.add(
            149.7576208261107,
            2753.3410351337648,
            -0.00000001907759705765955,
        );
        polyline.add(
            149.75762700577317,
            2753.3410352689743,
            -0.0024145466234173404,
        );
        polyline.add(
            176.35224446582103,
            2753.7944419559553,
            -0.000000003667288472897212,
        );
        polyline.add(176.35224565393378, 2753.7944419704727, 0.0);
        polyline.add(176.35227673059205, 2753.794442350188, 0.0);
        polyline.add(176.35229710705553, 2753.794442599162, 0.0);

        let result = polyline
            .remove_repeat_pos(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 7);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(
            result[0],
            PlineVertex::new(149.75759744152376, 2753.341034622115, 0.0)
        );
        assert_fuzzy_eq!(
            result[1],
            PlineVertex::new(
                149.75761269666256,
                2753.341034955893,
                -0.000000026349436410555433
            )
        );
        assert_fuzzy_eq!(
            result[2],
            PlineVertex::new(
                149.75759871737387,
                2753.3410346500286,
                -0.000000017257169693252198
            )
        );
        assert_fuzzy_eq!(
            result[3],
            PlineVertex::new(
                149.7576208261107,
                2753.3410351337648,
                -0.0024145466234173404
            )
        );
        assert_fuzzy_eq!(
            result[4],
            PlineVertex::new(176.35224446582103, 2753.7944419559553, 0.0)
        );
        assert_fuzzy_eq!(
            result[5],
            PlineVertex::new(176.35227673059205, 2753.794442350188, 0.0)
        );
        assert_fuzzy_eq!(
            result[6],
            PlineVertex::new(176.35229710705553, 2753.794442599162, 0.0)
        );
    }
}

#[test]
fn remove_redundant() {
    {
        // redundant point on line and repeat position
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(4.0, 4.0, 0.0);
        polyline.add(2.0, 4.0, 0.0);
        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 3);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(4.0, 4.0, 0.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(2.0, 4.0, 0.0));
    }

    {
        // self intersecting points along line (collinear but opposing direction, points should not
        // be removed)
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(2.5, 2.5, 0.0);
        polyline.add(4.0, 4.0, 0.0);
        polyline.add(2.0, 4.0, 0.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // simple counter clockwise circle with extra vertex along one arc
        let bulge = (std::f64::consts::FRAC_PI_2 / 4.0).tan();
        let mut polyline = Polyline::new_closed();
        polyline.add(0.0, 0.0, -bulge);
        polyline.add(1.0, 1.0, -bulge);
        polyline.add(2.0, 0.0, -1.0);
        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(0.0, 0.0, -1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(2.0, 0.0, -1.0));
    }

    {
        // arcs along greater arc
        let radius = 5.0;
        let max_angle = std::f64::consts::FRAC_PI_2;
        let count = 4;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);

        let mut polyline = Polyline::new();
        (0..=count)
            .map(|i| (i as f64) * sub_angle)
            .for_each(|angle| polyline.add(radius * angle.cos(), radius * angle.sin(), bulge));

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(
            result[0],
            PlineVertex::new(radius, 0.0, bulge_from_angle(max_angle))
        );
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.0, radius, bulge));
    }

    {
        // arcs along circle
        let radius = 5.0;
        let max_angle = std::f64::consts::TAU;
        let count = 10;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);

        let mut polyline = Polyline::new_closed();
        (0..count)
            .map(|i| (i as f64) * sub_angle)
            .for_each(|angle| polyline.add(radius * angle.cos(), radius * angle.sin(), bulge));

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(radius, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(-radius, 0.0, 1.0));
    }

    {
        // arcs along circle open polyline
        let radius = 5.0;
        let max_angle = std::f64::consts::TAU;
        let count = 10;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);

        let mut polyline = Polyline::new();
        (0..=count)
            .map(|i| (i as f64) * sub_angle)
            .for_each(|angle| polyline.add(radius * angle.cos(), radius * angle.sin(), bulge));

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(radius, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(-radius, 0.0, 1.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(radius, 0.0, bulge));
    }

    {
        // already minimum circle
        let radius = 5.0;

        let mut polyline = Polyline::new_closed();
        polyline.add(0.0, -radius, 1.0);
        polyline.add(0.0, radius, 1.0);

        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // closed half circle with arc that causes first vertex to be redundant
        let radius = 5.0;

        let bulge = bulge_from_angle(-std::f64::consts::FRAC_PI_2);

        let mut polyline = Polyline::new_closed();
        polyline.add(0.0, radius, bulge);
        polyline.add(radius, 0.0, 0.0);
        polyline.add(-radius, 0.0, bulge);

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-radius, 0.0, -1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(radius, 0.0, 0.0));
    }

    {
        // open polyline with bulge values that would cause first vertex to be redundant if
        // polyline were closed
        let radius = 5.0;

        let bulge = bulge_from_angle(-std::f64::consts::FRAC_PI_2);

        let mut polyline = Polyline::new();
        polyline.add(0.0, radius, bulge);
        polyline.add(radius, 0.0, 0.0);
        polyline.add(-radius, 0.0, bulge);

        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // closed path with redundant first vertex point along line
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(3.0, -2.0, 0.0);
        polyline.add(-2.0, -2.0, 0.0);
        polyline.add(-1.0, -1.0, 0.0);

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 3);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-2.0, -2.0, 0.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(3.0, -2.0, 0.0));
    }

    {
        // open polyline with values that would cause first vertex to be redundant due to being
        // collinear if polyline were closed
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(3.0, -2.0, 0.0);
        polyline.add(-2.0, -2.0, 0.0);
        polyline.add(-1.0, -1.0, 0.0);

        let result = polyline.remove_redundant(1e-5);
        assert!(result.is_none());
    }

    {
        // circle defined by 4 vertexes
        let bulge = (PI / 8.0).tan();
        let mut polyline = Polyline::new_closed();
        polyline.add(-0.5, 0.0, bulge);
        polyline.add(0.0, -0.5, bulge);
        polyline.add(0.5, 0.0, bulge);
        polyline.add(0.0, 0.5, bulge);

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-0.5, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.5, 0.0, 1.0));
    }

    {
        // rounded rectangle collapsed into circle
        let bulge = (PI / 8.0).tan();
        let mut polyline = Polyline::new_closed();
        polyline.add(-0.5, 0.0, bulge);
        polyline.add(0.0, -0.5, 0.0);
        polyline.add(0.0, -0.5, bulge);
        polyline.add(0.5, 0.0, 0.0);
        polyline.add(0.5, 0.0, bulge);
        polyline.add(0.0, 0.5, 0.0);
        polyline.add(0.0, 0.5, bulge);
        polyline.add(-0.5, 0.0, 0.0);

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-0.5, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.5, 0.0, 1.0));
    }

    {
        // rounded rectangle collapsed into circle shifted vertex positions
        let bulge = (PI / 8.0).tan();
        let mut polyline = Polyline::new_closed();
        polyline.add(-0.5, 0.0, 0.0);
        polyline.add(-0.5, 0.0, bulge);
        polyline.add(0.0, -0.5, 0.0);
        polyline.add(0.0, -0.5, bulge);
        polyline.add(0.5, 0.0, 0.0);
        polyline.add(0.5, 0.0, bulge);
        polyline.add(0.0, 0.5, 0.0);
        polyline.add(0.0, 0.5, bulge);

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-0.5, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.5, 0.0, 1.0));
    }

    {
        // rounded rectangle collapsed into circle (but kept as open polyline)
        let bulge = (PI / 8.0).tan();
        let mut polyline = Polyline::new();
        polyline.add(-0.5, 0.0, bulge);
        polyline.add(0.0, -0.5, 0.0);
        polyline.add(0.0, -0.5, bulge);
        polyline.add(0.5, 0.0, 0.0);
        polyline.add(0.5, 0.0, bulge);
        polyline.add(0.0, 0.5, 0.0);
        polyline.add(0.0, 0.5, bulge);
        polyline.add(-0.5, 0.0, 0.0);

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-0.5, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.5, 0.0, 1.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(-0.5, 0.0, 0.0));
    }

    {
        // rounded rectangle collapsed into circle with many repeat vertex positions
        let bulge = (PI / 8.0).tan();
        let mut polyline = Polyline::new_closed();
        polyline.add(-0.5, 0.0, 0.0);
        polyline.add(-0.5, 0.0, 0.0);
        polyline.add(-0.5, 0.0, 0.0);
        polyline.add(-0.5, 0.0, bulge);
        polyline.add(-0.5, 0.0, bulge);
        polyline.add(-0.5, 0.0, bulge);
        polyline.add(0.0, -0.5, 0.0);
        polyline.add(0.0, -0.5, 0.0);
        polyline.add(0.0, -0.5, 0.0);
        polyline.add(0.0, -0.5, bulge);
        polyline.add(0.5, 0.0, 0.0);
        polyline.add(0.5, 0.0, bulge);
        polyline.add(0.0, 0.5, 0.0);
        polyline.add(0.0, 0.5, bulge);
        polyline.add(0.0, 0.5, bulge);
        polyline.add(0.0, 0.5, bulge);

        let result = polyline
            .remove_redundant(1e-5)
            .expect("vertexes to be removed");
        assert_eq!(result.vertex_count(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-0.5, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.5, 0.0, 1.0));
    }

    {
        // n equal points
        let mut polyline1 = Polyline::with_capacity(3, false);
        polyline1.add(0.0, 0.0, 0.0);
        polyline1.add(0.0, 0.0, 0.0);
        polyline1.add(0.0, 0.0, 0.0);
        let mut polyline2 = Polyline::with_capacity(3, false);
        polyline2.add(1.0, 1.0, 0.0);
        polyline2.add(1.0, 1.0, 0.0);
        polyline2.add(1.0, 1.0, 1.0);
        let mut polyline3 = Polyline::with_capacity(2, false);
        polyline3.add(2.0, 2.0, 0.0);
        polyline3.add(2.0, 2.0, 1.0);

        let r1 = polyline1.remove_redundant(1e-5).unwrap();
        let r2 = polyline2.remove_redundant(1e-5).unwrap();
        let r3 = polyline3.remove_redundant(1e-5).unwrap();
        assert_eq!(r1.vertex_count(), 1);
        assert_eq!(r2.vertex_count(), 1);
        assert_eq!(r3.vertex_count(), 1);
        assert_fuzzy_eq!(r1[0], PlineVertex::new(0.0, 0.0, 0.0));
        assert_fuzzy_eq!(r2[0], PlineVertex::new(1.0, 1.0, 1.0));
        assert_fuzzy_eq!(r3[0], PlineVertex::new(2.0, 2.0, 1.0));
    }
}

#[test]
fn rotate_start() {
    {
        // empty polyline
        let polyline = Polyline::new_closed();
        assert!(
            polyline
                .rotate_start(0, Vector2::new(0.0, 0.0), 1e-5)
                .is_none()
        );
    }

    {
        // single vertex polyline
        let polyline = pline_closed![(1.0, 0.0, 0.0)];
        assert!(
            polyline
                .rotate_start(0, Vector2::new(0.0, 0.0), 1e-5)
                .is_none()
        );
    }

    {
        // open polyline
        let polyline = pline_open![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.5),
            (1.0, 1.0, 0.2),
            (0.0, 1.0, -0.1),
        ];
        assert!(
            polyline
                .rotate_start(0, Vector2::new(0.0, 0.0), 1e-5)
                .is_none()
        );
    }

    {
        // no change
        let polyline = pline_closed![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.5),
            (1.0, 1.0, 0.2),
            (0.0, 1.0, -0.1),
        ];

        let rot_no_change = polyline
            .rotate_start(0, Vector2::new(0.0, 0.0), 1e-5)
            .unwrap();
        assert!(rot_no_change.fuzzy_eq(&polyline));
    }

    {
        // end becomes start
        let polyline = pline_closed![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.5),
            (1.0, 1.0, 0.2),
            (0.0, 1.0, -0.1),
        ];

        let rot_end_is_start = polyline
            .rotate_start(polyline.vertex_count() - 1, Vector2::new(0.0, 1.0), 1e-5)
            .unwrap();

        let expected_end_as_start = pline_closed![
            (0.0, 1.0, -0.1),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.5),
            (1.0, 1.0, 0.2),
        ];

        assert_fuzzy_eq!(&rot_end_is_start, &expected_end_as_start);
    }

    {
        // split in middle of line segment
        let polyline = pline_closed![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
        ];

        let rot = polyline
            .rotate_start(0, Vector2::new(0.5, 0.0), 1e-5)
            .unwrap();
        let expected_rot = pline_closed![
            (0.5, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
        ];
        assert_fuzzy_eq!(rot, &expected_rot);
    }

    {
        // split in middle of arc segment
        let polyline = pline_closed![
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 1.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
        ];

        let rot = polyline
            .rotate_start(1, Vector2::new(1.5, 0.5), 1e-5)
            .unwrap();

        let expected_rot = pline_closed![
            (1.5, 0.5, bulge_from_angle(std::f64::consts::FRAC_PI_2)),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
            (1.0, 0.0, bulge_from_angle(std::f64::consts::FRAC_PI_2)),
        ];
        assert_fuzzy_eq!(rot, &expected_rot);
    }
}

#[test]
fn area() {
    {
        let mut circle = Polyline::new_closed();
        circle.add(0.0, 0.0, 1.0);
        circle.add(2.0, 0.0, 1.0);
        assert_fuzzy_eq!(circle.area(), PI);
        circle.invert_direction_mut();
        assert_fuzzy_eq!(circle.area(), -PI);
    }

    {
        let mut half_circle = Polyline::new_closed();
        half_circle.add(0.0, 0.0, -1.0);
        half_circle.add(2.0, 0.0, 0.0);
        assert_fuzzy_eq!(half_circle.area(), -0.5 * PI);
        half_circle.invert_direction_mut();
        assert_fuzzy_eq!(half_circle.area(), 0.5 * PI);
    }

    {
        let mut rectangle = Polyline::new_closed();
        rectangle.add(0.0, 0.0, 0.0);
        rectangle.add(3.0, 0.0, 0.0);
        rectangle.add(3.0, 2.0, 0.0);
        rectangle.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(rectangle.area(), 6.0);
        rectangle.invert_direction_mut();
        assert_fuzzy_eq!(rectangle.area(), -6.0);
    }

    {
        let mut open_polyline = Polyline::new();
        open_polyline.add(0.0, 0.0, 0.0);
        open_polyline.add(2.0, 0.0, 0.0);
        open_polyline.add(2.0, 2.0, 0.0);
        open_polyline.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(open_polyline.area(), 0.0);
        open_polyline.invert_direction_mut();
        assert_fuzzy_eq!(open_polyline.area(), 0.0);
    }

    {
        let empty_open_polyline = Polyline::<f64>::new();
        assert_fuzzy_eq!(empty_open_polyline.area(), 0.0);
    }

    {
        let empty_closed_polyline = Polyline::<f64>::new_closed();
        assert_fuzzy_eq!(empty_closed_polyline.area(), 0.0);
    }

    {
        let mut one_vertex_open_polyline = Polyline::<f64>::new();
        one_vertex_open_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_open_polyline.area(), 0.0);
    }

    {
        let mut one_vertex_closed_polyline = Polyline::<f64>::new_closed();
        one_vertex_closed_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_closed_polyline.area(), 0.0);
    }
}

#[test]
fn path_length() {
    {
        let empty_open_polyline = Polyline::<f64>::new();
        assert_fuzzy_eq!(empty_open_polyline.path_length(), 0.0);
    }

    {
        let empty_closed_polyline = Polyline::<f64>::new();
        assert_fuzzy_eq!(empty_closed_polyline.path_length(), 0.0);
    }

    {
        let mut one_vertex_open_polyline = Polyline::<f64>::new();
        one_vertex_open_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_open_polyline.path_length(), 0.0);
    }

    {
        let mut one_vertex_closed_polyline = Polyline::<f64>::new_closed();
        one_vertex_closed_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_closed_polyline.path_length(), 0.0);
    }

    {
        let mut circle = Polyline::new_closed();
        circle.add(0.0, 0.0, 1.0);
        circle.add(2.0, 0.0, 1.0);
        assert_fuzzy_eq!(circle.path_length(), TAU);
        circle.invert_direction_mut();
        assert_fuzzy_eq!(circle.path_length(), TAU);
    }

    {
        let mut half_circle = Polyline::new_closed();
        half_circle.add(0.0, 0.0, -1.0);
        half_circle.add(2.0, 0.0, 0.0);
        assert_fuzzy_eq!(half_circle.path_length(), PI + 2.0);
        half_circle.invert_direction_mut();
        assert_fuzzy_eq!(half_circle.path_length(), PI + 2.0);
    }

    {
        let mut rectangle = Polyline::new_closed();
        rectangle.add(0.0, 0.0, 0.0);
        rectangle.add(3.0, 0.0, 0.0);
        rectangle.add(3.0, 2.0, 0.0);
        rectangle.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(rectangle.path_length(), 10.0);
        rectangle.invert_direction_mut();
        assert_fuzzy_eq!(rectangle.path_length(), 10.0);
    }

    {
        let mut open_polyline = Polyline::new();
        open_polyline.add(0.0, 0.0, 0.0);
        open_polyline.add(3.0, 0.0, 0.0);
        open_polyline.add(3.0, 2.0, 0.0);
        open_polyline.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(open_polyline.path_length(), 8.0);
        open_polyline.invert_direction_mut();
        assert_fuzzy_eq!(open_polyline.path_length(), 8.0);
    }
}

#[test]
fn extents() {
    {
        let empty_pline = Polyline::<f64>::new();
        debug_assert_eq!(empty_pline.extents(), None);
    }

    {
        let mut one_vertex_pline = Polyline::new();
        one_vertex_pline.add(1.0, 1.0, 0.0);
        debug_assert_eq!(one_vertex_pline.extents(), None);
    }

    {
        // basic line
        let mut pline = pline_open![(-2.0, -1.0, 0.0), (3.0, 4.0, 0.0)];
        let extents = pline.extents().unwrap();
        assert_eq!(extents.min_x, -2.0);
        assert_eq!(extents.min_y, -1.0);
        assert_eq!(extents.max_x, 3.0);
        assert_eq!(extents.max_y, 4.0);

        pline.set_is_closed(true);
        let extents = pline.extents().unwrap();
        assert_eq!(extents.min_x, -2.0);
        assert_eq!(extents.min_y, -1.0);
        assert_eq!(extents.max_x, 3.0);
        assert_eq!(extents.max_y, 4.0);
    }

    {
        // axis aligned circle
        let mut pline = pline_closed![(-1.0, 0.0, 1.0), (1.0, 0.0, 1.0),];
        let extents = pline.extents().unwrap();
        assert_eq!(extents.min_x, -1.0);
        assert_eq!(extents.min_y, -1.0);
        assert_eq!(extents.max_x, 1.0);
        assert_eq!(extents.max_y, 1.0);

        // half circle
        pline.set_is_closed(false);
        let extents = pline.extents().unwrap();
        assert_eq!(extents.min_x, -1.0);
        assert_eq!(extents.min_y, -1.0);
        assert_eq!(extents.max_x, 1.0);
        assert_eq!(extents.max_y, 0.0);
    }

    {
        // axis aligned circle
        let mut pline = pline_closed![(0.0, -1.0, 1.0), (0.0, 1.0, 1.0),];
        let extents = pline.extents().unwrap();
        assert_eq!(extents.min_x, -1.0);
        assert_eq!(extents.min_y, -1.0);
        assert_eq!(extents.max_x, 1.0);
        assert_eq!(extents.max_y, 1.0);

        // half circle
        pline.set_is_closed(false);
        let extents = pline.extents().unwrap();
        assert_eq!(extents.min_x, 0.0);
        assert_eq!(extents.min_y, -1.0);
        assert_eq!(extents.max_x, 1.0);
        assert_eq!(extents.max_y, 1.0);
    }

    {
        // handles repeat position vertexes
        let pline = pline_closed![
            (-1.0, 0.0, 0.0),
            (-1.0, 0.0, 1.0),
            (-1.0, 0.0, 0.0),
            (-1.0, 0.0, 1.0),
            (1.0, 0.0, 1.0),
            (1.0, 0.0, 1.0)
        ];
        let extents = pline.extents().unwrap();
        assert_eq!(extents.min_x, -1.0);
        assert_eq!(extents.min_y, -1.0);
        assert_eq!(extents.max_x, 1.0);
        assert_eq!(extents.max_y, 1.0);
    }
}

macro_rules! assert_path_length_result_eq {
    ($left:expr, $right:expr) => {
        match ($left, $right) {
            (Ok((index1, point1)), Ok((index2, point2)))
                if index1 == index2 && point1.fuzzy_eq_eps(point2, 1e-5) => {}
            (Err(path_length1), Err(path_length2))
                if path_length1.fuzzy_eq_eps(path_length2, 1e-5) => {}
            _ => panic!(
                "result cases do not match: left: {:?}, right: {:?}",
                $left, $right
            ),
        }
    };
}

#[test]
fn find_point_at_path_length() {
    let pline = pline_closed![
        (0.0, 0.0, 1.0),
        (1.0, 0.0, -1.0),
        (1.0, 1.0, 0.0),
        (1.0, 2.0, 0.0)
    ];
    let pline_path_length = pline.path_length();

    // 0 path length (point at very start)
    {
        let r = pline.find_point_at_path_length(0.0);
        let expected = Ok((0, Vector2::new(0.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // total path length (point at very end)
    {
        let r = pline.find_point_at_path_length(pline_path_length);
        let expected = Ok((3, Vector2::new(0.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // negative path length
    {
        let r = pline.find_point_at_path_length(-1.0);
        let expected = Ok((0, Vector2::new(0.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // target path length greater than total
    {
        let r = pline.find_point_at_path_length(pline_path_length + 1.0);
        let expected = Err(pline_path_length);
        assert_path_length_result_eq!(r, expected);
    }

    // half path length of first seg
    {
        let target_path_length = seg_length(pline[0], pline[1]) / 2.0;
        let r = pline.find_point_at_path_length(target_path_length);
        let expected = Ok((0, Vector2::new(0.5, -0.5)));
        assert_path_length_result_eq!(r, expected);
    }

    // full path length of first seg
    {
        let target_path_length = seg_length(pline[0], pline[1]);
        let r = pline.find_point_at_path_length(target_path_length);
        let expected = Ok((0, Vector2::new(1.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // half path length into second seg
    {
        let target_path_length =
            seg_length(pline[0], pline[1]) + seg_length(pline[1], pline[2]) / 2.0;
        let r = pline.find_point_at_path_length(target_path_length);
        let expected = Ok((1, Vector2::new(0.5, 0.5)));
        assert_path_length_result_eq!(r, expected);
    }

    // half path length into third seg
    {
        let target_path_length = seg_length(pline[0], pline[1])
            + seg_length(pline[1], pline[2])
            + seg_length(pline[2], pline[3]) / 2.0;
        let r = pline.find_point_at_path_length(target_path_length);
        let expected = Ok((2, Vector2::new(1.0, 1.5)));
        assert_path_length_result_eq!(r, expected);
    }

    // sub slice tests (mostly to validate segment index offset)
    let sub_slice =
        PlineViewData::from_slice_points(&pline, pline[2].pos(), 2, pline[3].pos(), 3, 1e-5)
            .unwrap();
    let sub_slice_length = seg_length(pline[2], pline[3]);
    let view = sub_slice.view(&pline);

    // 0 path length (point at very start)
    {
        let r = view.find_point_at_path_length(0.0);
        let expected = Ok((0, Vector2::new(1.0, 1.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // total path length (point at very end)
    {
        let r = view.find_point_at_path_length(sub_slice_length);
        let expected = Ok((0, Vector2::new(1.0, 2.0)));
        assert_path_length_result_eq!(r, expected);
    }
}

#[test]
fn create_from_remove_repeat() {
    let pline = pline_closed![
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0),
    ];

    let result = Polyline::create_from_remove_repeat(&pline, 1e-5);

    let pline = pline_closed![
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
    ];

    assert!(result.fuzzy_eq(&pline));
}
