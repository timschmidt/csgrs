mod test_utils;

mod test_pline_contains {
    use cavalier_contours::polyline::PlineContainsResult::*;
    use cavalier_contours::polyline::*;
    use cavalier_contours::{pline_closed, pline_open};

    #[test]
    fn test_rectangle_contains_circle() {
        let rectangle = pline_closed![
            (-2.0, -2.0, 0.0),
            (2.0, -2.0, 0.0),
            (2.0, 2.0, 0.0),
            (-2.0, 2.0, 0.0),
        ];

        let circle = pline_closed![(-1.0, 0.0, 1.0), (1.0, 0.0, 1.0)];

        assert_eq!(rectangle.contains(&circle), Pline2InsidePline1);
        assert_eq!(circle.contains(&rectangle), Pline1InsidePline2);
    }

    #[test]
    fn test_rectangle_intersects_circle() {
        let rectangle = pline_closed![
            (-2.0, -2.0, 0.0),
            (0.5, -2.0, 0.0),
            (0.5, 2.0, 0.0),
            (-2.0, 2.0, 0.0),
        ];

        let circle = pline_closed![(-1.0, 0.0, 1.0), (1.0, 0.0, 1.0)];

        assert_eq!(rectangle.contains(&circle), Intersected);
        assert_eq!(circle.contains(&rectangle), Intersected);
    }

    #[test]
    fn test_disjoint() {
        let rectangle = pline_closed![
            (-2.0, -2.0, 0.0),
            (2.0, -2.0, 0.0),
            (2.0, 2.0, 0.0),
            (-2.0, 2.0, 0.0),
        ];

        let circle = pline_closed![(4.0, 0.0, 1.0), (5.0, 0.0, 1.0)];

        assert_eq!(rectangle.contains(&circle), Disjoint);
        assert_eq!(circle.contains(&rectangle), Disjoint);
    }

    #[test]
    fn test_copy() {
        let rectangle = pline_closed![
            (-2.0, -2.0, 0.0),
            (2.0, -2.0, 0.0),
            (2.0, 2.0, 0.0),
            (-2.0, 2.0, 0.0),
        ];

        assert_eq!(rectangle.contains(&rectangle.clone()), Intersected);
    }

    #[test]
    fn test_invalid() {
        let bad1 = pline_open![(0.0, 0.0, 0.0),];
        let bad2 = pline_open![(-2.0, -2.0, 0.0),];

        assert_eq!(bad1.contains(&bad2), InvalidInput);
        assert_eq!(bad2.contains(&bad1), InvalidInput);
    }

    #[test]
    fn test_self_intersect_scan() {
        let hourglass = pline_closed![
            (0.0, 2.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 1.0, 0.0),
            (1.0, 2.0, 0.0)
        ];
        assert!(hourglass.scan_for_self_intersect());
    }
}
