use cavalier_contours::{
    core::math::Vector2,
    pline_closed,
    polyline::{PlineSource, PlineSourceMut},
};

#[test]
fn point_and_circle() {
    let pl = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)];

    // inside the circle
    {
        let pt = Vector2::new(0.5, 0.0);
        assert_eq!(pl.winding_number(pt), 1);

        let mut pl = pl.clone();
        pl.invert_direction_mut();
        assert_eq!(pl.winding_number(pt), -1);
    }

    // outside the circle
    {
        let pt = Vector2::new(2.0, 0.0);
        assert_eq!(pl.winding_number(pt), 0);

        let mut pl = pl;
        pl.invert_direction_mut();
        assert_eq!(pl.winding_number(pt), 0);
    }
}

#[test]
fn point_and_rectangle() {
    let pl = pline_closed![
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 4.0, 0.0),
        (0.0, 4.0, 0.0),
    ];

    // inside the rectangle
    {
        let pt = Vector2::new(1.0, 1.0);
        assert_eq!(pl.winding_number(pt), 1);

        let mut pl = pl.clone();
        pl.invert_direction_mut();
        assert_eq!(pl.winding_number(pt), -1);
    }

    // outside the rectangle
    {
        let pt = Vector2::new(-1.0, 1.0);
        assert_eq!(pl.winding_number(pt), 0);

        let mut pl = pl;
        pl.invert_direction_mut();
        assert_eq!(pl.winding_number(pt), 0);
    }
}

#[test]
fn multiple_windings() {
    // path forming circle overlapping itself
    let pl = pline_closed![
        (0.0, 0.0, 1.0),
        (2.0, 0.0, 1.0),
        (0.0, 0.0, 1.0),
        (2.0, 0.0, 1.0),
    ];

    // inside the circle
    {
        let pt = Vector2::new(0.5, 0.0);
        assert_eq!(pl.winding_number(pt), 2);

        let mut pl = pl.clone();
        pl.invert_direction_mut();
        assert_eq!(pl.winding_number(pt), -2);
    }

    // outside the circle
    {
        let pt = Vector2::new(2.0, 0.0);
        assert_eq!(pl.winding_number(pt), 0);

        let mut pl = pl;
        pl.invert_direction_mut();
        assert_eq!(pl.winding_number(pt), 0);
    }
}

#[test]
fn point_outside_aligned_with_direction_vectors1() {
    let pl = pline_closed![
        (-10.0, 0.0, 1.0),
        (10.0, 0.0, 0.0),
        (20.0, 0.0, 0.0),
        (20.0, -10.0, 0.0),
        (-20.0, -10.0, 0.0),
        (-20.0, 0.0, 0.0)
    ];

    let pt = Vector2::zero();

    assert_eq!(pl.winding_number(pt), 0);
}

#[test]
fn point_outside_aligned_with_direction_vectors2() {
    let pl = pline_closed![
        (-5.51073e-15, -30.0, 0.269712),
        (26.0788, -14.8288, 0.0),
        (76.0788, 73.104, 0.12998),
        (80.0, 87.9329, 0.0),
        (80.0, 130.0, 0.0),
        (50.0, 130.0, 0.0),
        (50.0, 95.0, -0.414214),
        (40.0, 85.0, 0.0),
        (0.0, 85.0, 0.0),
    ];

    let pt = Vector2::new(-20.0, 85.0);

    assert_eq!(pl.winding_number(pt), 0);
}
