//! Geometry constructors used by parsers and checks.
//!
//! The module exposes a small, stable API over `geo` polygons. Submodules keep
//! Sketch/report conversion code separate from primitive polygon generation.

mod primitives;
mod sketch;
mod violations;

pub use primitives::{
    arc_line_polygons, circle_polygon, line_polygon, polygon_from_points, rect_polygon,
    transform_polygon,
};
pub use sketch::{empty_sketch, polygon_to_sketch, polygons_to_sketch};
pub use violations::multipolygon_to_shapes;

#[cfg(test)]
mod tests {
    use geo::{Area, Coord, LineString, MultiPolygon, Polygon};
    use proptest::prelude::*;

    use super::{
        arc_line_polygons, circle_polygon, empty_sketch, line_polygon, multipolygon_to_shapes,
        polygon_from_points, polygon_to_sketch, polygons_to_sketch, rect_polygon,
        transform_polygon,
    };
    use crate::LayerMetadata;

    const EPS: f64 = 1.0e-9;

    fn assert_close(left: f64, right: f64) {
        let tolerance = (left.abs().max(right.abs()).max(1.0)) * EPS;
        assert!(
            (left - right).abs() <= tolerance,
            "{left} was not within {tolerance} of {right}"
        );
    }

    fn assert_point_close(left: Coord<f64>, right: [f64; 2]) {
        assert_close(left.x, right[0]);
        assert_close(left.y, right[1]);
    }

    fn assert_ring_closed(polygon: &Polygon<f64>) {
        assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
        for hole in polygon.interiors() {
            assert_eq!(hole.0.first(), hole.0.last());
        }
    }

    fn distance(left: Coord<f64>, right: Coord<f64>) -> f64 {
        let dx = right.x - left.x;
        let dy = right.y - left.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn segment_length_sum(polygons: &[Polygon<f64>]) -> f64 {
        polygons
            .iter()
            .map(|polygon| distance(polygon.exterior().0[0], polygon.exterior().0[1]))
            .sum()
    }

    fn signed_area(polygon: &Polygon<f64>) -> f64 {
        polygon
            .exterior()
            .0
            .windows(2)
            .map(|window| window[0].x * window[1].y - window[1].x * window[0].y)
            .sum::<f64>()
            / 2.0
    }

    #[test]
    fn sketch_wrappers_preserve_metadata_and_geometry() {
        let square = polygon_from_points(vec![[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]);
        let triangle = polygon_from_points(vec![[10.0, 0.0], [11.0, 0.0], [10.0, 1.0]]);
        let single = polygon_to_sketch(
            square.clone(),
            Some(LayerMetadata {
                name: "single".to_string(),
            }),
        );
        let many = polygons_to_sketch(
            vec![square.clone(), triangle],
            Some(LayerMetadata {
                name: "many".to_string(),
            }),
        );
        let empty = empty_sketch(Some(LayerMetadata {
            name: "empty".to_string(),
        }));

        assert_eq!(single.metadata.as_ref().unwrap().name, "single");
        assert_eq!(many.metadata.as_ref().unwrap().name, "many");
        assert_eq!(empty.metadata.as_ref().unwrap().name, "empty");
        assert_eq!(single.to_multipolygon().0.len(), 1);
        assert_eq!(many.to_multipolygon().0.len(), 2);
        assert!(empty.to_multipolygon().0.is_empty());
        assert_close(
            single.to_multipolygon().unsigned_area(),
            square.unsigned_area(),
        );
    }

    #[test]
    fn sketch_wrappers_accept_empty_polygon_lists_without_losing_metadata() {
        let sketch = polygons_to_sketch(
            Vec::new(),
            Some(LayerMetadata {
                name: "empty multi".to_string(),
            }),
        );

        assert_eq!(sketch.metadata.as_ref().unwrap().name, "empty multi");
        assert!(sketch.to_multipolygon().0.is_empty());
    }

    #[test]
    fn line_polygon_rejects_degenerate_inputs() {
        assert!(line_polygon([0.0, 0.0], [0.0, 0.0], 1.0).is_none());
        assert!(line_polygon([0.0, 0.0], [1.0, 0.0], 0.0).is_none());
        assert!(line_polygon([0.0, 0.0], [1.0, 0.0], -1.0).is_none());
    }

    #[test]
    fn line_polygon_builds_expected_horizontal_rectangle() {
        let polygon = line_polygon([0.0, 0.0], [2.0, 0.0], 0.5).unwrap();

        assert_ring_closed(&polygon);
        assert_close(polygon.unsigned_area(), 1.0);
        assert_point_close(polygon.exterior().0[0], [0.0, 0.25]);
        assert_point_close(polygon.exterior().0[1], [2.0, 0.25]);
        assert_point_close(polygon.exterior().0[2], [2.0, -0.25]);
        assert_point_close(polygon.exterior().0[3], [0.0, -0.25]);
    }

    #[test]
    fn line_polygon_builds_expected_vertical_rectangle() {
        let polygon = line_polygon([1.0, 1.0], [1.0, 3.0], 0.4).unwrap();

        assert_ring_closed(&polygon);
        assert_close(polygon.unsigned_area(), 0.8);
        assert_point_close(polygon.exterior().0[0], [0.8, 1.0]);
        assert_point_close(polygon.exterior().0[1], [0.8, 3.0]);
        assert_point_close(polygon.exterior().0[2], [1.2, 3.0]);
        assert_point_close(polygon.exterior().0[3], [1.2, 1.0]);
    }

    #[test]
    fn line_polygon_area_is_length_times_width_for_diagonal_trace() {
        let polygon = line_polygon([-1.0, -1.0], [2.0, 3.0], 0.25).unwrap();

        assert_close(polygon.unsigned_area(), 5.0 * 0.25);
    }

    #[test]
    fn line_polygon_survives_tiny_nonzero_segments() {
        let polygon = line_polygon([1.0, -1.0], [1.0 + 1.0e-12, -1.0], 0.25).unwrap();

        assert_ring_closed(&polygon);
        assert!(polygon.unsigned_area().is_finite());
        assert!(
            polygon
                .exterior()
                .0
                .iter()
                .all(|coord| coord.x.is_finite() && coord.y.is_finite())
        );
    }

    #[test]
    fn reversed_line_polygon_covers_same_area() {
        let forward = line_polygon([0.0, 0.0], [2.0, 3.0], 0.7).unwrap();
        let reverse = line_polygon([2.0, 3.0], [0.0, 0.0], 0.7).unwrap();

        assert_close(forward.unsigned_area(), reverse.unsigned_area());
    }

    #[test]
    fn polygon_from_points_closes_open_ring() {
        let polygon = polygon_from_points(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]);

        assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
    }

    #[test]
    fn polygon_from_points_does_not_duplicate_closed_ring() {
        let polygon = polygon_from_points(vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]]);

        assert_eq!(polygon.exterior().0.len(), 4);
        assert_ring_closed(&polygon);
    }

    #[test]
    fn polygon_from_points_accepts_empty_and_single_point_inputs() {
        let empty = polygon_from_points(Vec::new());
        let single = polygon_from_points(vec![[2.0, 3.0]]);

        assert!(empty.exterior().0.is_empty());
        assert_eq!(single.exterior().0.len(), 1);
        assert_close(empty.unsigned_area(), 0.0);
        assert_close(single.unsigned_area(), 0.0);
    }

    #[test]
    fn circle_polygon_enforces_minimum_segment_count() {
        let polygon = circle_polygon([2.0, -3.0], 4.0, 3);

        assert_eq!(polygon.exterior().0.len(), 17);
        assert_ring_closed(&polygon);
        assert_point_close(polygon.exterior().0[0], [6.0, -3.0]);
        assert!(polygon.unsigned_area() < std::f64::consts::PI * 16.0);
    }

    #[test]
    fn circle_polygon_zero_radius_is_closed_zero_area_polygon() {
        let polygon = circle_polygon([2.0, -3.0], 0.0, 16);

        assert_ring_closed(&polygon);
        assert_close(polygon.unsigned_area(), 0.0);
        assert!(
            polygon
                .exterior()
                .0
                .iter()
                .all(|coord| coord.x == 2.0 && coord.y == -3.0)
        );
    }

    #[test]
    fn circle_polygon_negative_radius_flips_winding_but_not_area() {
        let positive = circle_polygon([0.0, 0.0], 2.0, 32);
        let negative = circle_polygon([0.0, 0.0], -2.0, 32);

        assert_ring_closed(&negative);
        assert_close(positive.unsigned_area(), negative.unsigned_area());
        assert_close(signed_area(&positive), signed_area(&negative));
        assert_point_close(negative.exterior().0[0], [-2.0, 0.0]);
    }

    #[test]
    fn rect_polygon_axis_aligned_corners_are_centered() {
        let polygon = rect_polygon([10.0, -5.0], [4.0, 2.0], 0.0);

        assert_ring_closed(&polygon);
        assert_close(polygon.unsigned_area(), 8.0);
        assert_point_close(polygon.exterior().0[0], [8.0, -6.0]);
        assert_point_close(polygon.exterior().0[1], [12.0, -6.0]);
        assert_point_close(polygon.exterior().0[2], [12.0, -4.0]);
        assert_point_close(polygon.exterior().0[3], [8.0, -4.0]);
    }

    #[test]
    fn rect_polygon_handles_right_angle_rotation() {
        let polygon = rect_polygon([1.0, 2.0], [4.0, 2.0], 90.0);

        assert_close(polygon.unsigned_area(), 8.0);
        assert_point_close(polygon.exterior().0[0], [2.0, 0.0]);
        assert_point_close(polygon.exterior().0[1], [2.0, 4.0]);
        assert_point_close(polygon.exterior().0[2], [0.0, 4.0]);
        assert_point_close(polygon.exterior().0[3], [0.0, 0.0]);
    }

    #[test]
    fn rect_polygon_zero_or_signed_dimensions_have_predictable_area() {
        let zero_width = rect_polygon([0.0, 0.0], [0.0, 2.0], 35.0);
        let signed_width = rect_polygon([0.0, 0.0], [-4.0, 2.0], 0.0);
        let signed_height = rect_polygon([0.0, 0.0], [4.0, -2.0], 0.0);

        assert_ring_closed(&zero_width);
        assert_close(zero_width.unsigned_area(), 0.0);
        assert_close(signed_width.unsigned_area(), 8.0);
        assert_close(signed_height.unsigned_area(), 8.0);
        assert!(signed_area(&signed_width).is_sign_negative());
        assert!(signed_area(&signed_height).is_sign_negative());
    }

    #[test]
    fn transform_polygon_moves_exterior_and_holes() {
        let polygon = Polygon::new(
            LineString(vec![
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 4.0, y: 0.0 },
                Coord { x: 4.0, y: 4.0 },
                Coord { x: 0.0, y: 4.0 },
                Coord { x: 0.0, y: 0.0 },
            ]),
            vec![LineString(vec![
                Coord { x: 1.0, y: 1.0 },
                Coord { x: 2.0, y: 1.0 },
                Coord { x: 2.0, y: 2.0 },
                Coord { x: 1.0, y: 2.0 },
                Coord { x: 1.0, y: 1.0 },
            ])],
        );

        let transformed = transform_polygon(&polygon, [10.0, 20.0], 90.0);

        assert_ring_closed(&transformed);
        assert_close(polygon.unsigned_area(), transformed.unsigned_area());
        assert_point_close(transformed.exterior().0[1], [10.0, 24.0]);
        assert_eq!(transformed.interiors().len(), 1);
        assert_point_close(transformed.interiors()[0].0[0], [9.0, 21.0]);
    }

    #[test]
    fn transform_polygon_identity_keeps_coordinates() {
        let polygon = rect_polygon([3.0, -4.0], [2.0, 6.0], 33.0);
        let transformed = transform_polygon(&polygon, [0.0, 0.0], 0.0);

        assert_eq!(polygon.exterior().0.len(), transformed.exterior().0.len());
        for (left, right) in polygon.exterior().0.iter().zip(&transformed.exterior().0) {
            assert_point_close(*right, [left.x, left.y]);
        }
    }

    #[test]
    fn transform_polygon_full_rotation_only_translates() {
        let polygon = rect_polygon([0.0, 0.0], [2.0, 4.0], 0.0);
        let transformed = transform_polygon(&polygon, [5.0, -7.0], 720.0);

        assert_point_close(transformed.exterior().0[0], [4.0, -9.0]);
        assert_point_close(transformed.exterior().0[1], [6.0, -9.0]);
        assert_point_close(transformed.exterior().0[2], [6.0, -5.0]);
        assert_point_close(transformed.exterior().0[3], [4.0, -5.0]);
    }

    #[test]
    fn arc_with_zero_angle_produces_no_valid_segments() {
        let polygons = arc_line_polygons([0.0, 0.0], [1.0, 0.0], 0.0, 0.1, 8);

        assert!(polygons.is_empty());
    }

    #[test]
    fn arc_line_polygons_enforces_minimum_segment_count() {
        let polygons = arc_line_polygons([0.0, 0.0], [1.0, 0.0], 90.0, 0.1, 1);

        assert_eq!(polygons.len(), 4);
        assert!(polygons.iter().all(|polygon| {
            polygon.exterior().0.len() == 5
                && polygon.exterior().0.first() == polygon.exterior().0.last()
        }));
    }

    #[test]
    fn arc_line_polygons_have_expected_chord_area_sum() {
        let polygons = arc_line_polygons([0.0, 0.0], [1.0, 0.0], 180.0, 0.25, 8);
        let expected_area = segment_length_sum(&polygons) * 0.25;
        let total_area = polygons.iter().map(Polygon::unsigned_area).sum::<f64>();

        assert_eq!(polygons.len(), 8);
        assert_close(total_area, expected_area);
    }

    #[test]
    fn arc_line_polygons_support_clockwise_and_full_circle_arcs() {
        let clockwise = arc_line_polygons([0.0, 0.0], [1.0, 0.0], -90.0, 0.1, 4);
        let full_circle = arc_line_polygons([0.0, 0.0], [1.0, 0.0], 360.0, 0.1, 16);

        assert_eq!(clockwise.len(), 4);
        assert_eq!(full_circle.len(), 16);
        assert!(clockwise[0].exterior().0[0].x > 1.0);
        assert!(clockwise[0].exterior().0[0].y < 0.0);
        let final_radius = distance(full_circle[15].exterior().0[1], Coord { x: 0.0, y: 0.0 });
        assert!(final_radius > 0.94 && final_radius < 1.06);
        assert!(clockwise.iter().chain(&full_circle).all(|polygon| {
            polygon.unsigned_area().is_finite()
                && polygon.exterior().0.first() == polygon.exterior().0.last()
        }));
    }

    #[test]
    fn arc_line_polygons_reject_invalid_width_or_zero_radius() {
        assert!(arc_line_polygons([0.0, 0.0], [1.0, 0.0], 90.0, 0.0, 8).is_empty());
        assert!(arc_line_polygons([0.0, 0.0], [0.0, 0.0], 90.0, 0.1, 8).is_empty());
    }

    #[test]
    fn multipolygon_to_shapes_filters_by_strict_min_area_and_preserves_holes() {
        let small = polygon_from_points(vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
        let with_hole = Polygon::new(
            LineString(vec![
                Coord { x: 0.0, y: 0.0 },
                Coord { x: 4.0, y: 0.0 },
                Coord { x: 4.0, y: 4.0 },
                Coord { x: 0.0, y: 4.0 },
                Coord { x: 0.0, y: 0.0 },
            ]),
            vec![LineString(vec![
                Coord { x: 1.0, y: 1.0 },
                Coord { x: 2.0, y: 1.0 },
                Coord { x: 2.0, y: 2.0 },
                Coord { x: 1.0, y: 2.0 },
                Coord { x: 1.0, y: 1.0 },
            ])],
        );
        let shapes = multipolygon_to_shapes(&MultiPolygon(vec![small, with_hole]), 1.0);

        assert_eq!(shapes.len(), 1);
        assert_close(shapes[0].area, 15.0);
        assert_eq!(shapes[0].holes.len(), 1);
        assert_eq!(shapes[0].exterior.first(), shapes[0].exterior.last());
        assert_eq!(shapes[0].holes[0].first(), shapes[0].holes[0].last());
    }

    #[test]
    fn multipolygon_to_shapes_handles_empty_and_negative_thresholds() {
        let empty = multipolygon_to_shapes(&MultiPolygon(vec![]), 0.0);
        let zero_area = polygon_from_points(vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]);
        let unit = polygon_from_points(vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);

        assert!(empty.is_empty());
        assert_eq!(
            multipolygon_to_shapes(&MultiPolygon(vec![zero_area]), -1.0e-9).len(),
            1
        );
        assert_eq!(
            multipolygon_to_shapes(&MultiPolygon(vec![unit]), -1.0).len(),
            1
        );
    }

    proptest! {
        #[test]
        fn generated_circles_have_positive_finite_area(
            x in -1000.0f64..1000.0,
            y in -1000.0f64..1000.0,
            radius in 0.001f64..1000.0,
            segments in 3usize..128,
        ) {
            let polygon = circle_polygon([x, y], radius, segments);
            let area = polygon.unsigned_area();
            prop_assert!(area.is_finite());
            prop_assert!(area > 0.0);
            prop_assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
        }

        #[test]
        fn generated_circles_keep_vertices_on_radius(
            x in -1000.0f64..1000.0,
            y in -1000.0f64..1000.0,
            radius in 0.001f64..1000.0,
            segments in 3usize..128,
        ) {
            let polygon = circle_polygon([x, y], radius, segments);
            for coord in polygon.exterior().0.iter().take(polygon.exterior().0.len() - 1) {
                let dx = coord.x - x;
                let dy = coord.y - y;
                prop_assert!(((dx * dx + dy * dy).sqrt() - radius).abs() <= radius.max(1.0) * 1.0e-9);
            }
        }

        #[test]
        fn generated_rectangles_have_expected_area(
            x in -1000.0f64..1000.0,
            y in -1000.0f64..1000.0,
            width in 0.001f64..1000.0,
            height in 0.001f64..1000.0,
            angle in -360.0f64..360.0,
        ) {
            let polygon = rect_polygon([x, y], [width, height], angle);
            prop_assert!((polygon.unsigned_area() - width * height).abs() < (width * height).max(1.0) * 1.0e-9);
        }

        #[test]
        fn generated_rectangles_with_signed_dimensions_have_absolute_area(
            x in -1000.0f64..1000.0,
            y in -1000.0f64..1000.0,
            width in -1000.0f64..1000.0,
            height in -1000.0f64..1000.0,
            angle in -720.0f64..720.0,
        ) {
            let polygon = rect_polygon([x, y], [width, height], angle);
            let expected_area = (width * height).abs();
            prop_assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
            prop_assert!((polygon.unsigned_area() - expected_area).abs() <= expected_area.max(1.0) * 1.0e-9);
        }

        #[test]
        fn generated_lines_have_expected_area(
            start_x in -1000.0f64..1000.0,
            start_y in -1000.0f64..1000.0,
            dx in -1000.0f64..1000.0,
            dy in -1000.0f64..1000.0,
            width in 0.001f64..1000.0,
        ) {
            prop_assume!(dx.abs() > 1.0e-6 || dy.abs() > 1.0e-6);
            let start = [start_x, start_y];
            let end = [start_x + dx, start_y + dy];
            let polygon = line_polygon(start, end, width).unwrap();
            let expected_area = (dx * dx + dy * dy).sqrt() * width;

            prop_assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
            prop_assert!((polygon.unsigned_area() - expected_area).abs() <= expected_area.max(1.0) * 1.0e-9);
        }

        #[test]
        fn generated_arcs_return_one_polygon_per_nonzero_chord(
            center_x in -1000.0f64..1000.0,
            center_y in -1000.0f64..1000.0,
            radius in 0.001f64..1000.0,
            angle in -720.0f64..720.0,
            width in 0.001f64..100.0,
            requested_segments in 0usize..64,
        ) {
            prop_assume!(angle.abs() > 1.0e-6);
            let segments = requested_segments.max(4);
            let polygons = arc_line_polygons(
                [center_x, center_y],
                [center_x + radius, center_y],
                angle,
                width,
                requested_segments,
            );

            prop_assert_eq!(polygons.len(), segments);
            for polygon in polygons {
                prop_assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
                prop_assert!(polygon.unsigned_area().is_finite());
                prop_assert!(polygon.unsigned_area() > 0.0);
            }
        }

        #[test]
        fn polygon_from_points_always_closes_nonempty_open_rings(
            points in prop::collection::vec((-1000.0f64..1000.0, -1000.0f64..1000.0), 2..32)
        ) {
            let polygon = polygon_from_points(points.into_iter().map(|(x, y)| [x, y]).collect());

            prop_assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
        }

        #[test]
        fn transform_preserves_polygon_area(angle in -360.0f64..360.0, x in -100.0f64..100.0, y in -100.0f64..100.0) {
            let polygon = rect_polygon([0.0, 0.0], [3.0, 2.0], 0.0);
            let transformed = transform_polygon(&polygon, [x, y], angle);
            prop_assert!((polygon.unsigned_area() - transformed.unsigned_area()).abs() < 1.0e-9);
        }

        #[test]
        fn transform_preserves_edge_lengths(
            width in 0.001f64..1000.0,
            height in 0.001f64..1000.0,
            original_angle in -360.0f64..360.0,
            transform_angle in -360.0f64..360.0,
            x in -1000.0f64..1000.0,
            y in -1000.0f64..1000.0,
        ) {
            let polygon = rect_polygon([0.0, 0.0], [width, height], original_angle);
            let transformed = transform_polygon(&polygon, [x, y], transform_angle);

            for (left, right) in polygon.exterior().0.windows(2).zip(transformed.exterior().0.windows(2)) {
                prop_assert!((distance(left[0], left[1]) - distance(right[0], right[1])).abs() <= width.max(height).max(1.0) * 1.0e-9);
            }
        }
    }
}
