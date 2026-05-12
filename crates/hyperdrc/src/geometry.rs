use csgrs::float_types::Real;
use csgrs::sketch::Sketch;
use std::f64::consts::PI;

use geo::{Area, Coord, Geometry, GeometryCollection, LineString, MultiPolygon, Polygon};

use crate::LayerMetadata;
use crate::report::ViolationPolygon;

pub fn polygon_to_sketch(
    polygon: Polygon<Real>,
    metadata: Option<LayerMetadata>,
) -> Sketch<LayerMetadata> {
    Sketch::from_geo(
        GeometryCollection(vec![Geometry::Polygon(polygon)]),
        metadata,
    )
}

pub fn polygons_to_sketch(
    polygons: Vec<Polygon<Real>>,
    metadata: Option<LayerMetadata>,
) -> Sketch<LayerMetadata> {
    Sketch::from_geo(
        GeometryCollection(vec![Geometry::MultiPolygon(MultiPolygon(polygons))]),
        metadata,
    )
}

pub fn empty_sketch(metadata: Option<LayerMetadata>) -> Sketch<LayerMetadata> {
    Sketch::from_geo(GeometryCollection::default(), metadata)
}

pub fn circle_polygon(center: [f64; 2], radius: f64, segments: usize) -> Polygon<Real> {
    let segments = segments.max(16);
    let mut coords = Vec::with_capacity(segments + 1);
    for index in 0..segments {
        let theta = 2.0 * PI * (index as f64) / (segments as f64);
        coords.push(Coord {
            x: center[0] + radius * theta.cos(),
            y: center[1] + radius * theta.sin(),
        });
    }
    coords.push(coords[0]);
    Polygon::new(LineString(coords), vec![])
}

pub fn rect_polygon(center: [f64; 2], size: [f64; 2], angle_degrees: f64) -> Polygon<Real> {
    let half_x = size[0] / 2.0;
    let half_y = size[1] / 2.0;
    let points = [
        [-half_x, -half_y],
        [half_x, -half_y],
        [half_x, half_y],
        [-half_x, half_y],
    ];
    polygon_from_points(
        points
            .iter()
            .map(|point| rotate_translate(*point, center, angle_degrees))
            .collect(),
    )
}

pub fn line_polygon(start: [f64; 2], end: [f64; 2], width: f64) -> Option<Polygon<Real>> {
    let dx = end[0] - start[0];
    let dy = end[1] - start[1];
    let length = (dx * dx + dy * dy).sqrt();
    if length == 0.0 || width <= 0.0 {
        return None;
    }

    let nx = -dy / length * width / 2.0;
    let ny = dx / length * width / 2.0;
    Some(polygon_from_points(vec![
        [start[0] + nx, start[1] + ny],
        [end[0] + nx, end[1] + ny],
        [end[0] - nx, end[1] - ny],
        [start[0] - nx, start[1] - ny],
    ]))
}

pub fn polygon_from_points(points: Vec<[f64; 2]>) -> Polygon<Real> {
    let mut coords = points
        .into_iter()
        .map(|point| Coord {
            x: point[0],
            y: point[1],
        })
        .collect::<Vec<_>>();

    if coords.first() != coords.last()
        && let Some(first) = coords.first().copied()
    {
        coords.push(first);
    }

    Polygon::new(LineString(coords), vec![])
}

pub fn transform_polygon(
    polygon: &Polygon<Real>,
    origin: [f64; 2],
    angle_degrees: f64,
) -> Polygon<Real> {
    let exterior = polygon
        .exterior()
        .0
        .iter()
        .map(|coord| rotate_translate([coord.x, coord.y], origin, angle_degrees))
        .collect::<Vec<_>>();
    let holes = polygon
        .interiors()
        .iter()
        .map(|ring| {
            LineString(
                ring.0
                    .iter()
                    .map(|coord| {
                        let point = rotate_translate([coord.x, coord.y], origin, angle_degrees);
                        Coord {
                            x: point[0],
                            y: point[1],
                        }
                    })
                    .collect(),
            )
        })
        .collect();

    Polygon::new(
        LineString(
            exterior
                .into_iter()
                .map(|point| Coord {
                    x: point[0],
                    y: point[1],
                })
                .collect(),
        ),
        holes,
    )
}

pub fn arc_line_polygons(
    center: [f64; 2],
    start: [f64; 2],
    angle_degrees: f64,
    width: f64,
    segments: usize,
) -> Vec<Polygon<Real>> {
    let segments = segments.max(4);
    let mut points = Vec::with_capacity(segments + 1);
    let start_vector = [start[0] - center[0], start[1] - center[1]];

    for index in 0..=segments {
        let theta = angle_degrees.to_radians() * (index as f64) / (segments as f64);
        let cos = theta.cos();
        let sin = theta.sin();
        points.push([
            center[0] + start_vector[0] * cos - start_vector[1] * sin,
            center[1] + start_vector[0] * sin + start_vector[1] * cos,
        ]);
    }

    points
        .windows(2)
        .filter_map(|window| line_polygon(window[0], window[1], width))
        .collect()
}

pub fn multipolygon_to_shapes(
    multipolygon: &MultiPolygon<Real>,
    min_area: f64,
) -> Vec<ViolationPolygon> {
    multipolygon
        .0
        .iter()
        .filter_map(|polygon| {
            let area = polygon.unsigned_area();
            (area > min_area).then(|| ViolationPolygon {
                area,
                exterior: ring_to_coordinates(polygon.exterior()),
                holes: polygon
                    .interiors()
                    .iter()
                    .map(ring_to_coordinates)
                    .collect(),
            })
        })
        .collect()
}

fn ring_to_coordinates(ring: &LineString<Real>) -> Vec<[f64; 2]> {
    ring.0.iter().map(|Coord { x, y }| [*x, *y]).collect()
}

fn rotate_translate(point: [f64; 2], center: [f64; 2], angle_degrees: f64) -> [f64; 2] {
    let theta = angle_degrees.to_radians();
    let cos = theta.cos();
    let sin = theta.sin();
    [
        center[0] + point[0] * cos - point[1] * sin,
        center[1] + point[0] * sin + point[1] * cos,
    ]
}

#[cfg(test)]
mod tests {
    use geo::Area;
    use proptest::prelude::*;

    use super::{
        arc_line_polygons, circle_polygon, line_polygon, polygon_from_points, rect_polygon,
        transform_polygon,
    };

    #[test]
    fn line_polygon_rejects_degenerate_inputs() {
        assert!(line_polygon([0.0, 0.0], [0.0, 0.0], 1.0).is_none());
        assert!(line_polygon([0.0, 0.0], [1.0, 0.0], 0.0).is_none());
        assert!(line_polygon([0.0, 0.0], [1.0, 0.0], -1.0).is_none());
    }

    #[test]
    fn polygon_from_points_closes_open_ring() {
        let polygon = polygon_from_points(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]);

        assert_eq!(polygon.exterior().0.first(), polygon.exterior().0.last());
    }

    #[test]
    fn arc_with_zero_angle_produces_no_valid_segments() {
        let polygons = arc_line_polygons([0.0, 0.0], [1.0, 0.0], 0.0, 0.1, 8);

        assert!(polygons.is_empty());
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
        fn transform_preserves_polygon_area(angle in -360.0f64..360.0, x in -100.0f64..100.0, y in -100.0f64..100.0) {
            let polygon = rect_polygon([0.0, 0.0], [3.0, 2.0], 0.0);
            let transformed = transform_polygon(&polygon, [x, y], angle);
            prop_assert!((polygon.unsigned_area() - transformed.unsigned_area()).abs() < 1.0e-9);
        }
    }
}
