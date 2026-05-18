use crate::{csg::CSG, float_types::Real, mesh::Mesh, sketch::Sketch};
use geo::{BoundingRect, GeometryCollection};
use nalgebra::{Point2, Point3, Vector3};

fn assert_close(actual: Real, expected: Real, eps: Real) {
    assert!(
        (actual - expected).abs() <= eps,
        "expected {expected}, got {actual}"
    );
}

#[test]
fn sketch_rectangle_populates_hypercurve_region() {
    let sketch = Sketch::rectangle(2.0, 3.0, "rect");

    assert_eq!(sketch.metadata, "rect");
    assert_eq!(sketch.material_contour_count(), 1);
    assert_eq!(sketch.hole_contour_count(), 0);
    assert!(!sketch.as_region().material_contours().is_empty());
}

#[test]
fn sketch_geometry_accessor_derives_from_hypercurve_region() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    let geometry = sketch.geometry();

    assert!(!geometry.0.is_empty());
    assert!(!sketch.as_region().is_empty());
}

#[test]
fn sketch_geometry_accessor_prefers_native_region_over_stale_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let geometry = sketch.geometry();
    let bounds = geometry.bounding_rect().expect("native region projection");

    assert_close(bounds.max().x, 2.0, 1e-9);
    assert_close(bounds.max().y, 3.0, 1e-9);
}

#[test]
fn sketch_from_geo_recomposes_boundary_geometry_into_hyper_types() {
    let polygon = geo::Polygon::new(
        geo::LineString::from(vec![
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0),
        ]),
        vec![],
    );
    let line = geo::LineString::from(vec![(3.0, 0.0), (4.0, 1.0), (5.0, 0.0)]);
    let mut sketch = Sketch::from_geo(
        GeometryCollection(vec![
            geo::Geometry::Polygon(polygon),
            geo::Geometry::LineString(line),
        ]),
        "from_geo",
    );

    assert_eq!(sketch.metadata, "from_geo");
    assert_eq!(sketch.material_contour_count(), 1);
    assert_eq!(sketch.wires().len(), 1);
    assert!(sketch.contains_xy(1.0, 1.0).unwrap());

    sketch.geometry = GeometryCollection::default();
    let geometry = sketch.geometry();
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::MultiPolygon(_))),
        "finite area cache should regenerate from Region2"
    );
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "finite line cache should regenerate from CurveString2"
    );
}

#[test]
fn sketch_from_geo_retains_unsupported_boundary_geometry() {
    let sketch = Sketch::from_geo(
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(1.0, 2.0))]),
        (),
    );

    assert!(sketch.as_region().is_empty());
    assert!(sketch.wires().is_empty());
    assert!(
        sketch
            .geometry()
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::Point(_))),
        "unsupported finite interop geometry should remain available"
    );
}

#[test]
fn sketch_region_rings_and_contains_do_not_need_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    let rings = sketch.region_rings();

    assert_eq!(rings.material.len(), 1);
    assert_eq!(rings.holes.len(), 0);
    assert!(sketch.contains_xy(1.0, 1.0).unwrap());
    assert!(!sketch.contains_xy(3.0, 1.0).unwrap());
}

#[test]
fn sketch_open_wire_geometry_derives_from_hypercurve_wires() {
    let mut sketch = Sketch::bezier(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]], 8, ());

    assert_eq!(sketch.wires().len(), 1);
    sketch.geometry = GeometryCollection::default();

    let geometry = sketch.geometry();
    assert_eq!(geometry.0.len(), 1);
    assert!(sketch.as_region().is_empty());
    assert_eq!(sketch.build_graphic_line_strings().line_strings.len(), 1);
}

#[test]
fn sketch_open_wire_transform_refreshes_hypercurve_wires() {
    let mut sketch = Sketch::bezier(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]], 8, ());
    sketch.geometry = GeometryCollection::default();

    let moved = sketch.translate(5.0, 7.0, 0.0);
    assert_eq!(moved.wires().len(), 1);
    assert!(
        moved
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| { (point[0] - 7.0).abs() <= 1e-9 && (point[1] - 7.0).abs() <= 1e-9 }),
        "native wire coordinates should be transformed without using the geo cache"
    );

    let mut cacheless = moved.clone();
    cacheless.geometry = GeometryCollection::default();
    let geometry = cacheless.geometry();
    let geo::Geometry::LineString(line) = &geometry.0[0] else {
        panic!("wire should re-export as LineString");
    };
    let first = line.0[0];

    assert_close(first.x, 5.0, 1e-9);
    assert_close(first.y, 7.0, 1e-9);
}

#[test]
fn sketch_transform_fallback_recomposes_supported_geometry() {
    let line = geo::LineString::from(vec![(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]);
    let mut sketch = Sketch::from_geo(
        GeometryCollection(vec![
            geo::Geometry::LineString(line),
            geo::Geometry::Point(geo::Point::new(9.0, 9.0)),
        ]),
        (),
    );

    let mut moved = sketch.translate(3.0, 4.0, 0.0);

    assert_eq!(moved.wires().len(), 1);
    assert!(moved.as_region().is_empty());

    moved.geometry = GeometryCollection::default();
    let geometry = moved.geometry();
    assert_eq!(geometry.0.len(), 1);
    let geo::Geometry::LineString(line) = &geometry.0[0] else {
        panic!("transformed supported geometry should regenerate from CurveString2");
    };
    let first = line.0[0];
    assert_close(first.x, 3.0, 1e-9);
    assert_close(first.y, 4.0, 1e-9);

    sketch.geometry = GeometryCollection::default();
    assert_eq!(sketch.geometry().0.len(), 1);
}

#[test]
fn sketch_transform_fallback_preserves_degenerate_revolve_profile() {
    let profile = Sketch::square(2.0, ())
        .translate(1.0, 0.0, 0.0)
        .rotate(90.0, 0.0, 0.0);

    let mesh = profile
        .revolve(360.0, 16)
        .expect("degenerate profile remains valid finite revolve input");

    assert!(!mesh.polygons.is_empty());
}

#[test]
fn sketch_mixed_region_and_wire_transform_preserves_both_native_parts() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]], 8, ());
    sketch.wires = wire.wires().to_vec();
    sketch.geometry = GeometryCollection::default();

    let moved = sketch.translate(5.0, 7.0, 0.0);

    assert_eq!(moved.material_contour_count(), 1);
    assert_eq!(moved.wires().len(), 1);
    assert!(moved.contains_xy(5.5, 7.5).unwrap());
    assert!(
        moved
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| point[0] >= 7.0 && point[1] >= 7.0),
        "native wire coordinates should be transformed without using the geo cache"
    );

    let mut cacheless = moved.clone();
    cacheless.geometry = GeometryCollection::default();
    let geometry = cacheless.geometry();
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::MultiPolygon(_))),
        "native region should project back to compatibility geometry"
    );
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "native wire should project back to compatibility geometry"
    );
}

#[test]
fn sketch_from_region_and_wires_composes_native_hyper_geometry() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]], 8, ());

    let mut sketch = Sketch::from_region_and_wires(
        area.as_region().clone(),
        wire.wires().to_vec(),
        "mixed",
    );

    assert_eq!(sketch.metadata, "mixed");
    assert_eq!(sketch.material_contour_count(), 1);
    assert_eq!(sketch.wires().len(), 1);
    assert!(sketch.contains_xy(1.0, 1.0).unwrap());

    sketch.geometry = GeometryCollection::default();
    let geometry = sketch.geometry();
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::MultiPolygon(_))),
        "native region should project to compatibility geometry"
    );
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "native wire should project to compatibility geometry"
    );
}

#[test]
fn sketch_geometry_accessor_prefers_mixed_native_topology_over_stale_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[5.0, -1.0], [6.0, 4.0], [7.0, 1.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let geometry = sketch.geometry();
    let bounds = geometry.bounding_rect().expect("native mixed projection");

    assert_close(bounds.min().x, 0.0, 1e-9);
    assert_close(bounds.min().y, -1.0, 1e-9);
    assert_close(bounds.max().x, 7.0, 1e-9);
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "native wire should appear in the finite compatibility projection"
    );
}

#[test]
fn sketch_mixed_boolean_fallback_recomposes_native_hyper_geometry() {
    let area = Sketch::rectangle(2.0, 2.0, ());
    let wire = Sketch::bezier(&[[0.0, 3.0], [1.0, 4.0], [2.0, 3.0]], 8, ());
    let mixed =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    let other = Sketch::rectangle(2.0, 2.0, ()).translate(1.0, 0.0, 0.0);

    let mut union = mixed.union(&other);

    assert!(!union.as_region().is_empty());
    assert_eq!(union.wires().len(), 1);

    union.geometry = GeometryCollection::default();
    let geometry = union.geometry();
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::MultiPolygon(_))),
        "mixed boolean fallback should regenerate area compatibility from Region2"
    );
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "mixed boolean fallback should regenerate wires from CurveString2"
    );
}

#[test]
fn sketch_boolean_fallback_prefers_native_wires_over_stale_sidecar_cache() {
    let area = Sketch::rectangle(2.0, 2.0, ());
    let wire = Sketch::bezier(&[[4.0, 0.0], [5.0, 1.0], [6.0, 0.0]], 8, ());
    let mut mixed =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    mixed.geometry =
        GeometryCollection(vec![geo::Geometry::LineString(geo::LineString::from(vec![
            (100.0, 100.0),
            (101.0, 100.0),
        ]))]);

    let mut union = mixed.union(&Sketch::empty(()));

    assert!(!union.as_region().is_empty());
    assert_eq!(union.wires().len(), 1);
    assert!(
        union
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| point[0] >= 6.0),
        "native CurveString2 wire should survive finite boolean fallback"
    );

    union.geometry = GeometryCollection::default();
    let geometry = union.geometry();
    assert!(
        geometry.0.iter().all(|geometry| {
            !matches!(
                geometry,
                geo::Geometry::LineString(line) if line.0.iter().any(|coord| coord.x >= 100.0)
            )
        }),
        "stale finite sidecar cache should not survive native boolean sidecar handling"
    );
}

#[test]
fn sketch_region_boolean_fast_path_preserves_native_wires_without_geo_cache() {
    let left_area = Sketch::rectangle(2.0, 2.0, ());
    let left_wire = Sketch::bezier(&[[0.0, 3.0], [1.0, 4.0], [2.0, 3.0]], 8, ());
    let right_area = Sketch::rectangle(2.0, 2.0, ()).translate(1.0, 0.0, 0.0);
    let right_wire = Sketch::bezier(&[[3.0, 3.0], [4.0, 4.0], [5.0, 3.0]], 8, ());

    let mut left = Sketch::from_region_and_wires(
        left_area.as_region().clone(),
        left_wire.wires().to_vec(),
        (),
    );
    let mut right = Sketch::from_region_and_wires(
        right_area.as_region().clone(),
        right_wire.wires().to_vec(),
        (),
    );
    left.geometry = GeometryCollection::default();
    right.geometry = GeometryCollection::default();

    let union = left.union(&right);
    let difference = left.difference(&right);

    assert!(!union.as_region().is_empty());
    assert_eq!(
        union.wires().len(),
        2,
        "native region boolean union must preserve both operands' wires"
    );
    assert!(
        union
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| point[0] >= 5.0),
        "right operand wire extent should survive native union"
    );

    assert!(!difference.as_region().is_empty());
    assert_eq!(
        difference.wires().len(),
        1,
        "native region boolean difference must preserve left operand wires"
    );
    assert!(
        difference
            .wire_polylines()
            .iter()
            .flatten()
            .all(|point| point[0] <= 2.0),
        "difference should not import right operand wires"
    );
}

#[test]
fn sketch_region_boolean_fast_path_ignores_stale_non_area_geo_cache() {
    let mut left = Sketch::rectangle(2.0, 2.0, ());
    let mut right = Sketch::rectangle(2.0, 2.0, ()).translate(1.0, 0.0, 0.0);
    left.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);
    right.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(60.0, 60.0))]);

    let mut union = left.union(&right);

    assert!(!union.as_region().is_empty());
    assert!(union.contains_xy(0.5, 0.5).unwrap());
    assert!(union.contains_xy(2.5, 0.5).unwrap());

    union.geometry = GeometryCollection::default();
    let geometry = union.geometry();
    assert!(
        geometry
            .0
            .iter()
            .all(|geometry| !matches!(geometry, geo::Geometry::Point(_))),
        "stale non-area compatibility geometry should not participate in native region boolean"
    );
}

#[test]
fn sketch_hilbert_curve_is_backed_by_native_wires() {
    let mut area = Sketch::rectangle(4.0, 4.0, ());
    area.geometry = GeometryCollection::default();

    let mut curve = area.hilbert_curve(3, 0.1);

    assert!(curve.as_region().is_empty());
    assert!(!curve.wires().is_empty());
    assert!(!curve.wire_polylines().is_empty());

    curve.geometry = GeometryCollection::default();
    let geometry = curve.geometry();
    assert!(
        geometry
            .0
            .iter()
            .all(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "native Hilbert wires should project to line strings only at compatibility boundaries"
    );
}

#[test]
fn sketch_hilbert_curve_prefers_native_wire_bounds_over_stale_geo_cache() {
    let mut sketch = Sketch::bezier(&[[5.0, 1.0], [6.0, 4.0], [7.0, 1.0]], 8, ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let curve = sketch.hilbert_curve(2, 0.0);

    assert!(curve.as_region().is_empty());
    assert!(!curve.wires().is_empty());
    assert!(
        curve.wire_polylines().iter().flatten().all(|point| {
            point[0] >= 5.0 && point[0] <= 7.0 && point[1] >= 1.0 && point[1] <= 4.0
        }),
        "Hilbert bounds should come from native CurveString2 wires, not stale finite geometry"
    );
}

#[test]
#[cfg(feature = "hershey-text")]
fn sketch_hershey_text_is_backed_by_native_wires() {
    static GLYPHS: [&str; 1] = ["RRSSUT"];
    let font = hershey::Font::new(&GLYPHS, 'A');

    let mut sketch = Sketch::from_hershey("A A", &font, 2.0, ());

    assert!(sketch.as_region().is_empty());
    assert_eq!(sketch.wires().len(), 2);
    assert_eq!(sketch.wire_polylines().len(), 2);

    sketch.geometry = GeometryCollection::default();
    let geometry = sketch.geometry();
    assert_eq!(geometry.0.len(), 2);
    assert!(
        geometry
            .0
            .iter()
            .all(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "Hershey wires should only become geo line strings at compatibility boundaries"
    );
}

#[test]
#[cfg(feature = "truetype-text")]
fn sketch_truetype_text_is_backed_by_native_region() {
    let font_data = include_bytes!("../../asar.ttf");
    let mut sketch: Sketch<()> = Sketch::text("ABC", font_data, 10.0, ());

    assert!(!sketch.as_region().is_empty());
    assert!(sketch.material_contour_count() > 0);
    assert!(sketch.native_xy_bounds().is_some());

    sketch.geometry = GeometryCollection::default();
    assert!(!sketch.geometry().is_empty());
    assert!(!sketch.build_graphic_line_strings().line_strings.is_empty());
}

#[test]
#[cfg(feature = "svg-io")]
fn sketch_svg_open_paths_are_backed_by_native_wires() {
    use crate::io::svg::FromSVG;

    let svg = r#"
<svg viewBox="0 0 10 10" xmlns="http://www.w3.org/2000/svg">
  <path d="M 1 1 L 4 1 L 4 4" />
  <polyline points="6,1 8,1 8,3" />
  <line x1="1" y1="8" x2="4" y2="8" />
</svg>
"#;

    let mut sketch = Sketch::<()>::from_svg(svg, ()).unwrap();

    assert!(sketch.as_region().is_empty());
    assert_eq!(sketch.wires().len(), 3);
    assert_eq!(sketch.wire_polylines().len(), 3);

    sketch.geometry = GeometryCollection::default();
    let geometry = sketch.geometry();
    assert_eq!(geometry.0.len(), 3);
    assert!(
        geometry
            .0
            .iter()
            .all(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "open SVG geometry should only project to geo line strings at compatibility boundaries"
    );
}

#[test]
#[cfg(feature = "svg-io")]
fn sketch_svg_exports_mixed_native_region_and_wires_without_geo_cache() {
    use crate::io::svg::ToSVG;

    let area = Sketch::rectangle(2.0, 2.0, ());
    let wires = Sketch::bezier(&[[4.0, 0.0], [5.0, 1.0], [6.0, 0.0]], 8, ())
        .wires()
        .to_vec();
    let mut sketch = Sketch::from_region_and_wires(area.as_region().clone(), wires, ());
    sketch.geometry = GeometryCollection::default();

    let svg = sketch.to_svg();

    assert!(
        svg.matches("<path").count() >= 2,
        "mixed native Sketch should export both filled region and open wire paths: {svg}"
    );
    assert!(
        svg.contains("fill=\"black\""),
        "missing filled region path: {svg}"
    );
    assert!(svg.contains("fill=\"none\""), "missing open wire path: {svg}");
    assert!(
        svg.contains("viewBox=\"0 0 6 2\""),
        "native SVG bounds should include region and wire extents: {svg}"
    );
}

#[test]
#[cfg(feature = "svg-io")]
fn sketch_svg_prefers_mixed_native_topology_over_stale_geo_cache() {
    use crate::io::svg::ToSVG;

    let area = Sketch::rectangle(2.0, 2.0, ());
    let wires = Sketch::bezier(&[[4.0, 0.0], [5.0, 1.0], [6.0, 0.0]], 8, ())
        .wires()
        .to_vec();
    let mut sketch = Sketch::from_region_and_wires(area.as_region().clone(), wires, ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let svg = sketch.to_svg();

    assert!(
        svg.matches("<path").count() >= 2,
        "mixed native Sketch should export region and wire paths from hypercurve: {svg}"
    );
    assert!(svg.contains("fill=\"black\""), "missing native region: {svg}");
    assert!(svg.contains("fill=\"none\""), "missing native wire: {svg}");
    assert!(
        svg.contains("viewBox=\"0 0 6 2\""),
        "stale finite geometry must not determine SVG bounds: {svg}"
    );
    assert!(
        !svg.contains("100"),
        "stale finite geometry should not leak into SVG output: {svg}"
    );
}

#[test]
fn sketch_triangulate_uses_hypercurve_region_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    let triangles = sketch.triangulate();

    assert_eq!(triangles.len(), 2);
    for triangle in triangles {
        for point in triangle {
            assert!(point.x.is_finite());
            assert!(point.y.is_finite());
            assert_eq!(point.z, 0.0);
        }
    }
}

#[test]
fn sketch_mesh_conversion_uses_hypercurve_region_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, "rect");
    sketch.geometry = GeometryCollection::default();

    let mesh = Mesh::from(sketch);

    assert_eq!(mesh.metadata, "rect");
    assert_eq!(mesh.polygons.len(), 1);
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 2.0, 1e-9);
    assert_close(bbox.maxs.y, 3.0, 1e-9);
}

#[test]
fn sketch_mesh_conversion_prefers_native_region_over_stale_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, "rect");
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let mesh = Mesh::from(sketch);

    assert_eq!(mesh.metadata, "rect");
    assert_eq!(
        mesh.polygons.len(),
        1,
        "native Region2 should define mesh conversion despite stale finite cache"
    );
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 2.0, 1e-9);
    assert_close(bbox.maxs.y, 3.0, 1e-9);
}

#[test]
#[cfg(feature = "mesh")]
fn sketch_from_mesh_promotes_flattened_projection_to_hypercurve_region() {
    let mesh = Mesh::cube(2.0, "cube");
    let mut sketch = Sketch::from(mesh);

    assert_eq!(sketch.metadata, "cube");
    assert!(!sketch.as_region().is_empty());
    assert!(!sketch.region_rings().material.is_empty());

    sketch.geometry = GeometryCollection::default();
    assert!(!sketch.geometry().is_empty());
    assert!(!sketch.triangulate().is_empty());
}

#[test]
fn sketch_renormalize_preserves_hypercurve_region_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(4.0, 4.0, ())
        .difference(&Sketch::rectangle(1.0, 1.0, ()).translate(1.0, 1.0, 0.0));
    sketch.geometry = GeometryCollection::default();

    let normalized = sketch.renormalize();

    assert_eq!(normalized.material_contour_count(), 1);
    assert_eq!(normalized.hole_contour_count(), 1);
    assert!(normalized.contains_xy(0.5, 0.5).unwrap());
    assert!(!normalized.contains_xy(1.5, 1.5).unwrap());
}

#[test]
fn sketch_renormalize_prefers_native_region_over_stale_non_area_geo_cache() {
    let mut sketch = Sketch::rectangle(4.0, 4.0, ())
        .difference(&Sketch::rectangle(1.0, 1.0, ()).translate(1.0, 1.0, 0.0));
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let mut normalized = sketch.renormalize();

    assert_eq!(normalized.material_contour_count(), 1);
    assert_eq!(normalized.hole_contour_count(), 1);
    assert!(normalized.contains_xy(0.5, 0.5).unwrap());
    assert!(!normalized.contains_xy(1.5, 1.5).unwrap());

    normalized.geometry = GeometryCollection::default();
    let geometry = normalized.geometry();
    assert!(
        geometry
            .0
            .iter()
            .all(|geometry| !matches!(geometry, geo::Geometry::Point(_))),
        "stale non-area compatibility geometry should not survive native renormalize"
    );
}

#[test]
fn sketch_mixed_renormalize_recomposes_native_hyper_geometry() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[0.0, 4.0], [1.0, 5.0], [2.0, 4.0]], 8, ());
    let sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());

    let mut normalized = sketch.renormalize();

    assert_eq!(normalized.material_contour_count(), 1);
    assert_eq!(normalized.wires().len(), 1);
    assert!(normalized.contains_xy(1.0, 1.0).unwrap());

    normalized.geometry = GeometryCollection::default();
    let geometry = normalized.geometry();
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::MultiPolygon(_))),
        "renormalized area should regenerate from Region2"
    );
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "renormalized wire should regenerate from CurveString2"
    );
}

#[test]
fn sketch_mixed_renormalize_preserves_native_wires_without_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[0.0, 4.0], [1.0, 5.0], [2.0, 4.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = GeometryCollection::default();

    let normalized = sketch.renormalize();

    assert_eq!(normalized.material_contour_count(), 1);
    assert_eq!(
        normalized.wires().len(),
        1,
        "native renormalize must preserve open CurveString2 wires"
    );
    assert!(
        normalized
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| point[1] >= 4.0),
        "renormalized wire extent should remain available"
    );
}

#[test]
fn sketch_mixed_renormalize_preserves_native_wires_over_stale_non_area_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[0.0, 4.0], [1.0, 5.0], [2.0, 4.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let normalized = sketch.renormalize();

    assert_eq!(normalized.material_contour_count(), 1);
    assert_eq!(
        normalized.wires().len(),
        1,
        "native renormalize must preserve wires even when the finite cache is stale"
    );
    assert!(
        normalized
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| point[1] >= 4.0),
        "renormalized wire extent should come from native CurveString2"
    );
}

#[test]
fn sketch_inverse_recomposes_supported_boundary_geometry() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[0.0, 4.0], [1.0, 5.0], [2.0, 4.0]], 8, ());
    let sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());

    let mut inverted = sketch.inverse();

    assert_eq!(inverted.material_contour_count(), 1);
    assert_eq!(inverted.wires().len(), 1);

    inverted.geometry = GeometryCollection::default();
    let geometry = inverted.geometry();
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::MultiPolygon(_))),
        "inverted supported area should regenerate from Region2"
    );
    assert!(
        geometry
            .0
            .iter()
            .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "inverted supported wire should regenerate from CurveString2"
    );
}

#[test]
fn sketch_inverse_prefers_native_region_over_stale_non_area_geo_cache() {
    let mut sketch = Sketch::rectangle(4.0, 4.0, ())
        .difference(&Sketch::rectangle(1.0, 1.0, ()).translate(1.0, 1.0, 0.0));
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let mut inverted = sketch.inverse();

    assert_eq!(inverted.material_contour_count(), 1);
    assert_eq!(inverted.hole_contour_count(), 1);
    assert!(inverted.contains_xy(0.5, 0.5).unwrap());
    assert!(!inverted.contains_xy(1.5, 1.5).unwrap());

    inverted.geometry = GeometryCollection::default();
    let geometry = inverted.geometry();
    assert!(
        geometry
            .0
            .iter()
            .all(|geometry| !matches!(geometry, geo::Geometry::Point(_))),
        "stale non-area compatibility geometry should not survive native inverse"
    );
}

#[test]
fn sketch_mixed_inverse_preserves_native_wires_over_stale_non_area_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[0.0, 4.0], [1.0, 5.0], [2.0, 4.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let inverted = sketch.inverse();

    assert_eq!(inverted.material_contour_count(), 1);
    assert_eq!(
        inverted.wires().len(),
        1,
        "native inverse must preserve open CurveString2 wires despite a stale cache"
    );
    assert!(
        inverted
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| point[1] >= 4.0),
        "inverted wire extent should come from native CurveString2"
    );
}

#[test]
fn sketch_graphics_uses_hypercurve_region_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    let lines = sketch.build_graphic_line_strings();

    assert_eq!(lines.line_strings.len(), 1);
    assert_eq!(lines.total_line_count(), 4);
    assert!(
        lines
            .line_strings
            .iter()
            .flat_map(|line| line.points.iter())
            .all(|point| point.iter().all(|value| value.is_finite()))
    );
}

#[test]
fn sketch_graphics_include_mixed_native_region_and_wires_without_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[5.0, -1.0], [6.0, 4.0], [7.0, 1.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = GeometryCollection::default();

    let lines = sketch.build_graphic_line_strings();

    assert_eq!(lines.line_strings.len(), 2);
    assert!(
        lines
            .line_strings
            .iter()
            .flat_map(|line| line.points.iter())
            .any(|point| point[0] >= 7.0_f32),
        "native wire extent should be present in graphic output: {lines:?}"
    );
}

#[test]
fn sketch_graphics_prefer_mixed_native_topology_over_stale_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[5.0, -1.0], [6.0, 4.0], [7.0, 1.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let lines = sketch.build_graphic_line_strings();

    assert_eq!(lines.line_strings.len(), 2);
    assert!(
        lines
            .line_strings
            .iter()
            .flat_map(|line| line.points.iter())
            .any(|point| point[0] >= 7.0_f32),
        "native wire extent should beat stale finite geometry: {lines:?}"
    );
    assert!(
        lines
            .line_strings
            .iter()
            .flat_map(|line| line.points.iter())
            .all(|point| point[0] <= 7.0_f32 && point[1] <= 4.0_f32),
        "stale finite cache should not contribute render lines: {lines:?}"
    );
}

#[test]
fn sketch_region_area_paths_ignore_stale_area_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let bbox = sketch.bounding_box();
    assert_close(bbox.maxs.x, 2.0, 1e-9);
    assert_close(bbox.maxs.y, 3.0, 1e-9);

    let lines = sketch.build_graphic_line_strings();
    assert_eq!(lines.total_line_count(), 4);
    assert!(
        lines
            .line_strings
            .iter()
            .flat_map(|line| line.points.iter())
            .all(|point| point[0] <= 2.0_f32 && point[1] <= 3.0_f32)
    );
}

#[test]
fn sketch_mixed_native_bounds_include_region_and_wires_without_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[5.0, -1.0], [6.0, 4.0], [7.0, 1.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = GeometryCollection::default();

    let bbox = sketch.bounding_box();

    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, -1.0, 1e-9);
    assert_close(bbox.maxs.x, 7.0, 1e-9);
    assert!(bbox.maxs.y >= 3.0);
    assert!(bbox.maxs.y <= 4.0 + 1e-9);
}

#[test]
fn sketch_to_multipolygon_projects_hypercurve_region_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(4.0, 4.0, ())
        .difference(&Sketch::rectangle(1.0, 1.0, ()).translate(1.0, 1.0, 0.0));
    sketch.geometry = GeometryCollection::default();

    let multipolygon = sketch.to_multipolygon();

    assert_eq!(multipolygon.0.len(), 1);
    assert_eq!(multipolygon.0[0].interiors().len(), 1);
}

#[test]
fn sketch_primitives_start_as_hypercurve_regions() {
    let rectangle = Sketch::rectangle(2.0, 3.0, ());
    let triangle = Sketch::right_triangle(2.0, 3.0, ());
    let polygon = Sketch::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]], ());
    let circle = Sketch::circle(1.0, 16, ());
    let ellipse = Sketch::ellipse(2.0, 1.0, 16, ());
    let ngon = Sketch::regular_ngon(7, 1.0, ());
    let trapezoid = Sketch::trapezoid(1.0, 2.0, 1.5, 0.25, ());
    let star = Sketch::star(5, 2.0, 0.75, ());

    for sketch in [
        rectangle, triangle, polygon, circle, ellipse, ngon, trapezoid, star,
    ] {
        assert_eq!(sketch.material_contour_count(), 1);
        assert_eq!(sketch.hole_contour_count(), 0);
        assert!(!sketch.triangulate().is_empty());
        assert!(!sketch.region_rings().material.is_empty());
    }
}

#[test]
fn sketch_curved_approximations_start_as_hypercurve_regions() {
    let shapes = [
        Sketch::teardrop(2.0, 4.0, 12, ()),
        Sketch::egg(2.0, 3.0, 16, ()),
        Sketch::rounded_rectangle(3.0, 2.0, 0.25, 4, ()),
        Sketch::squircle(2.0, 2.0, 16, ()),
        Sketch::pie_slice(2.0, 15.0, 130.0, 8, ()),
        Sketch::supershape(1.0, 1.0, 5.0, 0.35, 1.7, 1.7, 24, ()),
        Sketch::heart(2.0, 2.0, 32, ()),
        Sketch::bezier(&[[0.0, 0.0], [1.5, 0.0], [1.5, 1.5], [0.0, 0.0]], 16, ()),
        Sketch::airfoil_naca4(2.0, 4.0, 12.0, 1.0, 16, ()),
    ];

    for sketch in shapes {
        assert_eq!(sketch.material_contour_count(), 1);
        assert_eq!(sketch.hole_contour_count(), 0);
        assert!(!sketch.as_region().is_empty());
        assert!(!sketch.region_rings().material.is_empty());
    }
}

#[test]
#[cfg(feature = "metaballs")]
fn sketch_metaballs_start_as_hypercurve_regions() {
    let balls = [(Point2::new(0.0, 0.0), 1.0), (Point2::new(0.55, 0.0), 1.0)];

    let mut sketch = Sketch::metaballs(&balls, (24, 24), 1.0, 0.35, ());

    assert!(!sketch.as_region().is_empty());
    assert!(!sketch.region_rings().material.is_empty());

    sketch.geometry = GeometryCollection::default();
    assert!(!sketch.geometry().is_empty());
    assert!(!sketch.triangulate().is_empty());
}

#[test]
fn sketch_boolean_difference_keeps_region_hole() {
    let outer = Sketch::rectangle(10.0, 10.0, ());
    let inner = Sketch::rectangle(4.0, 4.0, ()).translate(3.0, 3.0, 0.0);

    let donut = outer.difference(&inner);

    assert_eq!(donut.material_contour_count(), 1);
    assert_eq!(donut.hole_contour_count(), 1);
}

#[test]
fn sketch_booleans_use_region_when_geo_cache_is_empty() {
    let mut left = Sketch::rectangle(4.0, 4.0, ());
    let mut right = Sketch::rectangle(4.0, 4.0, ()).translate(1.5, 0.75, 0.0);
    left.geometry = GeometryCollection::default();
    right.geometry = GeometryCollection::default();

    let union = left.union(&right);
    let difference = left.difference(&right);
    let intersection = left.intersection(&right);
    let xor = left.xor(&right);

    assert!(!union.as_region().is_empty());
    assert!(!difference.as_region().is_empty());
    assert!(!intersection.as_region().is_empty());
    assert!(!xor.as_region().is_empty());
    assert_eq!(intersection.material_contour_count(), 1);
}

#[test]
fn sketch_boolean_shared_boundary_falls_back_from_region_uncertainty() {
    let mut left = Sketch::rectangle(4.0, 4.0, ());
    let mut right = Sketch::rectangle(4.0, 4.0, ()).translate(2.0, 0.0, 0.0);
    left.geometry = GeometryCollection::default();
    right.geometry = GeometryCollection::default();

    let union = left.union(&right);
    let intersection = left.intersection(&right);

    assert!(!union.as_region().is_empty());
    assert!(!intersection.as_region().is_empty());
}

#[test]
fn sketch_region_feeds_legacy_triangulation_graphics_bounds_and_transform() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    assert_eq!(sketch.triangulate().len(), 2);
    assert_eq!(sketch.build_graphic_line_strings().total_line_count(), 4);
    let bbox = sketch.bounding_box();
    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 2.0, 1e-9);
    assert_close(bbox.maxs.y, 3.0, 1e-9);

    let moved = sketch.translate(5.0, 7.0, 0.0);
    assert_eq!(moved.material_contour_count(), 1);
    assert!(!moved.region_rings().material.is_empty());
    let moved_bbox = moved.bounding_box();
    assert_close(moved_bbox.mins.x, 5.0, 1e-9);
    assert_close(moved_bbox.mins.y, 7.0, 1e-9);
    assert_close(moved_bbox.maxs.x, 7.0, 1e-9);
    assert_close(moved_bbox.maxs.y, 10.0, 1e-9);
}

#[test]
fn sketch_region_transform_does_not_need_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    let transformed = sketch.scale(2.0, 3.0, 1.0).translate(-1.0, 4.0, 0.0);

    assert_eq!(transformed.material_contour_count(), 1);
    assert!(transformed.contains_xy(1.0, 5.0).unwrap());
    assert!(!transformed.contains_xy(4.0, 5.0).unwrap());
    let bbox = transformed.bounding_box();
    assert_close(bbox.mins.x, -1.0, 1e-9);
    assert_close(bbox.mins.y, 4.0, 1e-9);
    assert_close(bbox.maxs.x, 3.0, 1e-9);
    assert_close(bbox.maxs.y, 13.0, 1e-9);
}

#[test]
fn sketch_region_transform_ignores_stale_non_area_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let moved = sketch.translate(5.0, 7.0, 0.0);

    assert_eq!(moved.material_contour_count(), 1);
    assert!(moved.contains_xy(5.5, 7.5).unwrap());
    let bbox = moved.bounding_box();
    assert_close(bbox.mins.x, 5.0, 1e-9);
    assert_close(bbox.mins.y, 7.0, 1e-9);
    assert_close(bbox.maxs.x, 7.0, 1e-9);
    assert_close(bbox.maxs.y, 10.0, 1e-9);
}

#[test]
fn sketch_bounding_box_uses_hypercurve_region_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(6.0, 5.0, ())
        .difference(&Sketch::rectangle(2.0, 2.0, ()).translate(2.0, 1.0, 0.0));
    sketch.geometry = GeometryCollection::default();

    let bbox = sketch.bounding_box();

    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 6.0, 1e-9);
    assert_close(bbox.maxs.y, 5.0, 1e-9);
    assert_close(bbox.mins.z, 0.0, 1e-9);
    assert_close(bbox.maxs.z, 0.0, 1e-9);
}

#[test]
fn sketch_region_feeds_revolve_sweep_and_hilbert_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(1.0, 2.0, ()).translate(1.0, 0.0, 0.0);
    sketch.geometry = GeometryCollection::default();

    let revolved = sketch.revolve(180.0, 8).expect("region-backed revolve");
    assert!(!revolved.polygons.is_empty());

    let swept = sketch.sweep(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(0.5, 0.0, 2.0),
    ]);
    assert!(!swept.polygons.is_empty());

    let path = sketch.hilbert_curve(2, 0.0);
    assert!(!path.geometry().0.is_empty());
}

#[test]
#[cfg(feature = "offset")]
fn sketch_region_feeds_offset_and_skeleton_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    let sharp = sketch.offset(0.25);
    let rounded = sketch.offset_rounded(0.25);
    let skeleton = sketch.straight_skeleton(true);

    assert!(!sharp.as_region().is_empty());
    assert!(!rounded.as_region().is_empty());
    assert!(!skeleton.geometry().0.is_empty());
}

#[test]
#[cfg(feature = "offset")]
fn sketch_offset_results_recompose_as_native_hyper_geometry() {
    let sketch = Sketch::rectangle(2.0, 3.0, ());

    let mut sharp = sketch.offset(0.25);
    let mut rounded = sketch.offset_rounded(0.25);
    let mut skeleton = sketch.straight_skeleton(true);

    assert!(!sharp.as_region().is_empty());
    assert!(!rounded.as_region().is_empty());
    assert!(skeleton.as_region().is_empty());
    assert!(!skeleton.wires().is_empty());

    sharp.geometry = GeometryCollection::default();
    rounded.geometry = GeometryCollection::default();
    skeleton.geometry = GeometryCollection::default();

    assert!(!sharp.geometry().is_empty());
    assert!(!rounded.geometry().is_empty());
    assert!(!skeleton.geometry().is_empty());
    assert!(
        skeleton
            .geometry()
            .0
            .iter()
            .all(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
        "straight skeleton compatibility cache should regenerate from CurveString2 wires"
    );
}

#[test]
#[cfg(feature = "offset")]
fn sketch_mixed_straight_skeleton_preserves_native_wires_over_stale_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[4.0, 0.0], [5.0, 1.0], [6.0, 0.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let mut skeleton = sketch.straight_skeleton(true);

    assert!(skeleton.as_region().is_empty());
    assert!(!skeleton.wires().is_empty());
    assert!(
        skeleton
            .wire_polylines()
            .iter()
            .flatten()
            .any(|point| point[0] >= 6.0),
        "native open wire should survive straight skeleton despite stale finite cache"
    );

    skeleton.geometry = GeometryCollection::default();
    let geometry = skeleton.geometry();
    assert!(
        geometry.0.iter().all(|geometry| {
            !matches!(
                geometry,
                geo::Geometry::LineString(line) if line.0.iter().any(|coord| coord.x >= 100.0)
            )
        }),
        "stale finite sidecar cache should not survive skeleton recomposition"
    );
}

#[test]
#[cfg(feature = "offset")]
fn sketch_mixed_offset_preserves_native_wires_without_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[4.0, 0.0], [5.0, 1.0], [6.0, 0.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = GeometryCollection::default();

    let mut sharp = sketch.offset(0.25);
    let mut rounded = sketch.offset_rounded(0.25);

    for offset in [&mut sharp, &mut rounded] {
        assert!(!offset.as_region().is_empty());
        assert_eq!(
            offset.wires().len(),
            1,
            "area offset should preserve independent native open wires"
        );
        assert!(
            offset
                .wire_polylines()
                .iter()
                .flatten()
                .any(|point| point[0] >= 6.0),
            "preserved wire extent should remain available"
        );

        offset.geometry = GeometryCollection::default();
        let geometry = offset.geometry();
        assert!(
            geometry
                .0
                .iter()
                .any(|geometry| matches!(geometry, geo::Geometry::MultiPolygon(_))),
            "offset area should regenerate from Region2"
        );
        assert!(
            geometry
                .0
                .iter()
                .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
            "preserved offset wire should regenerate from CurveString2"
        );
    }
}

#[test]
#[cfg(feature = "offset")]
fn sketch_mixed_offset_preserves_native_wires_over_stale_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, ());
    let wire = Sketch::bezier(&[[4.0, 0.0], [5.0, 1.0], [6.0, 0.0]], 8, ());
    let mut sketch =
        Sketch::from_region_and_wires(area.as_region().clone(), wire.wires().to_vec(), ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let mut sharp = sketch.offset(0.25);
    let mut rounded = sketch.offset_rounded(0.25);

    for offset in [&mut sharp, &mut rounded] {
        assert!(!offset.as_region().is_empty());
        assert_eq!(
            offset.wires().len(),
            1,
            "native open wires must survive offset even when the finite cache is stale"
        );
        assert!(
            offset
                .wire_polylines()
                .iter()
                .flatten()
                .any(|point| point[0] >= 6.0),
            "preserved wire should come from native CurveString2, not stale geometry"
        );

        offset.geometry = GeometryCollection::default();
        let geometry = offset.geometry();
        assert!(
            geometry
                .0
                .iter()
                .any(|geometry| matches!(geometry, geo::Geometry::LineString(_))),
            "preserved wire should regenerate into the finite compatibility projection"
        );
    }
}

#[test]
#[cfg(feature = "offset")]
fn sketch_sharp_offset_uses_native_hypercurve_for_simple_material_region() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    let mut offset = sketch.offset(0.25);

    assert_eq!(offset.material_contour_count(), 1);
    assert_eq!(offset.hole_contour_count(), 0);
    assert!(offset.contains_xy(-0.125, 1.5).unwrap());
    assert!(!offset.contains_xy(-0.375, 1.5).unwrap());

    offset.geometry = GeometryCollection::default();
    let bbox = offset.bounding_box();
    assert_close(bbox.mins.x, -0.25, 1e-9);
    assert_close(bbox.mins.y, -0.25, 1e-9);
    assert_close(bbox.maxs.x, 2.25, 1e-9);
    assert_close(bbox.maxs.y, 3.25, 1e-9);
    assert!(!offset.triangulate().is_empty());
}

#[test]
#[cfg(feature = "offset")]
fn sketch_offset_ignores_stale_area_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = Sketch::rectangle(100.0, 100.0, ()).geometry().into_owned();

    let sharp = sketch.offset(0.25);
    let rounded = sketch.offset_rounded(0.25);

    for offset in [sharp, rounded] {
        let bbox = offset.bounding_box();
        assert!(bbox.maxs.x < 3.0, "offset used stale wide cache: {bbox:?}");
        assert!(bbox.maxs.y < 4.0, "offset used stale tall cache: {bbox:?}");
    }
}

#[test]
#[cfg(feature = "offset")]
fn sketch_sharp_offset_ignores_stale_non_area_geo_cache() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let offset = sketch.offset(0.25);

    assert_eq!(offset.material_contour_count(), 1);
    assert_eq!(offset.hole_contour_count(), 0);
    let bbox = offset.bounding_box();
    assert_close(bbox.mins.x, -0.25, 1e-9);
    assert_close(bbox.mins.y, -0.25, 1e-9);
    assert_close(bbox.maxs.x, 2.25, 1e-9);
    assert_close(bbox.maxs.y, 3.25, 1e-9);
}

#[test]
fn sketch_region_feeds_exporters_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry = GeometryCollection::default();

    #[cfg(feature = "svg-io")]
    {
        use crate::io::svg::ToSVG;
        let svg = sketch.to_svg();
        assert!(svg.contains("<path"));
    }

    #[cfg(feature = "gerber-io")]
    {
        use crate::io::gerber::ToGerber;
        let gerber = sketch.to_gerber().expect("region-backed Gerber export");
        assert!(!gerber.is_empty());
    }
}

#[test]
#[cfg(feature = "gerber-io")]
fn sketch_gerber_prefers_native_region_over_stale_non_area_geo_cache() {
    use crate::io::gerber::ToGerber;

    let mut sketch = Sketch::rectangle(2.0, 3.0, ());
    sketch.geometry =
        GeometryCollection(vec![geo::Geometry::Point(geo::Point::new(50.0, 50.0))]);

    let gerber = String::from_utf8(sketch.to_gerber().expect("native Gerber export"))
        .expect("Gerber output is UTF-8");

    assert!(
        gerber.contains("G36*"),
        "missing native region start: {gerber}"
    );
    assert!(gerber.contains("G37*"), "missing native region end: {gerber}");
    assert!(
        !gerber.contains("50000000"),
        "stale point coordinates should not leak into Gerber: {gerber}"
    );
}

#[test]
fn sketch_extrude_uses_region_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, "rect");
    sketch.geometry = GeometryCollection::default();

    let mesh = sketch.extrude_vector(Vector3::new(0.0, 0.0, 4.0));

    assert_eq!(mesh.metadata, "rect");
    assert!(!mesh.polygons.is_empty());
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert_close(bbox.mins.z, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 2.0, 1e-9);
    assert_close(bbox.maxs.y, 3.0, 1e-9);
    assert_close(bbox.maxs.z, 4.0, 1e-9);
}

#[test]
fn sketch_open_wire_extrude_uses_native_wires_when_geo_cache_is_empty() {
    let mut sketch = Sketch::bezier(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]], 8, "wire");
    sketch.geometry = GeometryCollection::default();

    let mesh = sketch.extrude_vector(Vector3::new(0.0, 0.0, 2.0));

    assert_eq!(mesh.metadata, "wire");
    assert!(!mesh.polygons.is_empty());
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.z, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 2.0, 1e-9);
    assert_close(bbox.maxs.z, 2.0, 1e-9);
}

#[test]
fn sketch_mixed_extrude_preserves_native_wires_over_stale_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, "mixed");
    let wire = Sketch::bezier(&[[5.0, 0.0], [6.0, 2.0], [7.0, 0.0]], 8, "mixed");
    let mut sketch = Sketch::from_region_and_wires(
        area.as_region().clone(),
        wire.wires().to_vec(),
        "mixed",
    );
    sketch.geometry = Sketch::rectangle(100.0, 100.0, "stale")
        .geometry()
        .into_owned();

    let mesh = sketch.extrude_vector(Vector3::new(0.0, 0.0, 2.0));

    assert_eq!(mesh.metadata, "mixed");
    assert!(
        mesh.polygons.len() > 8,
        "mixed native extrusion should include capped region polygons and wire side strips"
    );
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert_close(bbox.mins.z, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 7.0, 1e-9);
    assert_close(bbox.maxs.z, 2.0, 1e-9);
    assert!(
        bbox.maxs.y <= 3.0 + 1e-9,
        "stale 100x100 compatibility cache should not drive extrusion bounds"
    );
}

#[test]
fn sketch_region_extrude_preserves_origin() {
    let mut sketch = Sketch::rectangle(2.0, 2.0, ());
    sketch.geometry = GeometryCollection::default();
    sketch.set_origin(crate::vertex::Vertex::new(
        Point3::new(10.0, 20.0, 30.0),
        Vector3::z(),
    ));

    let mesh = sketch.extrude_vector(Vector3::new(0.0, 0.0, 2.0));

    assert!(!mesh.polygons.is_empty());
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.x, 10.0, 1e-9);
    assert_close(bbox.mins.y, 20.0, 1e-9);
    assert_close(bbox.mins.z, 30.0, 1e-9);
    assert_close(bbox.maxs.z, 32.0, 1e-9);
}

#[test]
fn sketch_sweep_uses_region_holes_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(4.0, 4.0, "profile")
        .difference(&Sketch::rectangle(1.0, 1.0, "cut").translate(1.0, 1.0, 0.0));
    sketch.geometry = GeometryCollection::default();

    let mesh = sketch.sweep(&[Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 3.0)]);

    assert_eq!(mesh.metadata, "profile");
    assert!(!mesh.polygons.is_empty());
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.z, 0.0, 1e-9);
    assert_close(bbox.maxs.z, 3.0, 1e-9);
}

#[test]
fn sketch_mixed_sweep_preserves_native_wires_over_stale_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, "mixed");
    let wire = Sketch::bezier(&[[5.0, 0.0], [6.0, 2.0], [7.0, 0.0]], 8, "mixed");
    let mut sketch = Sketch::from_region_and_wires(
        area.as_region().clone(),
        wire.wires().to_vec(),
        "mixed",
    );
    sketch.geometry = Sketch::rectangle(100.0, 100.0, "stale")
        .geometry()
        .into_owned();

    let mesh = sketch.sweep(&[Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 2.0)]);

    assert_eq!(mesh.metadata, "mixed");
    assert!(
        mesh.polygons.len() > 12,
        "mixed native sweep should include region walls, caps, and open wire strips"
    );
    let bbox = mesh.bounding_box();
    assert_close(bbox.mins.x, 0.0, 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert_close(bbox.mins.z, 0.0, 1e-9);
    assert_close(bbox.maxs.x, 7.0, 1e-9);
    assert_close(bbox.maxs.z, 2.0, 1e-9);
    assert!(
        bbox.maxs.y <= 3.0 + 1e-9,
        "stale 100x100 compatibility cache should not drive sweep bounds"
    );
}

#[test]
fn sketch_revolve_uses_region_holes_when_geo_cache_is_empty() {
    let mut sketch = Sketch::rectangle(2.0, 3.0, "profile")
        .difference(&Sketch::rectangle(0.5, 0.5, "cut").translate(0.5, 0.5, 0.0));
    sketch.geometry = GeometryCollection::default();

    let mesh = sketch.revolve(180.0, 8).expect("region-backed holed revolve");

    assert_eq!(mesh.metadata, "profile");
    assert!(!mesh.polygons.is_empty());
    let bbox = mesh.bounding_box();
    assert!(bbox.maxs.y >= 3.0 - 1e-9);
}

#[test]
fn sketch_mixed_revolve_preserves_native_wires_over_stale_geo_cache() {
    let area = Sketch::rectangle(2.0, 3.0, "mixed").translate(1.0, 0.0, 0.0);
    let wire = Sketch::bezier(&[[5.0, 0.0], [6.0, 2.0], [7.0, 0.0]], 8, "mixed");
    let mut sketch = Sketch::from_region_and_wires(
        area.as_region().clone(),
        wire.wires().to_vec(),
        "mixed",
    );
    sketch.geometry = Sketch::rectangle(100.0, 100.0, "stale")
        .geometry()
        .into_owned();

    let mesh = sketch
        .revolve(180.0, 8)
        .expect("mixed native revolve should build from hypercurve topology");

    assert_eq!(mesh.metadata, "mixed");
    assert!(
        mesh.polygons.len() > 80,
        "mixed native revolve should include region and open wire revolution strips"
    );
    let bbox = mesh.bounding_box();
    assert!(bbox.mins.x <= -7.0 + 1e-9);
    assert!(bbox.maxs.x >= 7.0 - 1e-9);
    assert!(bbox.maxs.z >= 7.0 - 1e-9);
    assert_close(bbox.mins.y, 0.0, 1e-9);
    assert!(
        bbox.maxs.y <= 3.0 + 1e-9,
        "stale 100x100 compatibility cache should not drive revolve bounds"
    );
}
