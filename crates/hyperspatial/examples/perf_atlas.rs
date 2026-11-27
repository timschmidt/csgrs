//! Performance atlas for Hyperspatial: exact geodesy, map projections and
//! local frames. Covers geodetic/ECEF conversion in both directions, the
//! closest-point inverse near its degenerate sets, ENU frames and chained
//! round trips, spherical, ellipsoidal and transverse Mercator in both
//! directions, and UTM zone selection.
//!
//! Axes: point count (atomic batch length), distance to degeneracy (`2^-k`)
//! near the polar axis, the oblate medial disk, the centre, the Mercator pole
//! and the transverse chart edge, coordinate fraction bits, northing and
//! easting magnitude, and chained round trips. Input kinds are the angle
//! representations a survey or CAD import produces (binary64 radians, whole
//! degrees, seven-decimal degrees, rational radians), Cartesian
//! representations (integer metres, binary64 metres, irrational round-trip
//! output) and the reference surface (WGS84 oblate, sphere, prolate).
//!
//! Every timed call materializes its results as `f64` through the crate's
//! lossy display boundary, so lazily represented coordinates are evaluated.
//! With `--resources`, results report their exact coordinate structure:
//! `nodes` sums the expression nodes of every output coordinate, `depth` is
//! the deepest coordinate expression and `bits` the largest stored rational.
//! UTM selections report the selection count and the deepest evidence
//! receipt instead.
//!
//! Run through `scripts/perf-atlas/run.sh`, or directly:
//! `cargo run --release -p hyperspatial --all-features --example perf_atlas -- --out atlas.json [--quick] [--resources]`.
use std::f64::consts::PI;
use std::sync::Arc;

use hyperlattice::Point3;
use hyperreal::Rational;
use hyperspatial::{
    Angle, Context, EcefPosition, Ellipsoid, EllipsoidalMercator, GeodeticPosition, InputAccuracy,
    InputProvenance, Latitude, Length, Limits, LocalAxes, LocalFrame, Longitude, LongitudePolicy,
    MercatorParameters, PoleMeridian, PredicatePolicy, ProjectedPosition, Real, ReferenceFrame,
    SphericalMercator, SphericalTransverseMercator, TransverseLongitudePolicy,
    TransverseMercatorParameters, UtmSelection, ecef_to_geodetic_batch, geodetic_to_ecef_batch,
};
use perf_atlas::{Atlas, Checked, Rng, Shape, Shaped, powers_of_two};

/// Heap accounting for `--resources` runs; only this atlas binary installs it.
#[global_allocator]
static ALLOCATOR: perf_atlas::CountingAllocator = perf_atlas::CountingAllocator;

/// WGS84 equatorial radius in metres, for generating Cartesian inputs.
const A: f64 = 6_378_137.0;

/// Angle representations of geodetic inputs.
const ANGLE_KINDS: [&str; 4] = [
    "binary64 radians",
    "whole degrees",
    "7-decimal degrees",
    "rational radians",
];

/// A strict context whose step and batch limits do not end a sweep early.
/// The receipt depth limit keeps its default, so chained round trips meet it.
fn context() -> Context {
    Context::new(
        PredicatePolicy::STRICT,
        Limits {
            max_steps: 1 << 40,
            max_batch_len: 1 << 24,
            ..Limits::default()
        },
    )
}

fn reference() -> ReferenceFrame {
    ReferenceFrame::fixed("perf-atlas:static-earth").expect("valid frame name")
}

fn provenance() -> InputProvenance {
    InputProvenance::new("perf atlas input", InputAccuracy::ExactModel)
}

fn int(value: i64) -> Real {
    Real::from(value)
}

fn frac(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

fn dyadic(value: f64) -> Real {
    Real::try_from(value).expect("finite input")
}

/// `2^-k` as an exact real.
fn pow2_neg(k: u32) -> Real {
    let mut out = Real::one();
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        out = (out / int(1 << step)).expect("nonzero power of two");
        left -= step;
    }
    out
}

/// `2^k` as an exact real.
fn pow2(k: u32) -> Real {
    let mut out = Real::one();
    let mut left = k;
    while left > 0 {
        let step = left.min(62);
        out *= int(1 << step);
        left -= step;
    }
    out
}

fn surface(kind: &str) -> Arc<Ellipsoid> {
    let mut c = context();
    Arc::new(
        match kind {
            "sphere" => Ellipsoid::sphere(Length::metres(int(6_378_137)), &mut c),
            // A prolate surface 0.3% longer along the polar axis.
            "prolate" => Ellipsoid::from_axes(
                Length::metres(int(6_378_137)),
                Length::metres(int(6_397_271)),
                &mut c,
            ),
            _ => Ellipsoid::wgs84(&mut c),
        }
        .expect("valid ellipsoid"),
    )
}

/// An angle of about `degrees` in one representation.
fn angle(kind: &str, degrees: f64, rng: &mut Rng) -> Angle {
    match kind {
        "binary64 radians" => Angle::radians(dyadic(degrees.to_radians())),
        "whole degrees" => Angle::degrees(int(degrees.round() as i64)),
        "7-decimal degrees" => Angle::degrees(frac((degrees * 1e7).round() as i64, 10_000_000)),
        "rational radians" => {
            let q = rng.range(1 << 15, 1 << 16);
            Angle::radians(frac(
                (degrees.to_radians() * q as f64).round() as i64,
                q as u64,
            ))
        }
        other => unreachable!("unknown angle kind {other}"),
    }
}

/// A height of about `metres` in the representation family of `kind`.
fn height(kind: &str, metres: f64) -> Length {
    Length::metres(match kind {
        "binary64 radians" => dyadic(metres),
        "7-decimal degrees" => frac((metres * 1000.0).round() as i64, 1000),
        _ => int(metres.round() as i64),
    })
}

fn geodetic_at(
    kind: &str,
    latitude: f64,
    longitude: f64,
    metres: f64,
    surface: &Arc<Ellipsoid>,
    rng: &mut Rng,
) -> GeodeticPosition {
    let mut c = context();
    GeodeticPosition::new(
        Latitude::new(angle(kind, latitude, rng), &mut c).expect("latitude in range"),
        Longitude::unrolled(angle(kind, longitude, rng)),
        height(kind, metres),
        surface.clone(),
        reference(),
        provenance(),
        &mut c,
    )
    .expect("valid geodetic input")
}

/// A random terrestrial position: latitude within ±85°, longitude within
/// ±`lon` degrees, height between -100 m and 9 km.
fn geodetic(kind: &str, lon: f64, surface: &Arc<Ellipsoid>, rng: &mut Rng) -> GeodeticPosition {
    let latitude = (rng.unit() * 2.0 - 1.0) * 85.0;
    let longitude = (rng.unit() * 2.0 - 1.0) * lon;
    let metres = rng.unit() * 9100.0 - 100.0;
    geodetic_at(kind, latitude, longitude, metres, surface, rng)
}

fn geodetics(
    kind: &str,
    n: usize,
    lon: f64,
    surface: &Arc<Ellipsoid>,
    rng: &mut Rng,
) -> Vec<GeodeticPosition> {
    (0..n).map(|_| geodetic(kind, lon, surface, rng)).collect()
}

fn ecef(point: Point3) -> EcefPosition {
    EcefPosition::new(point, reference(), provenance(), &mut context()).expect("valid ECEF input")
}

/// A random Cartesian point near the surface, in one representation:
/// integer metres, binary64 metres, or the irrational output of a forward
/// conversion of a 7-decimal-degree position.
fn cartesian(kind: &str, surface: &Arc<Ellipsoid>, rng: &mut Rng) -> EcefPosition {
    if kind == "round trip (irrational)" {
        return geodetic("7-decimal degrees", 179.0, surface, rng)
            .to_ecef(&mut context())
            .expect("forward conversion");
    }
    let latitude = (rng.unit() * 2.0 - 1.0) * 1.45;
    let longitude = (rng.unit() * 2.0 - 1.0) * 3.1;
    let radius = A + rng.unit() * 9100.0 - 100.0;
    let xyz = [
        radius * latitude.cos() * longitude.cos(),
        radius * latitude.cos() * longitude.sin(),
        radius * latitude.sin(),
    ];
    let [x, y, z] = xyz.map(|v| {
        if kind.starts_with("binary64") {
            dyadic(v)
        } else {
            int(v.round() as i64)
        }
    });
    ecef(Point3::new(x, y, z))
}

// ------------------------------------------------------------------ shapes

/// Sum of expression nodes, deepest expression and largest stored rational.
fn reals_shape<'a>(values: impl Iterator<Item = &'a Real>) -> Shape {
    values.fold(Shape::default(), |shape, value| {
        let size = value.representation_size(1 << 20);
        Shape {
            nodes: shape.nodes + size.nodes as u64,
            depth: shape.depth.max(size.depth as u64),
            bits: shape.bits.max(size.max_rational_bits),
        }
    })
}

fn point_reals(point: &Point3) -> [&Real; 3] {
    [&point.x, &point.y, &point.z]
}

fn geodetic_reals(position: &GeodeticPosition) -> [&Real; 3] {
    [
        position.latitude().angle().as_radians(),
        position.longitude().angle().as_radians(),
        position.height().as_metres(),
    ]
}

fn ecef_shape(result: &Checked<Vec<EcefPosition>>) -> Option<Shape> {
    let values = result.as_ref().ok()?;
    Some(reals_shape(
        values.iter().flat_map(|p| point_reals(p.coordinates())),
    ))
}

fn one_ecef_shape(result: &Checked<EcefPosition>) -> Option<Shape> {
    let value = result.as_ref().ok()?;
    Some(reals_shape(point_reals(value.coordinates()).into_iter()))
}

fn geodetic_shape(result: &Checked<Vec<GeodeticPosition>>) -> Option<Shape> {
    let values = result.as_ref().ok()?;
    Some(reals_shape(values.iter().flat_map(geodetic_reals)))
}

fn projected_shape<P>(result: &Checked<Vec<ProjectedPosition<P>>>) -> Option<Shape> {
    let values = result.as_ref().ok()?;
    Some(reals_shape(
        values.iter().flat_map(|p| point_reals(p.coordinates())),
    ))
}

fn utm_shape(result: &Checked<Vec<UtmSelection>>) -> Option<Shape> {
    let values = result.as_ref().ok()?;
    Some(Shape {
        nodes: values.len() as u64,
        depth: values
            .iter()
            .map(|s| s.receipt().depth() as u64)
            .max()
            .unwrap_or(0),
        bits: 0,
    })
}

// ----------------------------------------------------------------- forcing

fn shown_point(point: &Point3) -> bool {
    point.to_f64_array_lossy().is_some()
}

fn shown_geodetic(position: &GeodeticPosition) -> bool {
    geodetic_reals(position)
        .iter()
        .all(|value| value.to_f64_lossy().is_some())
}

/// Materialize every result as `f64`; a failed call or display is marked.
fn forced<T>(result: hyperspatial::Result<Vec<T>>, shown: impl Fn(&T) -> bool) -> Checked<Vec<T>> {
    let values = result.map_err(|e| e.to_string())?;
    if values.iter().all(shown) {
        Ok(values)
    } else {
        Err("f64 display failed".into())
    }
}

fn forced_one<T>(result: hyperspatial::Result<T>, shown: impl Fn(&T) -> bool) -> Checked<T> {
    let value = result.map_err(|e| e.to_string())?;
    if shown(&value) {
        Ok(value)
    } else {
        Err("f64 display failed".into())
    }
}

// -------------------------------------------------------------- projections

fn web_mercator() -> SphericalMercator {
    SphericalMercator::web_mercator(reference(), LongitudePolicy::Unrolled, &mut context())
        .expect("valid projection")
}

fn ellipsoidal_mercator(kind: &str) -> EllipsoidalMercator {
    EllipsoidalMercator::new(
        surface(kind),
        reference(),
        MercatorParameters::greenwich(LongitudePolicy::Unrolled),
        &mut context(),
    )
    .expect("valid projection")
}

fn transverse() -> SphericalTransverseMercator {
    SphericalTransverseMercator::new(
        Length::metres(int(6_378_137)),
        reference(),
        TransverseMercatorParameters::greenwich(TransverseLongitudePolicy::RejectOutsideChart),
        &mut context(),
    )
    .expect("valid projection")
}

/// Projected coordinates for fresh inverse ingress: integer metres within
/// `|x| <= x_max`, `|y| <= y_max`, or the irrational output of a forward
/// projection of a 7-decimal-degree position.
fn projected_coordinates(
    kind: &str,
    x_max: f64,
    y_max: f64,
    forward: impl Fn(&GeodeticPosition) -> Point3,
    surface: &Arc<Ellipsoid>,
    lon: f64,
    rng: &mut Rng,
) -> Point3 {
    if kind == "round trip (irrational)" {
        return forward(&geodetic("7-decimal degrees", lon, surface, rng));
    }
    let x = (rng.unit() * 2.0 - 1.0) * x_max;
    let y = (rng.unit() * 2.0 - 1.0) * y_max;
    let h = rng.unit() * 9100.0 - 100.0;
    if kind.starts_with("binary64") {
        Point3::new(dyadic(x), dyadic(y), dyadic(h))
    } else {
        Point3::new(
            int(x.round() as i64),
            int(y.round() as i64),
            int(h.round() as i64),
        )
    }
}

// ---------------------------------------------------------------------- UTM

fn utm_position(kind: &str, sphere: &Arc<Ellipsoid>, rng: &mut Rng) -> GeodeticPosition {
    let (latitude, longitude): (Real, Real) = match kind {
        "uniform" => (
            frac(rng.range(-799_999, 839_999), 10_000),
            frac(rng.range(-1_800_000, 1_799_999), 10_000),
        ),
        // Norway (56°-64°N, 3°-12°E) and Svalbard (72°-84°N, 0°-42°E).
        "Norway/Svalbard" => {
            if rng.coin() {
                (
                    frac(rng.range(560_000, 639_999), 10_000),
                    frac(rng.range(30_000, 119_999), 10_000),
                )
            } else {
                (
                    frac(rng.range(720_000, 839_999), 10_000),
                    frac(rng.range(0, 419_999), 10_000),
                )
            }
        }
        "on zone boundaries" => (int(rng.range(-79, 83)), int(6 * rng.range(-30, 29))),
        "near boundaries (2^-40°)" => {
            let offset = pow2_neg(40) * int(if rng.coin() { 1 } else { -1 });
            (
                int(8 * rng.range(-9, 9)) + offset.clone(),
                int(6 * rng.range(-29, 29)) + offset,
            )
        }
        other => unreachable!("unknown UTM kind {other}"),
    };
    let mut c = context();
    GeodeticPosition::new(
        Latitude::new(Angle::degrees(latitude), &mut c).expect("latitude in range"),
        Longitude::unrolled(Angle::degrees(longitude)),
        Length::metres(Real::zero()),
        sphere.clone(),
        reference(),
        provenance(),
        &mut c,
    )
    .expect("valid geodetic input")
}

fn main() {
    let mut atlas = Atlas::from_args("hyperspatial");
    let counts = powers_of_two(0, 10);
    let small_counts = powers_of_two(0, 8);

    // ---------------------------------------------------------- geodesy

    atlas.operation(
        "geodesy",
        "geodetic to ECEF",
        "point count",
        "Atomic WGS84 geodetic-to-ECEF batch of random terrestrial positions, displayed as f64.",
    );
    for kind in ANGLE_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| geodetics(kind, n as usize, 179.0, &surface("WGS84"), rng),
            |positions| {
                Shaped::new(
                    forced(
                        geodetic_to_ecef_batch(positions, &mut context()),
                        |p: &EcefPosition| shown_point(p.coordinates()),
                    ),
                    ecef_shape,
                )
            },
        );
    }

    let cartesian_kinds = [
        ("integer metres", "WGS84"),
        ("binary64 metres", "WGS84"),
        ("round trip (irrational)", "WGS84"),
        ("integer metres, prolate", "prolate"),
    ];
    atlas.operation(
        "geodesy",
        "ECEF to geodetic",
        "point count",
        "Atomic closest-point inverse batch (Vermeille/Karney) of random near-surface points, displayed as f64.",
    );
    for (kind, shape) in cartesian_kinds {
        atlas.sweep(
            kind,
            &small_counts,
            |n, rng| {
                let s = surface(shape);
                let points: Vec<EcefPosition> =
                    (0..n as usize).map(|_| cartesian(kind, &s, rng)).collect();
                (points, s)
            },
            |(points, s)| {
                Shaped::new(
                    forced(
                        ecef_to_geodetic_batch(
                            points,
                            s.clone(),
                            PoleMeridian::Reject,
                            &mut context(),
                        ),
                        shown_geodetic,
                    ),
                    geodetic_shape,
                )
            },
        );
    }

    let ks: Vec<f64> = (0..=128).step_by(8).map(f64::from).collect();
    atlas.operation(
        "geodesy",
        "ECEF to geodetic near degeneracy",
        "distance to degeneracy (2^-k m)",
        "Closest-point inverse of one WGS84 point at 2^-k m from a degenerate set: the polar axis (a pole needs a meridian), the oblate equatorial medial disk (nonunique closest point) and the centre.",
    );
    for kind in [
        "near polar axis",
        "near medial disk",
        "near centre",
        "near medial disk rim",
    ] {
        atlas.sweep(
            kind,
            &ks,
            |k, _| {
                let s = surface("WGS84");
                let a = s.equatorial_radius().as_metres().clone();
                let b = s.polar_radius().as_metres().clone();
                let eps = pow2_neg(k as u32);
                // The medial disk has radius (a^2 - b^2) / a.
                let rim = ((&a * &a - &b * &b) / a.clone()).expect("positive radius");
                let point = match kind {
                    "near polar axis" => Point3::new(eps, Real::zero(), b + int(1000)),
                    "near medial disk" => {
                        Point3::new((rim / int(2)).expect("nonzero"), Real::zero(), eps)
                    }
                    "near centre" => Point3::new(eps.clone(), eps.clone(), eps),
                    _ => Point3::new(rim + eps.clone(), Real::zero(), eps),
                };
                (ecef(point), s)
            },
            |(point, s)| {
                Shaped::new(
                    forced(
                        point
                            .to_geodetic_closest(s.clone(), PoleMeridian::Reject, &mut context())
                            .map(|g| vec![g]),
                        shown_geodetic,
                    ),
                    geodetic_shape,
                )
            },
        );
    }

    let bits = powers_of_two(0, 10);
    let fraction = |kind: &str, b: u32, rng: &mut Rng| -> EcefPosition {
        let base = cartesian("integer metres", &surface("WGS84"), rng);
        let p = base.coordinates();
        let part = |rng: &mut Rng| -> Real {
            let numerator: Real = rng.decimal(b).parse().expect("decimal digits");
            if kind == "dyadic" {
                (numerator / pow2(b)).expect("nonzero")
            } else {
                let denominator: Real = rng.decimal(b.max(2)).parse().expect("decimal digits");
                (numerator / (denominator + Real::one())).expect("nonzero")
            }
        };
        ecef(Point3::new(
            p.x.clone() + part(rng),
            p.y.clone() + part(rng),
            p.z.clone() + part(rng),
        ))
    };
    atlas.operation(
        "geodesy",
        "ECEF to geodetic by coordinate bits",
        "coordinate fraction bits",
        "Closest-point inverse of one WGS84 point whose coordinates are integer metres plus a fraction with b-bit parts.",
    );
    for kind in ["dyadic", "rational"] {
        atlas.sweep(
            kind,
            &bits,
            |b, rng| fraction(kind, b as u32, rng),
            |point| {
                Shaped::new(
                    forced(
                        point
                            .to_geodetic_closest(
                                surface("WGS84"),
                                PoleMeridian::Reject,
                                &mut context(),
                            )
                            .map(|g| vec![g]),
                        shown_geodetic,
                    ),
                    geodetic_shape,
                )
            },
        );
    }
    atlas.fuzz(
        "mixed",
        &powers_of_two(0, 8),
        |m, rng| {
            let kind = if rng.coin() { "dyadic" } else { "rational" };
            let b = rng.range(1, m as i64) as u32;
            fraction(kind, b, rng)
        },
        |point| {
            forced_one(
                point.to_geodetic_closest(surface("WGS84"), PoleMeridian::Reject, &mut context()),
                shown_geodetic,
            )
            .is_ok()
        },
    );

    // ----------------------------------------------------------- local

    let anchor = |kind: &str, rng: &mut Rng| -> LocalFrame {
        let s = surface("WGS84");
        let origin = geodetic(kind, 179.0, &s, rng);
        LocalFrame::new(&origin, LocalAxes::Enu, &mut context()).expect("valid anchor")
    };
    let near = |frame: &LocalFrame, kind: &str, rng: &mut Rng| -> EcefPosition {
        let o = frame.origin().coordinates();
        let step = |rng: &mut Rng| -> Real {
            let metres = rng.range(-50_000, 50_000);
            if kind == "binary64 metres" {
                dyadic(metres as f64 + rng.unit())
            } else {
                int(metres)
            }
        };
        ecef(Point3::new(
            o.x.clone() + step(rng),
            o.y.clone() + step(rng),
            o.z.clone() + step(rng),
        ))
    };
    atlas.operation(
        "local",
        "ECEF to local (ENU)",
        "point count",
        "Rotate a batch of ECEF points within 50 km into a prepared ENU frame (anchor in each angle representation), displayed as f64.",
    );
    for kind in ANGLE_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let frame = anchor(kind, rng);
                let points: Vec<EcefPosition> = (0..n as usize)
                    .map(|_| near(&frame, "integer metres", rng))
                    .collect();
                (frame, points)
            },
            |(frame, points)| {
                let mut c = context();
                let mut out = Vec::with_capacity(points.len());
                for point in points {
                    match frame.to_local_position(point, &mut c) {
                        Ok(local) if shown_point(local.coordinates()) => out.push(local),
                        Ok(_) => return Err("f64 display failed".to_string()),
                        Err(e) => return Err(e.to_string()),
                    }
                }
                Ok(out.len())
            },
        );
    }
    let digits: Vec<f64> = (0..=16).step_by(2).map(f64::from).collect();
    atlas.grid(
        "decimal-degree anchor",
        ("anchor decimal digits", &digits),
        ("point fraction bits", &powers_of_two(0, 8)),
        |d, b, rng| {
            let scale = 10i64.pow(d as u32);
            let s = surface("WGS84");
            let latitude = frac(rng.range(-85 * scale, 85 * scale), scale as u64);
            let longitude = frac(rng.range(-179 * scale, 179 * scale), scale as u64);
            let mut c = context();
            let origin = GeodeticPosition::new(
                Latitude::new(Angle::degrees(latitude), &mut c).expect("latitude in range"),
                Longitude::unrolled(Angle::degrees(longitude)),
                Length::metres(int(100)),
                s,
                reference(),
                provenance(),
                &mut c,
            )
            .expect("valid anchor");
            let frame = LocalFrame::new(&origin, LocalAxes::Enu, &mut c).expect("valid anchor");
            let o = frame.origin().coordinates().clone();
            let part = |rng: &mut Rng| -> Real {
                let numerator: Real = rng.decimal(b as u32).parse().expect("decimal digits");
                int(rng.range(-50_000, 50_000)) + (numerator / pow2(b as u32)).expect("nonzero")
            };
            let point = ecef(Point3::new(
                o.x + part(rng),
                o.y + part(rng),
                o.z + part(rng),
            ));
            (frame, point)
        },
        |(frame, point)| {
            forced_one(
                frame
                    .to_local_position(point, &mut context())
                    .map(|l| l.coordinates().clone()),
                shown_point,
            )
            .is_ok()
        },
    );

    atlas.operation(
        "local",
        "ECEF/local round trips",
        "round trips",
        "Chain n ECEF->ENU->ECEF round trips through one anchor and display the final point. Each trip deepens the retained evidence receipt by two levels, so the default receipt depth limit (128) ends the chain.",
    );
    for kind in ["7-decimal degrees", "binary64 radians"] {
        atlas.sweep(
            kind,
            &powers_of_two(0, 7),
            |n, rng| {
                let frame = anchor(kind, rng);
                let point = near(&frame, "integer metres", rng);
                (frame, point, n as usize)
            },
            |(frame, point, n)| {
                let mut c = context();
                let mut current = point.clone();
                let chained = (|| {
                    for _ in 0..*n {
                        let local = frame.to_local_position(&current, &mut c)?;
                        current = frame.to_ecef_position(&local, &mut c)?;
                    }
                    Ok(current)
                })();
                Shaped::new(
                    forced_one(chained, |p: &EcefPosition| shown_point(p.coordinates())),
                    one_ecef_shape,
                )
            },
        );
    }

    // ------------------------------------------------------- projection

    atlas.operation(
        "projection",
        "spherical Mercator forward",
        "point count",
        "Atomic Web (pseudo-)Mercator projection batch, displayed as f64.",
    );
    for kind in ANGLE_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let projection = web_mercator();
                let points = geodetics(kind, n as usize, 179.0, projection.ellipsoid(), rng);
                (projection, points)
            },
            |(projection, points)| {
                Shaped::new(
                    forced(
                        projection.project_batch(points, &mut context()),
                        |p: &ProjectedPosition<SphericalMercator>| shown_point(p.coordinates()),
                    ),
                    projected_shape,
                )
            },
        );
    }

    let inverse_kinds = [
        "integer metres",
        "binary64 metres",
        "round trip (irrational)",
    ];
    atlas.operation(
        "projection",
        "spherical Mercator inverse",
        "point count",
        "Atomic Web (pseudo-)Mercator inverse batch of fresh projected inputs (no retained forward source), displayed as f64.",
    );
    for kind in inverse_kinds {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let projection = web_mercator();
                let mut c = context();
                let points: Vec<ProjectedPosition<SphericalMercator>> = (0..n as usize)
                    .map(|_| {
                        let xyz = projected_coordinates(
                            kind,
                            2.0e7,
                            2.0e7,
                            |g| {
                                projection
                                    .project(g, &mut context())
                                    .expect("forward")
                                    .coordinates()
                                    .clone()
                            },
                            projection.ellipsoid(),
                            179.0,
                            rng,
                        );
                        projection
                            .position(xyz, provenance(), &mut c)
                            .expect("projected input")
                    })
                    .collect();
                (projection, points)
            },
            |(projection, points)| {
                Shaped::new(
                    forced(
                        projection.unproject_batch(points, &mut context()),
                        shown_geodetic,
                    ),
                    geodetic_shape,
                )
            },
        );
    }

    atlas.operation(
        "projection",
        "ellipsoidal Mercator forward",
        "point count",
        "Atomic ellipsoidal Mercator projection batch of 7-decimal-degree positions on each surface, displayed as f64.",
    );
    for kind in ["WGS84", "sphere", "prolate"] {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let projection = ellipsoidal_mercator(kind);
                let points = geodetics(
                    "7-decimal degrees",
                    n as usize,
                    179.0,
                    projection.ellipsoid(),
                    rng,
                );
                (projection, points)
            },
            |(projection, points)| {
                Shaped::new(
                    forced(
                        projection.project_batch(points, &mut context()),
                        |p: &ProjectedPosition<EllipsoidalMercator>| shown_point(p.coordinates()),
                    ),
                    projected_shape,
                )
            },
        );
    }

    let ellipsoidal_inverse = |kind: &str, surface_kind: &str, n: usize, rng: &mut Rng| {
        let projection = ellipsoidal_mercator(surface_kind);
        let mut c = context();
        let points: Vec<ProjectedPosition<EllipsoidalMercator>> = (0..n)
            .map(|_| {
                let xyz = projected_coordinates(
                    kind,
                    2.0e7,
                    2.0e7,
                    |g| {
                        projection
                            .project(g, &mut context())
                            .expect("forward")
                            .coordinates()
                            .clone()
                    },
                    projection.ellipsoid(),
                    179.0,
                    rng,
                );
                projection
                    .position(xyz, provenance(), &mut c)
                    .expect("projected input")
            })
            .collect();
        (projection, points)
    };
    atlas.operation(
        "projection",
        "ellipsoidal Mercator inverse",
        "point count",
        "Atomic ellipsoidal Mercator inverse batch of fresh projected inputs: the native inverse isometric latitude, displayed as f64.",
    );
    for (kind, surface_kind) in [
        ("integer metres", "WGS84"),
        ("round trip (irrational)", "WGS84"),
        ("integer metres, sphere", "sphere"),
        ("integer metres, prolate", "prolate"),
    ] {
        atlas.sweep(
            kind,
            &small_counts,
            |n, rng| ellipsoidal_inverse(kind, surface_kind, n as usize, rng),
            |(projection, points)| {
                Shaped::new(
                    forced(
                        projection.unproject_batch(points, &mut context()),
                        shown_geodetic,
                    ),
                    geodetic_shape,
                )
            },
        );
    }

    let pole_ks: Vec<f64> = (0..=96).step_by(8).map(f64::from).collect();
    atlas.operation(
        "projection",
        "Mercator near pole",
        "distance to pole (2^-k rad)",
        "Project one position at latitude pi/2 - 2^-k (the projection's singular pole), displayed as f64.",
    );
    for kind in ["spherical", "ellipsoidal WGS84", "ellipsoidal prolate"] {
        atlas.sweep(
            kind,
            &pole_ks,
            |k, _| {
                let s = match kind {
                    "ellipsoidal prolate" => surface("prolate"),
                    _ => surface("WGS84"),
                };
                let mut c = context();
                let latitude = (Real::pi() / int(2)).expect("nonzero") - pow2_neg(k as u32);
                let position = GeodeticPosition::new(
                    Latitude::new(Angle::radians(latitude), &mut c).expect("latitude in range"),
                    Longitude::unrolled(Angle::degrees(int(10))),
                    Length::metres(int(100)),
                    s.clone(),
                    reference(),
                    provenance(),
                    &mut c,
                )
                .expect("valid position");
                (position, s)
            },
            |(position, s)| {
                let projected = if kind == "spherical" {
                    SphericalMercator::new(
                        s.clone(),
                        reference(),
                        MercatorParameters::greenwich(LongitudePolicy::Unrolled),
                        &mut context(),
                    )
                    .and_then(|p| p.project(position, &mut context()))
                    .map(|p| p.coordinates().clone())
                } else {
                    EllipsoidalMercator::new(
                        s.clone(),
                        reference(),
                        MercatorParameters::greenwich(LongitudePolicy::Unrolled),
                        &mut context(),
                    )
                    .and_then(|p| p.project(position, &mut context()))
                    .map(|p| p.coordinates().clone())
                };
                forced_one(projected, shown_point).is_ok()
            },
        );
    }
    atlas.fuzz(
        "spherical",
        &pole_ks,
        |k, rng| {
            let k = (k as i64 + rng.range(-4, 4)).max(0) as u32;
            let s = surface("WGS84");
            let mut c = context();
            let latitude = (Real::pi() / int(2)).expect("nonzero") - pow2_neg(k);
            let latitude = latitude - pow2_neg(k + 8) * int(rng.range(0, 255));
            let position = GeodeticPosition::new(
                Latitude::new(Angle::radians(latitude), &mut c).expect("latitude in range"),
                Longitude::unrolled(Angle::degrees(int(rng.range(-179, 179)))),
                Length::metres(int(100)),
                s,
                reference(),
                provenance(),
                &mut c,
            )
            .expect("valid position");
            (position, web_mercator())
        },
        |(position, projection)| {
            forced_one(
                projection
                    .project(position, &mut context())
                    .map(|p| p.coordinates().clone()),
                shown_point,
            )
            .is_ok()
        },
    );

    let northing_ks: Vec<f64> = (0..=64).step_by(4).map(f64::from).collect();
    atlas.operation(
        "projection",
        "ellipsoidal Mercator inverse by northing",
        "northing (2^k m)",
        "Invert one fresh ellipsoidal Mercator point with northing 2^k m: latitudes approach the pole doubly exponentially, and the inverse keeps an exact open-latitude proof.",
    );
    for kind in ["WGS84", "sphere", "prolate"] {
        atlas.sweep(
            kind,
            &northing_ks,
            |k, _| {
                let projection = ellipsoidal_mercator(kind);
                let point = projection
                    .position(
                        Point3::new(int(1_000_000), pow2(k as u32), int(100)),
                        provenance(),
                        &mut context(),
                    )
                    .expect("projected input");
                (projection, point)
            },
            |(projection, point)| {
                Shaped::new(
                    forced(
                        projection.unproject(point, &mut context()).map(|g| vec![g]),
                        shown_geodetic,
                    ),
                    geodetic_shape,
                )
            },
        );
    }

    atlas.operation(
        "projection",
        "transverse Mercator forward",
        "point count",
        "Atomic spherical transverse Mercator projection batch within ±80° of the central meridian, displayed as f64.",
    );
    for kind in ANGLE_KINDS {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let projection = transverse();
                let points = geodetics(kind, n as usize, 80.0, projection.ellipsoid(), rng);
                (projection, points)
            },
            |(projection, points)| {
                Shaped::new(
                    forced(
                        projection.project_batch(points, &mut context()),
                        |p: &ProjectedPosition<SphericalTransverseMercator>| {
                            shown_point(p.coordinates())
                        },
                    ),
                    projected_shape,
                )
            },
        );
    }

    atlas.operation(
        "projection",
        "transverse Mercator inverse",
        "point count",
        "Atomic spherical transverse Mercator inverse batch of fresh projected inputs, displayed as f64.",
    );
    for kind in inverse_kinds {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let projection = transverse();
                let mut c = context();
                // Northings stay inside the closed strip |y| <= pi R / 2.
                let points: Vec<ProjectedPosition<SphericalTransverseMercator>> = (0..n as usize)
                    .map(|_| {
                        let xyz = projected_coordinates(
                            kind,
                            1.5e7,
                            0.95 * PI / 2.0 * A,
                            |g| {
                                projection
                                    .project(g, &mut context())
                                    .expect("forward")
                                    .coordinates()
                                    .clone()
                            },
                            projection.ellipsoid(),
                            80.0,
                            rng,
                        );
                        projection
                            .position(xyz, provenance(), &mut c)
                            .expect("projected input")
                    })
                    .collect();
                (projection, points)
            },
            |(projection, points)| {
                Shaped::new(
                    forced(
                        projection.unproject_batch(points, PoleMeridian::Reject, &mut context()),
                        shown_geodetic,
                    ),
                    geodetic_shape,
                )
            },
        );
    }

    atlas.operation(
        "projection",
        "transverse Mercator near chart edge",
        "distance to edge (2^-k rad)",
        "Project one position at longitude difference pi/2 - 2^-k from the central meridian: on the equator this approaches the singular edge, where easting grows like k.",
    );
    for (kind, latitude) in [("equator", 0), ("latitude 30°", 30), ("latitude 89°", 89)] {
        atlas.sweep(
            kind,
            &pole_ks,
            |k, _| {
                let projection = transverse();
                let mut c = context();
                let longitude = (Real::pi() / int(2)).expect("nonzero") - pow2_neg(k as u32);
                let position = GeodeticPosition::new(
                    Latitude::new(Angle::degrees(int(latitude)), &mut c).expect("in range"),
                    Longitude::unrolled(Angle::radians(longitude)),
                    Length::metres(int(100)),
                    projection.ellipsoid().clone(),
                    reference(),
                    provenance(),
                    &mut c,
                )
                .expect("valid position");
                (projection, position)
            },
            |(projection, position)| {
                forced_one(
                    projection
                        .project(position, &mut context())
                        .map(|p| p.coordinates().clone()),
                    shown_point,
                )
                .is_ok()
            },
        );
    }

    atlas.operation(
        "utm",
        "UTM zone selection",
        "point count",
        "Atomic UTM zone and hemisphere selection batch, including the Norway/Svalbard exceptions, exact zone and band boundaries and points 2^-40° from them.",
    );
    for kind in [
        "uniform",
        "Norway/Svalbard",
        "on zone boundaries",
        "near boundaries (2^-40°)",
    ] {
        atlas.sweep(
            kind,
            &counts,
            |n, rng| {
                let s = surface("sphere");
                (0..n as usize)
                    .map(|_| utm_position(kind, &s, rng))
                    .collect::<Vec<_>>()
            },
            |positions| {
                Shaped::new(
                    UtmSelection::select_batch(positions, &mut context())
                        .map_err(|e| e.to_string()),
                    utm_shape,
                )
            },
        );
    }

    atlas.finish();
}
