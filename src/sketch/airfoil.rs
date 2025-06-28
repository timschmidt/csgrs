use crate::float_types::Real;
use crate::sketch::sketch::Sketch;
use geo::{
    Geometry, GeometryCollection, LineString, Orient,
    Polygon as GeoPolygon, orient::Direction,
};

impl<S: Clone + Send + Sync> Sketch<S> {
    /// Generate a NACA 4-digit airfoil (e.g. "2412", "0015").
    ///
    /// ## Parameters
    /// - `chord` – physical chord length you want (same units as the rest of your model)
    /// - `samples` – number of points per surface (≥ 10 is required; NP total = 2 × samples + 1)
    /// - `max_camber` - max camber %, the first digit
    /// - `camber_pos` - camber position, the second digit
    /// - `thickness` - thickness %, the last two digits
    ///
    /// The function returns a single closed polygon lying in the *XY* plane with its
    /// leading edge at the origin and the chord running along +X.
    pub fn naca_exact(
        chord: Real, samples: usize, metadata: Option<S>,
        max_camber: Real, camber_pos: Real, thickness: Real
    ) -> Sketch<S> {
        let m = max_camber;
        let p = camber_pos;
        let tt = thickness;

        // thickness half-profile -----------------------------------------------
        let half_profile = |x: Real| -> Real {
            5.0 * tt
                * (0.2969 * x.sqrt() - 0.1260 * x - 0.3516 * x * x + 0.2843 * x * x * x
                    - 0.1015 * x * x * x * x)
        };

        // mean-camber line & slope ---------------------------------------------
        let camber = |x: Real| -> (Real, Real) {
            if x < p {
                let yc = m / (p * p) * (2.0 * p * x - x * x);
                let dy = 2.0 * m / (p * p) * (p - x);
                (yc, dy)
            } else {
                let yc = m / ((1.0 - p).powi(2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x * x);
                let dy = 2.0 * m / ((1.0 - p).powi(2)) * (p - x);
                (yc, dy)
            }
        };

        // --- sample upper & lower surfaces ------------------------------------
        let n = samples as Real;
        let mut coords: Vec<(Real, Real)> = Vec::with_capacity(2 * samples + 1);

        // leading-edge → trailing-edge (upper)
        for i in 0..=samples {
            let xc = i as Real / n; // 0–1
            let x = xc * chord; // physical
            let t = half_profile(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();

            let xu = x - t * theta.sin();
            let yu = chord * (yc_val + t * theta.cos());
            coords.push((xu, yu));
        }

        // trailing-edge → leading-edge (lower)
        for i in (1..samples).rev() {
            let xc = i as Real / n;
            let x = xc * chord;
            let t = half_profile(xc);
            let (yc_val, dy) = camber(xc);
            let theta = dy.atan();

            let xl = x + t * theta.sin();
            let yl = chord * (yc_val - t * theta.cos());
            coords.push((xl, yl));
        }

        coords.push(coords[0]); // close

        let polygon_2d = GeoPolygon::new(LineString::from(coords), vec![])
            .orient(Direction::Default);
        Sketch::from_geo(
            GeometryCollection(vec![Geometry::Polygon(polygon_2d)]),
            metadata,
        )
    }

    /// Generate a NACA 4-digit airfoil (e.g. "2412", "0015").
    ///
    /// ## Parameters
    /// * `code` – 4 ASCII digits describing **camber**, **camber-pos**, **thickness**  
    /// * `chord` – physical chord length you want (same units as the rest of your model)  
    /// * `samples` – number of points per surface (≥ 10 is required; NP total = 2 × samples + 1)  
    ///
    /// The function returns a single closed polygon lying in the *XY* plane with its
    /// leading edge at the origin and the chord running along +X.
    pub fn airfoil(code: &str, chord: Real, samples: usize, metadata: Option<S>) -> Sketch<S> {
        assert!(
            code.len() == 4 && code.chars().all(|c| c.is_ascii_digit()),
            "NACA code must be exactly 4 digits"
        );
        assert!(samples >= 10, "Need at least 10 points per surface");

        // -------- decode code ----------------------------------------------
        let m = code[0..1].parse::<Real>().unwrap() / 100.0; // max-camber %
        let p = code[1..2].parse::<Real>().unwrap() / 10.0; // camber-pos
        let tt = code[2..4].parse::<Real>().unwrap() / 100.0; // thickness %

        Self::naca_exact(
            chord, samples, metadata,
            m, p, tt
        )
    }
}
