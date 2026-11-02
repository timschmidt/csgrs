//! Complete sections of a native ring torus by non-coaxial analytic carriers.
//!
//! A ring torus is the tensor product of two rational circles: its major
//! circle and the meridian circle of its tube. Using the same four exact arcs
//! as the cylinder atlas for both circles gives sixteen rational biquadratic
//! patches whose controls and weights are rational in the torus frame. The torus is bounded, so no height bound is needed; each
//! patch is sectioned completely by the implicit/tensor route.

use super::curve_implicit::ImplicitCarrier;
use super::cylinder_atlas::{
    COARSE_TANGENTS, PatchContact, section_patches_with_adjacency, unit_arcs,
};
use super::*;

/// Intersects `torus` with `other` through the exact torus atlas.
pub(super) fn intersect(
    torus: &TorusSurface,
    other: ImplicitCarrier<'_>,
    torus_operand: SurfaceIntersectionOperand,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    let patches = atlas(torus)?;
    // Patch `major * n + minor` is the image of a product of closed arcs, and
    // the torus chart is injective on the circle product, so two patches meet
    // only when both arc pairs are equal or consecutive (with n >= 3 arcs):
    // along a shared edge when one pair is equal, else at one shared corner.
    let n = unit_arcs(COARSE_TANGENTS)?.len();
    debug_assert_eq!(patches.len(), n * n);
    let adjacent = move |first: usize, second: usize| patch_contact(n, first, second);
    section_patches_with_adjacency(patches, other, torus_operand, Some(&adjacent))
}

/// How patches `first` and `second` (index `major * n + minor`) of an atlas
/// with `n` arcs per circle meet: rows (v) follow the major arc and columns
/// (u) the minor arc, so patches in consecutive major arcs share a v boundary,
/// patches in consecutive minor arcs share a u boundary, and patches
/// consecutive in both share only a corner.
fn patch_contact(n: usize, first: usize, second: usize) -> PatchContact {
    let near = |a: usize, b: usize| {
        let gap = (a + n - b) % n;
        gap == 0 || gap == 1 || gap == n - 1
    };
    let major = (first / n, second / n);
    let minor = (first % n, second % n);
    if n < 3 {
        PatchContact::Edge
    } else if !near(major.0, major.1) || !near(minor.0, minor.1) {
        PatchContact::Disjoint
    } else if major.0 != major.1 && minor.0 != minor.1 {
        PatchContact::Corner
    } else if major.0 != major.1 {
        PatchContact::EdgeAlong(1)
    } else {
        PatchContact::EdgeAlong(0)
    }
}

/// `center + (R + r m.x)(a.x X + a.y Y) + r m.y Z` with weights
/// `w_major * w_minor`: the revolution of the rational meridian circle `m`
/// by the rational major circle `a`.
fn atlas(torus: &TorusSurface) -> GeometryResult<Vec<Surface>> {
    let arcs = unit_arcs(COARSE_TANGENTS)?;
    let frame = &torus.frame;
    let mut patches = Vec::with_capacity(arcs.len() * arcs.len());
    for major in &arcs {
        let major_controls = [&major.start, &major.middle, &major.end];
        let major_weights = [Real::one(), major.weight.clone(), Real::one()];
        for minor in &arcs {
            let minor_controls = [&minor.start, &minor.middle, &minor.end];
            let minor_weights = [Real::one(), minor.weight.clone(), Real::one()];
            let mut controls = Vec::with_capacity(3);
            let mut weights = Vec::with_capacity(3);
            for (a, wa) in major_controls.iter().zip(&major_weights) {
                let mut row = Vec::with_capacity(3);
                let mut row_weights = Vec::with_capacity(3);
                for (m, wm) in minor_controls.iter().zip(&minor_weights) {
                    let radial = &torus.major_radius + &torus.minor_radius * &m.0;
                    let axial = &torus.minor_radius * &m.1;
                    row.push(
                        torus.center.clone()
                            + frame.x.clone() * (&radial * &a.0)
                            + frame.y.clone() * (&radial * &a.1)
                            + frame.z.clone() * axial,
                    );
                    row_weights.push(wa * wm);
                }
                controls.push(row);
                weights.push(row_weights);
            }
            patches.push(Surface::rational_bezier(controls, weights)?);
        }
    }
    Ok(patches)
}

/// Whether an existing coaxial special route owns the pair: a sphere centred
/// on the torus axis, or a cone or torus with parallel axis on the same line.
pub(super) fn special_route_applies(
    torus: &TorusSurface,
    other: ImplicitCarrier<'_>,
) -> GeometryResult<bool> {
    let zero = |value: &Real| -> GeometryResult<bool> {
        Ok(decided_order(compare_reals(
            value,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? == Ordering::Equal)
    };
    let axis = &torus.frame.z;
    let on_axis = |point: &Point3| -> GeometryResult<bool> {
        let offset = point - &torus.center;
        let radial = &offset - &(axis.clone() * axis.dot(&offset));
        zero(&radial.norm_squared())
    };
    let parallel = |direction: &Vector3| zero(&axis.cross(direction).norm_squared());
    Ok(match other {
        ImplicitCarrier::Sphere(sphere) => on_axis(&sphere.center)?,
        // Coaxial pairs keep the meridian-circle route; every other pair is
        // sectioned through the atlas (see the torus atlas witnesses for the
        // measured release envelope).
        ImplicitCarrier::Torus(other) => parallel(&other.frame.z)? && on_axis(&other.center)?,
        ImplicitCarrier::Cone(cone) => parallel(&cone.frame.z)? && on_axis(&cone.apex)?,
        // Cylinders are charted by the cylinder atlas.
        ImplicitCarrier::Cylinder(_) => true,
    })
}

#[cfg(test)]
mod contact_tests {
    use super::*;

    /// The contact table agrees with the actual atlas controls: shared edges
    /// share a whole boundary control row (v) or column (u), corner contacts
    /// share a corner control point but no edge, and disjoint patches share
    /// no corner control point.
    #[test]
    fn patch_contacts_match_the_atlas_controls() {
        let surface = Surface::torus(
            Point3::origin(),
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            3.into(),
            1.into(),
        )
        .unwrap();
        let SurfaceGeometry::Torus(torus) = &surface.data.geometry else {
            unreachable!("a torus surface");
        };
        let patches = atlas(torus).unwrap();
        let n = unit_arcs(COARSE_TANGENTS).unwrap().len();
        assert_eq!(patches.len(), n * n);
        let controls = |index: usize| {
            let SurfaceGeometry::RationalBezier(patch) = &patches[index].data.geometry else {
                unreachable!("atlas patches are rational Bézier");
            };
            patch.control_points.clone()
        };
        let equal = |a: &Point3, b: &Point3| {
            point3_equal(a, b, crate::STRICT_PREDICATES).value() == Some(true)
        };
        let same_line = |a: &[Point3], b: &[Point3]| {
            a.iter().zip(b).all(|(p, q)| equal(p, q))
                || a.iter().zip(b.iter().rev()).all(|(p, q)| equal(p, q))
        };
        let rows = |c: &Vec<Vec<Point3>>| vec![c[0].clone(), c[c.len() - 1].clone()];
        let columns = |c: &Vec<Vec<Point3>>| {
            vec![
                c.iter().map(|row| row[0].clone()).collect::<Vec<_>>(),
                c.iter()
                    .map(|row| row[row.len() - 1].clone())
                    .collect::<Vec<_>>(),
            ]
        };
        let corners = |c: &Vec<Vec<Point3>>| {
            let last = c.len() - 1;
            let width = c[0].len() - 1;
            vec![
                c[0][0].clone(),
                c[0][width].clone(),
                c[last][0].clone(),
                c[last][width].clone(),
            ]
        };
        for first in 0..patches.len() {
            for second in 0..patches.len() {
                if first == second {
                    continue;
                }
                let (a, b) = (controls(first), controls(second));
                let share = |x: Vec<Vec<Point3>>, y: Vec<Vec<Point3>>| {
                    x.iter().any(|p| y.iter().any(|q| same_line(p, q)))
                };
                let shared_corner = corners(&a)
                    .iter()
                    .any(|p| corners(&b).iter().any(|q| equal(p, q)));
                match patch_contact(n, first, second) {
                    PatchContact::EdgeAlong(1) => assert!(share(rows(&a), rows(&b))),
                    PatchContact::EdgeAlong(0) => assert!(share(columns(&a), columns(&b))),
                    PatchContact::Corner => {
                        assert!(shared_corner);
                        assert!(!share(rows(&a), rows(&b)) && !share(columns(&a), columns(&b)));
                    }
                    PatchContact::Disjoint => assert!(!shared_corner),
                    other => panic!("unexpected contact {}", matches!(other, PatchContact::Edge)),
                }
            }
        }
    }
}
