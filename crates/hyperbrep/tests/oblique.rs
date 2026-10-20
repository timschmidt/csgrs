#[allow(dead_code)]
#[path = "support/oblique.rs"]
mod support;

use hyperbrep::{Matrix4, Real};

#[test]
fn distant_angular_charts_preserve_conic_inverse_and_plane_contacts() {
    support::conic_queries();
}

#[test]
fn oblique_torus_half_continues_into_axial_band() {
    support::torus_band_workflow(&Matrix4::identity(), &Real::one());
}

#[test]
fn oblique_torus_band_survives_reflected_exact_similarities() {
    for scale in [support::q(1, 1000), Real::from(2).sqrt().unwrap()] {
        support::torus_band_workflow(&support::similarity(&scale, true), &scale);
    }
}
