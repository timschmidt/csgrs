use crate::mesh::Mesh;
use crate::sketch::Profile;
use crate::{Real, csg::CSG};

fn assert_recorded(name: &str) {
    let snapshot = hyperreal::dispatch_trace::take_trace();
    let correlation = snapshot.correlation_summary();
    assert!(
        correlation.dispatch_events > 0 || correlation.rational_temporaries > 0,
        "{name} did not emit an exact-computation path trace"
    );
}

#[test]
fn public_mesh_boolean_emits_correlated_dispatch_trace() {
    let left = Mesh::cube(Real::from(4), ());
    let right =
        Mesh::cube(Real::from(4), ()).translate(Real::from(1), Real::from(1), Real::from(1));

    hyperreal::dispatch_trace::reset();
    let result = hyperreal::dispatch_trace::with_recording(|| left.try_union(&right));
    assert!(
        !result
            .expect("overlapping cube union remains certified")
            .polygons
            .is_empty()
    );
    assert_recorded("csgrs mesh boolean");
}

#[test]
fn public_profile_boolean_emits_correlated_dispatch_trace() {
    let left = Profile::rectangle(Real::from(4), Real::from(4));
    let right = Profile::circle(Real::from(3), 24);

    hyperreal::dispatch_trace::reset();
    let result = hyperreal::dispatch_trace::with_recording(|| left.try_union(&right));
    assert!(
        !result
            .expect("overlapping profile union remains certified")
            .as_region()
            .is_empty()
    );
    assert_recorded("csgrs profile boolean");
}

#[test]
fn public_certified_circle_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile =
        hyperreal::dispatch_trace::with_recording(|| Profile::circle(Real::from(3), 24));
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs certified circle");
}

#[test]
fn public_certified_ellipse_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::ellipse(Real::from(8), Real::from(4), 24)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs certified ellipse");
}

#[test]
fn public_certified_star_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::star(12, Real::from(8), Real::from(4))
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs certified star");
}

#[test]
fn public_certified_regular_ngon_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile =
        hyperreal::dispatch_trace::with_recording(|| Profile::regular_ngon(7, Real::from(4)));
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs certified regular n-gon");
}

#[test]
fn public_certified_teardrop_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::teardrop(Real::from(6), Real::from(10), 24)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs certified teardrop");
}

#[test]
fn public_exact_egg_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::egg(Real::from(6), Real::from(10), 24)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs exact egg");
}

#[test]
fn public_certified_squircle_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::squircle(Real::from(8), Real::from(6), 24)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs certified squircle");
}

#[test]
fn public_certified_ring_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::ring(Real::from(6), Real::from(2), 24)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_eq!(profile.hole_contour_count(), 1);
    assert_recorded("csgrs certified ring");
}

#[test]
fn public_naca_airfoil_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::airfoil_naca4(
            Real::from(2),
            Real::from(4),
            Real::from(12),
            Real::from(20),
            80,
        )
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs NACA airfoil");
}

#[test]
fn public_involute_gear_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::involute_gear(
            Real::from(2),
            20,
            Real::from(20),
            Real::zero(),
            Real::zero(),
            4,
        )
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs involute gear");
}

#[test]
fn public_cycloidal_gear_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::cycloidal_gear(Real::from(2), 12, Real::from(1), Real::zero(), 4)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs cycloidal gear");
}

#[test]
fn public_cycloidal_rack_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::cycloidal_rack(Real::from(2), 4, Real::zero(), 8)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs cycloidal rack");
}

#[test]
fn public_involute_rack_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::involute_rack(Real::from(2), 4, Real::from(20), Real::zero(), Real::zero())
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs involute rack");
}

#[test]
fn public_aligned_reuleaux_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile =
        hyperreal::dispatch_trace::with_recording(|| Profile::reuleaux(3, Real::from(6), 24));
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs aligned Reuleaux polygon");
}

#[test]
fn public_exact_heart_emits_correlated_dispatch_trace() {
    hyperreal::dispatch_trace::reset();
    let profile = hyperreal::dispatch_trace::with_recording(|| {
        Profile::heart(Real::from(8), Real::from(6), 32)
    });
    assert_eq!(profile.material_contour_count(), 1);
    assert_recorded("csgrs exact heart");
}

#[test]
fn public_free_writers_emit_correlated_dispatch_trace() {
    let mesh = Mesh::cube(Real::from(2), ());

    hyperreal::dispatch_trace::reset();
    let output_size = hyperreal::dispatch_trace::with_recording(|| {
        let mut obj = Vec::new();
        crate::io::obj::write_obj(&mesh, &mut obj, "trace").expect("OBJ writer");
        let mut ply = Vec::new();
        crate::io::ply::write_ply(&mesh, &mut ply, "trace").expect("PLY writer");
        let mut amf = Vec::new();
        crate::io::amf::write_amf(&mesh, &mut amf, "trace", "millimeter").expect("AMF writer");
        let colored = crate::io::amf::to_amf_with_color(
            &mesh,
            "trace",
            "millimeter",
            (Real::one(), Real::zero(), Real::zero()),
        )
        .expect("colored AMF serializer");
        let mut gltf = Vec::new();
        crate::io::gltf::write_gltf(&mesh, &mut gltf, "trace").expect("glTF writer");
        obj.len() + ply.len() + amf.len() + colored.len() + gltf.len()
    });
    assert!(output_size > 0);
    assert_recorded("csgrs free writer exporters");
}
