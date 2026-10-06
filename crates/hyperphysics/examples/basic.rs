use hyperlattice::Vector3;
use hyperphysics::{
    AxisAlignedBox3, BodyId, BodyKind, ExactBody3, ExactFixture3, ExactMaterial, FixtureId,
    MaterialId, PhysicsShape3,
};
use hyperreal::Real;

fn main() -> hyperphysics::PhysicsResult<()> {
    let material = ExactMaterial::new(MaterialId::new("aluminum")?, "aluminum", Real::from(2700))?;
    let bounds = AxisAlignedBox3::new(
        Vector3::new([Real::from(0), Real::from(0), Real::from(0)]),
        Vector3::new([Real::from(1), Real::from(1), Real::from(1)]),
    )?;
    let fixture = ExactFixture3::new(
        FixtureId::new("fixture-0")?,
        PhysicsShape3::axis_aligned_box(bounds),
        material,
    );
    let body = ExactBody3::new(BodyId::new("body-0")?, BodyKind::Dynamic, vec![fixture]);

    assert_eq!(body.fixtures().len(), 1);
    Ok(())
}
