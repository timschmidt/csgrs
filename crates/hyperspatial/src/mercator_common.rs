//! Shared Mercator domain, identity, longitude and evidence enforcement.
use crate::{
    Angle, Certainty, Context, Decision, Ellipsoid, Error, GeodeticPosition, GeodeticSource,
    InputProvenance, Latitude, Length, Longitude, LongitudePolicy, MercatorParameters,
    ProjectedPosition, Receipt, ReferenceFrame, Result,
};
use hyperlattice::Point3;
use hyperreal::Real;
use std::{cmp::Ordering, sync::Arc};

#[derive(Debug)]
pub(crate) struct Definition {
    pub(crate) ellipsoid: Arc<Ellipsoid>,
    pub(crate) reference: ReferenceFrame,
    pub(crate) parameters: MercatorParameters,
    scale_radius: Real,
    pub(crate) receipt: Receipt,
}

// Private family dispatch: public callers cannot supply unchecked inverse laws.
pub(crate) trait Mercator: Clone {
    const FORWARD: &'static str;
    const INVERSE: &'static str;
    fn definition(&self) -> &Arc<Definition>;
    fn isometric(&self, latitude: &Latitude) -> Result<Real>;
    fn inverse_latitude(
        &self,
        psi: Real,
        parent: Receipt,
        context: &mut Context,
    ) -> Result<Latitude>;
    fn original(source: &GeodeticPosition) -> Option<&ProjectedPosition<Self>>;
    fn retain(source: Arc<ProjectedPosition<Self>>) -> GeodeticSource;
}

impl Definition {
    pub(crate) fn new(
        ellipsoid: Arc<Ellipsoid>,
        reference: ReferenceFrame,
        parameters: MercatorParameters,
        operation: &'static str,
        mut decisions: Vec<Decision>,
        context: &mut Context,
    ) -> Result<Arc<Self>> {
        if context.compare(
            "Mercator scale > 0",
            &parameters.scale,
            &Real::zero(),
            &mut decisions,
        )? != Ordering::Greater
        {
            return Err(Error::InvalidInput("Mercator scale must be positive"));
        }
        let scale_radius = ellipsoid.equatorial_radius().as_metres() * &parameters.scale;
        let mut parents = vec![ellipsoid.receipt().clone()];
        parents.extend(parameters.central_meridian.receipt().cloned());
        let receipt = context.receipt(operation, parents, decisions, None)?;
        Ok(Arc::new(Self {
            ellipsoid,
            reference,
            parameters,
            scale_radius,
            receipt,
        }))
    }
    fn principal_domain(
        &self,
        delta: &Real,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<()> {
        let pi = Real::pi();
        if context.compare("Mercator delta longitude >= -pi", delta, &(-&pi), decisions)?
            == Ordering::Less
            || context.compare("Mercator delta longitude < pi", delta, &pi, decisions)?
                != Ordering::Less
        {
            return Err(Error::InvalidInput(
                "outside Mercator principal longitude strip",
            ));
        }
        Ok(())
    }

    fn inverse_longitude(
        &self,
        easting: &Real,
        context: &mut Context,
        decisions: &mut Vec<Decision>,
    ) -> Result<Real> {
        let delta = ((easting - self.parameters.false_easting.as_metres()) / &self.scale_radius)?;
        if self.parameters.longitude_policy != LongitudePolicy::Unrolled {
            self.principal_domain(&delta, context, decisions)?;
        }
        Ok(self.parameters.central_meridian.angle().as_radians() + delta)
    }
}

pub(crate) fn project<P: Mercator>(
    projection: &P,
    source: &GeodeticPosition,
    context: &mut Context,
) -> Result<ProjectedPosition<P>> {
    let definition = projection.definition();
    context.checkpoint()?;
    let mut decisions = Vec::new();
    definition
        .reference
        .compatible(source.reference(), context, &mut decisions)?;
    definition
        .ellipsoid
        .compatible(source.ellipsoid(), context, &mut decisions)?;
    let latitude = source.latitude().angle().as_radians();
    let half_pi = (Real::pi() / Real::from(2))?;
    if !source.latitude().has_open_domain() {
        match context.compare(
            "Mercator abs(latitude) < pi/2",
            &latitude.abs(),
            &half_pi,
            &mut decisions,
        )? {
            Ordering::Less => {}
            Ordering::Equal => {
                return Err(Error::ProjectionSingularity(
                    "Mercator at a geographic pole",
                ));
            }
            Ordering::Greater => {
                return Err(Error::InvalidInput("Mercator latitude outside poles"));
            }
        }
    }
    let mut delta = source.longitude().angle().as_radians()
        - definition.parameters.central_meridian.angle().as_radians();
    match definition.parameters.longitude_policy {
        LongitudePolicy::Unrolled => {}
        LongitudePolicy::RejectOutsidePrincipal => {
            definition.principal_domain(&delta, context, &mut decisions)?
        }
        LongitudePolicy::WrapSigned => {
            delta = Longitude::wrap_signed(Angle::radians(delta), context)?
                .angle()
                .as_radians()
                .clone()
        }
    }
    let coordinates = if let Some(original) = P::original(source)
        && Arc::ptr_eq(definition, original.projection().definition())
        && source.receipt().certainty() != Certainty::Approximate
    {
        // A certified inverse retains the represented source relation.
        original.coordinates().clone()
    } else {
        let isometric = projection.isometric(source.latitude())?;
        Point3::new(
            definition.parameters.false_easting.as_metres() + &definition.scale_radius * delta,
            definition.parameters.false_northing.as_metres() + &definition.scale_radius * isometric,
            source.height().as_metres().clone(),
        )
    };
    let receipt = context.receipt(
        P::FORWARD,
        vec![definition.receipt.clone(), source.receipt().clone()],
        decisions,
        None,
    )?;
    Ok(ProjectedPosition {
        coordinates,
        projection: projection.clone(),
        receipt,
        source: Some(Arc::new(source.clone())),
    })
}

pub(crate) fn position<P: Mercator>(
    projection: &P,
    coordinates: Point3,
    provenance: InputProvenance,
    context: &mut Context,
) -> Result<ProjectedPosition<P>> {
    let definition = projection.definition();
    context.checkpoint()?;
    let mut decisions = Vec::new();
    definition.inverse_longitude(&coordinates.x, context, &mut decisions)?;
    let receipt = context.receipt(
        "Mercator projected input",
        vec![definition.receipt.clone()],
        decisions,
        Some(provenance),
    )?;
    Ok(ProjectedPosition {
        coordinates,
        projection: projection.clone(),
        receipt,
        source: None,
    })
}

pub(crate) fn unproject<P: Mercator>(
    projection: &P,
    source: &ProjectedPosition<P>,
    context: &mut Context,
) -> Result<GeodeticPosition> {
    context.checkpoint()?;
    let definition = projection.definition();
    if !Arc::ptr_eq(definition, source.projection().definition()) {
        return Err(Error::ProjectionMismatch);
    }
    let mut decisions = Vec::new();
    let longitude = definition.inverse_longitude(&source.coordinates.x, context, &mut decisions)?;
    let receipt = context.receipt(
        P::INVERSE,
        vec![definition.receipt.clone(), source.receipt.clone()],
        decisions,
        None,
    )?;
    let latitude = if let Some(original) = source.source_geodetic()
        && source.receipt().certainty() != Certainty::Approximate
    {
        Latitude::retained(original.latitude(), receipt, context)?
    } else {
        let psi = ((&source.coordinates.y - definition.parameters.false_northing.as_metres())
            / &definition.scale_radius)?;
        projection.inverse_latitude(psi, receipt, context)?
    };
    let receipt = latitude.receipt().clone();
    Ok(GeodeticPosition {
        latitude,
        longitude: Longitude::unrolled(Angle::radians(longitude)),
        height: Length::metres(source.coordinates.z.clone()),
        ellipsoid: definition.ellipsoid.clone(),
        reference: definition.reference.clone(),
        receipt,
        source: P::retain(Arc::new(source.clone())),
    })
}
