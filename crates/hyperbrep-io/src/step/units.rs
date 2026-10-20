use super::{
    Error, ErrorKind,
    decode::{Args, Resolver, Site, component, parts, reference},
    error::{insert, push},
};
use crate::part21::{Value, ValueKind};
use hyperbrep::Real;
use std::collections::HashSet;

/// Unit category; plane and solid angles remain distinct despite zero SI dimensions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UnitKind {
    /// Length, normalized to metres.
    Length,
    /// Plane angle, normalized to radians.
    PlaneAngle,
    /// Solid angle, normalized to steradians.
    SolidAngle,
}
impl UnitKind {
    fn marker(self) -> &'static str {
        match self {
            Self::Length => "LENGTH_UNIT",
            Self::PlaneAngle => "PLANE_ANGLE_UNIT",
            Self::SolidAngle => "SOLID_ANGLE_UNIT",
        }
    }
}

/// A declared unit resolved through its exact authored conversion chain.
#[derive(Clone, Debug)]
pub struct Unit {
    /// Original entity ID.
    pub source: u64,
    /// Unit category.
    pub kind: UnitKind,
    /// Exact positive scale to the category's SI unit.
    pub scale_to_si: Real,
    /// Name for a conversion-based unit; SI units need no invented name.
    pub name: Option<String>,
}
/// Source uncertainty metadata; never a geometric merging or healing tolerance.
#[derive(Debug)]
pub struct Uncertainty {
    /// Original uncertainty entity ID.
    pub source: u64,
    /// Declared unit category.
    pub kind: UnitKind,
    /// Exact nonnegative magnitude in SI units.
    pub value_in_si: Real,
    /// Authored name.
    pub name: String,
    /// Authored description.
    pub description: String,
}
/// Resolved units for a three-dimensional geometric representation context.
#[derive(Debug)]
pub struct ContextUnits {
    /// Source context ID.
    pub source: u64,
    /// Required length unit.
    pub length: Unit,
    /// Optional plane-angle unit; angular mapping must require it when needed.
    pub plane_angle: Option<Unit>,
    /// Optional solid-angle unit.
    pub solid_angle: Option<Unit>,
    /// Uncertainty records in source order, preserved without applying them.
    pub uncertainties: Vec<Uncertainty>,
}

struct Pending {
    id: u64,
    kind: UnitKind,
    factor: Real,
    name: String,
    site: Site,
}
impl Resolver<'_, '_, '_> {
    pub(super) fn unit(&mut self, start: u64) -> Result<Unit, Error> {
        let mut id = start;
        let mut path = Vec::<Pending>::new();
        let mut active = HashSet::new();
        let mut resolved;
        loop {
            let entity = self.entity(id)?;
            let site = Site::entity(entity);
            if let Some(unit) = self.units.get(&id) {
                resolved = unit.clone();
                break;
            }
            active
                .try_reserve(1)
                .map_err(|_| site.error(ErrorKind::Allocation))?;
            if !active.insert(id) {
                return Err(site.error(ErrorKind::UnitCycle(id)));
            }
            let kinds = [UnitKind::Length, UnitKind::PlaneAngle, UnitKind::SolidAngle];
            let mut kinds = kinds
                .into_iter()
                .filter(|kind| component(entity, kind.marker()).is_some());
            let kind = kinds
                .next()
                .ok_or_else(|| site.error(ErrorKind::Unsupported("unit category")))?;
            if kinds.next().is_some() {
                return Err(site.invalid("multiple unit categories"));
            }
            parts(
                entity,
                &["NAMED_UNIT", kind.marker()],
                &["SI_UNIT", "CONVERSION_BASED_UNIT"],
            )?;
            Args::new(
                component(entity, kind.marker()).expect("checked component"),
                site,
            )
            .finish()?;
            let mut named = Args::new(
                component(entity, "NAMED_UNIT").expect("checked component"),
                site,
            );
            let dimensions = named.next()?;
            named.finish()?;
            match (
                component(entity, "SI_UNIT"),
                component(entity, "CONVERSION_BASED_UNIT"),
            ) {
                (Some(record), None) => {
                    if dimensions.kind() != ValueKind::Derived {
                        return Err(site.invalid("SI dimensions must be derived"));
                    }
                    let mut args = Args::new(record, site);
                    let exponent = match args.next()?.kind() {
                        ValueKind::Omitted => 0,
                        ValueKind::Enumeration(name) => prefix(name)
                            .ok_or_else(|| site.error(ErrorKind::Unsupported("SI prefix")))?,
                        _ => {
                            return Err(
                                site.error(ErrorKind::Expected("SI prefix or omitted value"))
                            );
                        }
                    };
                    let name_kind = match args.next()?.kind() {
                        ValueKind::Enumeration("METRE") => UnitKind::Length,
                        ValueKind::Enumeration("RADIAN") => UnitKind::PlaneAngle,
                        ValueKind::Enumeration("STERADIAN") => UnitKind::SolidAngle,
                        _ => return Err(site.error(ErrorKind::Unsupported("SI unit name"))),
                    };
                    args.finish()?;
                    if kind != name_kind {
                        return Err(site.invalid("SI name disagrees with unit category"));
                    }
                    let mut scale = Real::one();
                    for _ in 0..exponent.unsigned_abs() {
                        scale *= Real::from(10);
                    }
                    if exponent < 0 {
                        scale = scale
                            .inverse()
                            .map_err(|e| site.error(ErrorKind::Numeric(e)))?;
                    }
                    resolved = Unit {
                        source: id,
                        kind,
                        scale_to_si: scale,
                        name: None,
                    };
                    self.scale_bits(&resolved.scale_to_si, None, site)?;
                    self.cache_unit(&resolved, site)?;
                    break;
                }
                (None, Some(record)) => {
                    self.dimensions(reference(dimensions, site)?, kind)?;
                    let mut args = Args::new(record, site);
                    let name = self.text(args.next()?, site)?;
                    let measure = args.reference()?;
                    args.finish()?;
                    let (measure_kind, factor, target) = self.measure(measure)?;
                    if measure_kind != kind {
                        return Err(site.invalid("conversion measure has the wrong dimension"));
                    }
                    if !positive(&factor) {
                        return Err(site.invalid("unit conversion factor must be positive"));
                    }
                    push(
                        &mut path,
                        Pending {
                            id,
                            kind,
                            factor,
                            name,
                            site,
                        },
                    )
                    .map_err(|kind| site.error(kind))?;
                    id = target;
                }
                _ => {
                    return Err(site.error(ErrorKind::Expected(
                        "exactly one SI or conversion-based unit component",
                    )));
                }
            }
        }
        for pending in path.into_iter().rev() {
            if resolved.kind != pending.kind {
                return Err(pending
                    .site
                    .invalid("conversion target has the wrong dimension"));
            }
            self.scale_bits(&pending.factor, Some(&resolved.scale_to_si), pending.site)?;
            resolved = Unit {
                source: pending.id,
                kind: pending.kind,
                scale_to_si: pending.factor * resolved.scale_to_si,
                name: Some(pending.name),
            };
            self.cache_unit(&resolved, pending.site)?;
        }
        Ok(resolved)
    }
    fn scale_bits(&self, left: &Real, right: Option<&Real>, site: Site) -> Result<(), Error> {
        let bits = |value: &Real| {
            let value = value
                .exact_rational_ref()
                .expect("unit factors are rational");
            (value.numerator().bits(), value.denominator().bits())
        };
        let (mut numerator, mut denominator) = bits(left);
        if let Some(right) = right {
            let (n, d) = bits(right);
            numerator = numerator.saturating_add(n);
            denominator = denominator.saturating_add(d);
        }
        let observed = usize::try_from(numerator.max(denominator));
        if observed.is_err() || observed.is_ok_and(|bits| bits > self.limits.unit_scale_bits) {
            return Err(site.error(ErrorKind::Limit {
                resource: super::Resource::UnitScaleBits,
                limit: self.limits.unit_scale_bits,
                observed: observed.unwrap_or(usize::MAX),
            }));
        }
        Ok(())
    }
    fn cache_unit(&mut self, unit: &Unit, site: Site) -> Result<(), Error> {
        self.units
            .try_reserve(1)
            .map_err(|_| site.error(ErrorKind::Allocation))?;
        self.units.insert(unit.source, unit.clone());
        Ok(())
    }
    fn dimensions(&mut self, id: u64, kind: UnitKind) -> Result<(), Error> {
        let mut args = self.simple(id, "DIMENSIONAL_EXPONENTS")?;
        for index in 0..7 {
            let exponent = self.scalar(args.next()?, args.site)?;
            let expected = if index == 0 && kind == UnitKind::Length {
                Real::one()
            } else {
                Real::zero()
            };
            if exponent != expected {
                return Err(args
                    .site
                    .invalid("dimensional exponents disagree with unit category"));
            }
        }
        args.finish()
    }
    fn measure(&mut self, id: u64) -> Result<(UnitKind, Real, u64), Error> {
        let entity = self.entity(id)?;
        let site = Site::entity(entity);
        let record = if entity.is_complex() {
            parts(
                entity,
                &["MEASURE_WITH_UNIT"],
                &[
                    "LENGTH_MEASURE_WITH_UNIT",
                    "PLANE_ANGLE_MEASURE_WITH_UNIT",
                    "SOLID_ANGLE_MEASURE_WITH_UNIT",
                ],
            )?;
            for item in entity
                .components()
                .filter(|r| r.name() != "MEASURE_WITH_UNIT")
            {
                Args::new(item, site).finish()?;
            }
            component(entity, "MEASURE_WITH_UNIT").expect("checked component")
        } else {
            let record = entity.components().next().expect("syntax component");
            if !matches!(
                record.name(),
                "MEASURE_WITH_UNIT"
                    | "LENGTH_MEASURE_WITH_UNIT"
                    | "PLANE_ANGLE_MEASURE_WITH_UNIT"
                    | "SOLID_ANGLE_MEASURE_WITH_UNIT"
            ) {
                return Err(site.error(ErrorKind::Expected("measure with unit")));
            }
            record
        };
        let mut args = Args::new(record, site);
        let (kind, amount) = self.measure_value(args.next()?, site)?;
        let target = args.reference()?;
        args.finish()?;
        for (name, expected) in [
            ("LENGTH_MEASURE_WITH_UNIT", UnitKind::Length),
            ("PLANE_ANGLE_MEASURE_WITH_UNIT", UnitKind::PlaneAngle),
            ("SOLID_ANGLE_MEASURE_WITH_UNIT", UnitKind::SolidAngle),
        ] {
            if component(entity, name).is_some() && kind != expected {
                return Err(site.invalid("measure subtype disagrees with its value"));
            }
        }
        Ok((kind, amount, target))
    }
    fn measure_value(
        &mut self,
        value: Value<'_, '_>,
        site: Site,
    ) -> Result<(UnitKind, Real), Error> {
        let (kind, strict_positive) = match value.kind() {
            ValueKind::Typed("LENGTH_MEASURE") => (UnitKind::Length, false),
            ValueKind::Typed("POSITIVE_LENGTH_MEASURE") => (UnitKind::Length, true),
            ValueKind::Typed("PLANE_ANGLE_MEASURE") => (UnitKind::PlaneAngle, false),
            ValueKind::Typed("POSITIVE_PLANE_ANGLE_MEASURE") => (UnitKind::PlaneAngle, true),
            ValueKind::Typed("SOLID_ANGLE_MEASURE") => (UnitKind::SolidAngle, false),
            _ => return Err(site.error(ErrorKind::Unsupported("measure value type"))),
        };
        let amount = self.scalar(value.children().next().expect("syntax typed arity"), site)?;
        if strict_positive && !positive(&amount) {
            return Err(site.invalid("positive measure is not positive"));
        }
        Ok((kind, amount))
    }
    pub(super) fn context_units(&mut self, id: u64) -> Result<ContextUnits, Error> {
        let entity = self.entity(id)?;
        let site = Site::entity(entity);
        parts(
            entity,
            &[
                "GEOMETRIC_REPRESENTATION_CONTEXT",
                "GLOBAL_UNIT_ASSIGNED_CONTEXT",
                "REPRESENTATION_CONTEXT",
            ],
            &["GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT"],
        )?;
        let mut geometry = Args::new(
            component(entity, "GEOMETRIC_REPRESENTATION_CONTEXT").expect("checked component"),
            site,
        );
        let dimension = geometry.next()?;
        if !matches!(dimension.kind(), ValueKind::Integer(_))
            || self.scalar(dimension, site)? != Real::from(3)
        {
            return Err(site.error(ErrorKind::Expected("3D geometric context")));
        }
        geometry.finish()?;
        let mut context = Args::new(
            component(entity, "REPRESENTATION_CONTEXT").expect("checked component"),
            site,
        );
        self.text(context.next()?, site)?;
        self.text(context.next()?, site)?;
        context.finish()?;
        let mut units = Args::new(
            component(entity, "GLOBAL_UNIT_ASSIGNED_CONTEXT").expect("checked component"),
            site,
        );
        let mut length = None;
        let mut plane_angle = None;
        let mut solid_angle = None;
        for value in units.list()? {
            let unit = self.unit(reference(value, site)?)?;
            let slot = match unit.kind {
                UnitKind::Length => &mut length,
                UnitKind::PlaneAngle => &mut plane_angle,
                UnitKind::SolidAngle => &mut solid_angle,
            };
            if slot.replace(unit).is_some() {
                return Err(site.invalid("more than one unit for a category"));
            }
        }
        units.finish()?;
        let length =
            length.ok_or_else(|| site.error(ErrorKind::Expected("declared length unit")))?;
        let mut uncertainties = Vec::new();
        let mut uncertainty_ids = HashSet::new();
        if let Some(record) = component(entity, "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT") {
            let mut args = Args::new(record, site);
            for value in args.list()? {
                let id = reference(value, site)?;
                if !insert(&mut uncertainty_ids, id).map_err(|kind| site.error(kind))? {
                    return Err(site.invalid("duplicate uncertainty reference"));
                }
                let mut uncertainty = self.simple(id, "UNCERTAINTY_MEASURE_WITH_UNIT")?;
                let (kind, amount) = self.measure_value(uncertainty.next()?, uncertainty.site)?;
                let unit = self.unit(uncertainty.reference()?)?;
                if kind != unit.kind || !nonnegative(&amount) {
                    return Err(uncertainty
                        .site
                        .invalid("invalid uncertainty dimension or magnitude"));
                }
                let name = self.text(uncertainty.next()?, uncertainty.site)?;
                let description = self.text(uncertainty.next()?, uncertainty.site)?;
                uncertainty.finish()?;
                push(
                    &mut uncertainties,
                    Uncertainty {
                        source: id,
                        kind,
                        value_in_si: amount * unit.scale_to_si,
                        name,
                        description,
                    },
                )
                .map_err(|kind| site.error(kind))?;
            }
            if uncertainties.is_empty() {
                return Err(site.invalid("empty uncertainty context"));
            }
            args.finish()?;
        }
        Ok(ContextUnits {
            source: id,
            length,
            plane_angle,
            solid_angle,
            uncertainties,
        })
    }
}
fn positive(value: &Real) -> bool {
    value
        .exact_rational_ref()
        .is_some_and(|value| !value.is_zero() && !value.is_negative())
}
fn nonnegative(value: &Real) -> bool {
    value
        .exact_rational_ref()
        .is_some_and(|value| !value.is_negative())
}
fn prefix(name: &str) -> Option<i32> {
    Some(match name {
        "YOTTA" => 24,
        "ZETTA" => 21,
        "EXA" => 18,
        "PETA" => 15,
        "TERA" => 12,
        "GIGA" => 9,
        "MEGA" => 6,
        "KILO" => 3,
        "HECTO" => 2,
        "DECA" => 1,
        "DECI" => -1,
        "CENTI" => -2,
        "MILLI" => -3,
        "MICRO" => -6,
        "NANO" => -9,
        "PICO" => -12,
        "FEMTO" => -15,
        "ATTO" => -18,
        "ZEPTO" => -21,
        "YOCTO" => -24,
        _ => return None,
    })
}
