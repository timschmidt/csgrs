//! Explicit STEP spline admission through native positive-weight carriers.
use super::super::decode::{component, parts};
use super::*;
use crate::part21::{Entity, Value, ValueKind};
use hyperbrep::CurveParameterLocation;

pub(super) fn is_spline(entity: Entity<'_, '_>) -> bool {
    component(entity, "B_SPLINE_CURVE_WITH_KNOTS").is_some()
}

fn logical(args: &mut Args<'_, '_>) -> Result<Option<bool>, Error> {
    match args.next()?.kind() {
        ValueKind::Enumeration("T") => Ok(Some(true)),
        ValueKind::Enumeration("F") => Ok(Some(false)),
        ValueKind::Enumeration("U") => Ok(None),
        _ => Err(args
            .site
            .error(ErrorKind::Expected("LOGICAL (.T., .F. or .U.)"))),
    }
}

impl Resolver<'_, '_, '_> {
    fn spline_integer(&mut self, value: Value<'_, '_>, site: Site) -> Result<usize, Error> {
        // Charge and check the shared exact-number contract before converting
        // a count. Integer overflow cannot drive a vector allocation.
        self.scalar(value, site)?;
        let ValueKind::Integer(number) = value.kind() else {
            return Err(site.error(ErrorKind::Expected("nonnegative integer")));
        };
        number
            .lexeme()
            .parse()
            .map_err(|_| site.invalid("spline integer is outside usize"))
    }

    pub(super) fn spline3(&mut self, id: u64, scale: &Real) -> Result<Curve3, Error> {
        let entity = self.entity(id)?;
        let site = Site::entity(entity);
        let knot_record = component(entity, "B_SPLINE_CURVE_WITH_KNOTS")
            .ok_or_else(|| site.error(ErrorKind::Unsupported("spline curve form")))?;
        if entity.is_complex() {
            parts(
                entity,
                &[
                    "BOUNDED_CURVE",
                    "B_SPLINE_CURVE",
                    "B_SPLINE_CURVE_WITH_KNOTS",
                    "CURVE",
                    "GEOMETRIC_REPRESENTATION_ITEM",
                    "REPRESENTATION_ITEM",
                ],
                &["RATIONAL_B_SPLINE_CURVE"],
            )?;
            for name in ["BOUNDED_CURVE", "CURVE", "GEOMETRIC_REPRESENTATION_ITEM"] {
                Args::new(component(entity, name).expect("required component"), site).finish()?;
            }
            let mut name = Args::new(
                component(entity, "REPRESENTATION_ITEM").expect("required component"),
                site,
            );
            self.text(name.next()?, site)?;
            name.finish()?;
        }
        let mut base = Args::new(
            if entity.is_complex() {
                component(entity, "B_SPLINE_CURVE").expect("required component")
            } else {
                knot_record
            },
            site,
        );
        if !entity.is_complex() {
            self.text(base.next()?, site)?;
        }
        let degree = self.spline_integer(base.next()?, site)?;
        let mut controls = Vec::new();
        for value in base.list()? {
            let point = self.point(reference(value, site)?)?;
            push(
                &mut controls,
                Point3::new(point.x * scale, point.y * scale, point.z * scale),
            )
            .map_err(|kind| site.error(kind))?;
        }
        if degree == 0 || degree >= controls.len() {
            return Err(site.invalid("invalid spline degree"));
        }
        match base.next()?.kind() {
            ValueKind::Enumeration(
                "POLYLINE_FORM" | "CIRCULAR_ARC" | "ELLIPTIC_ARC" | "PARABOLIC_ARC"
                | "HYPERBOLIC_ARC" | "UNSPECIFIED",
            ) => {}
            _ => return Err(site.error(ErrorKind::Expected("b_spline_curve_form"))),
        }
        let closed = logical(&mut base)?;
        // Source hints remain in the retained STEP document. They never
        // replace native whole-image, simplicity or manifold validation.
        let _self_intersect = logical(&mut base)?;
        let mut args = if entity.is_complex() {
            base.finish()?;
            Args::new(knot_record, site)
        } else {
            base
        };
        let mut multiplicities = Vec::new();
        let expected = controls
            .len()
            .checked_add(degree)
            .and_then(|n| n.checked_add(1))
            .ok_or_else(|| site.invalid("spline knot count overflow"))?;
        let mut total = 0usize;
        for value in args.list()? {
            let count = self.spline_integer(value, site)?;
            if count == 0 {
                return Err(site.invalid("zero knot multiplicity"));
            }
            total = total
                .checked_add(count)
                .ok_or_else(|| site.invalid("spline knot count overflow"))?;
            if total > expected {
                return Err(site.invalid("knot multiplicities exceed control/degree count"));
            }
            push(&mut multiplicities, count).map_err(|kind| site.error(kind))?;
        }
        if total != expected {
            return Err(site.invalid("knot multiplicities disagree with control/degree count"));
        }
        self.spline_knots(total, site)?;
        let mut knots = Vec::new();
        knots
            .try_reserve_exact(total)
            .map_err(|_| site.error(ErrorKind::Allocation))?;
        let mut values = args.list()?;
        let mut previous = None;
        for count in multiplicities {
            let knot = self.scalar(
                values
                    .next()
                    .ok_or_else(|| site.invalid("missing distinct knot"))?,
                site,
            )?;
            if let Some(previous) = previous.as_ref()
                && order(site, previous, &knot)? != Ordering::Less
            {
                return Err(site.invalid("distinct knots must be strictly increasing"));
            }
            knots.extend(std::iter::repeat_n(knot.clone(), count));
            previous = Some(knot);
        }
        if values.next().is_some() {
            return Err(site.invalid("excess distinct knot"));
        }
        match args.next()?.kind() {
            ValueKind::Enumeration(
                "UNIFORM_KNOTS" | "QUASI_UNIFORM_KNOTS" | "PIECEWISE_BEZIER_KNOTS" | "UNSPECIFIED",
            ) => {}
            _ => return Err(site.error(ErrorKind::Expected("knot_type"))),
        }
        args.finish()?;
        let mut weights = Vec::new();
        weights
            .try_reserve_exact(controls.len())
            .map_err(|_| site.error(ErrorKind::Allocation))?;
        if let Some(record) = component(entity, "RATIONAL_B_SPLINE_CURVE") {
            let mut args = Args::new(record, site);
            for value in args.list()? {
                if weights.len() == controls.len() {
                    return Err(site.invalid("excess spline weight"));
                }
                weights.push(self.scalar(value, site)?);
            }
            args.finish()?;
            if weights.len() != controls.len() {
                return Err(site.invalid("missing spline weight"));
            }
        } else {
            weights.resize(controls.len(), Real::one());
        }
        let curve = Curve3::nurbs(degree, controls, weights, knots)
            .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        if let Some(closed) = closed {
            let endpoints_equal = same_vector(
                site,
                &(curve
                    .start()
                    .map_err(|e| site.error(ErrorKind::Geometry(e)))?
                    - curve
                        .end()
                        .map_err(|e| site.error(ErrorKind::Geometry(e)))?),
                &Vector3::zero(),
            )?;
            if endpoints_equal != closed {
                return Err(site.invalid("closed_curve disagrees with spline endpoints"));
            }
        }
        Ok(curve)
    }
}

pub(super) fn unique_parameter(
    location: CurveParameterLocation,
    site: Site,
) -> Result<Real, Error> {
    match location {
        CurveParameterLocation::None => {
            Err(site.invalid("vertex is not on the finite spline carrier"))
        }
        CurveParameterLocation::Parameters(parameters) if parameters.len() == 1 => {
            parameters[0].try_real().cloned().ok_or_else(|| {
                site.error(ErrorKind::Geometry(GeometryError::UnrepresentableParameter))
            })
        }
        _ => Err(site.error(ErrorKind::Unsupported("ambiguous spline vertex preimage"))),
    }
}

impl Builder {
    pub(super) fn spline_edge(
        &mut self,
        resolver: &mut Resolver<'_, '_, '_>,
        geometry: u64,
        start: u64,
        end: u64,
        same_sense: bool,
        scale: &Real,
    ) -> Result<(Edge, SourceCurveParameters), Error> {
        let site = Site::entity(resolver.entity(geometry)?);
        let curve = resolver.spline3(geometry, scale)?;
        let first = &self.by_vertex[&start];
        let second = &self.by_vertex[&end];
        let parameter = |point: &Point3| {
            unique_parameter(
                curve
                    .parameters_of(point)
                    .map_err(|e| site.error(ErrorKind::Geometry(e)))?,
                site,
            )
        };
        let a = parameter(&first.point)?;
        let b = parameter(&second.point)?;
        let parameters = SourceCurveParameters {
            curve: geometry,
            start: a.clone(),
            end: b.clone(),
        };
        curves::require_sense(&parameters, same_sense, site)?;
        let curve = if same_sense {
            curve.subcurve(&a, &b)
        } else {
            curve.subcurve(&b, &a).and_then(|c| c.reversed())
        }
        .map_err(|e| site.error(ErrorKind::Geometry(e)))?;
        let key_curve = self
            .model
            .curve(curve.clone())
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        let key = self
            .model
            .edge(&first.key, &second.key, &key_curve, curve.domain().clone())
            .map_err(|e| site.error(ErrorKind::Build(e)))?;
        Ok((
            Edge {
                key,
                start,
                end,
                circle: None,
                spline: Some(curve),
            },
            parameters,
        ))
    }
}
