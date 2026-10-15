use super::{Error, ErrorKind, decode::Resolver};
use hyperbrep::{Point3, Real, Surface, Vector3};

pub(super) struct Plane {
    pub surface: Surface,
    pub origin: Point3,
    pub x: Vector3,
    pub y: Vector3,
}
impl Resolver<'_, '_, '_> {
    pub fn point(&mut self, id: u64) -> Result<Point3, Error> {
        let mut args = self.simple(id, "CARTESIAN_POINT")?;
        if let Some(point) = self.points.get(&id) {
            return Ok(point.clone());
        }
        self.text(args.next()?, args.site)?;
        let mut values = args.list()?;
        let mut next = || {
            let value = values
                .next()
                .ok_or_else(|| args.site.error(ErrorKind::Expected("three coordinates")))?;
            self.scalar(value, args.site)
        };
        let point = Point3::new(next()?, next()?, next()?);
        if values.next().is_some() {
            return Err(args.site.error(ErrorKind::Expected("three coordinates")));
        }
        args.finish()?;
        self.points
            .try_reserve(1)
            .map_err(|_| Error::at(Some(id), 0, ErrorKind::Allocation))?;
        self.points.insert(id, point.clone());
        Ok(point)
    }
    fn direction(&mut self, id: u64) -> Result<Vector3, Error> {
        let mut args = self.simple(id, "DIRECTION")?;
        self.text(args.next()?, args.site)?;
        let mut values = args.list()?;
        let mut next = || {
            let value = values.next().ok_or_else(|| {
                args.site
                    .error(ErrorKind::Expected("three direction ratios"))
            })?;
            self.scalar(value, args.site)
        };
        let vector = Vector3::new([next()?, next()?, next()?]);
        if values.next().is_some() {
            return Err(args
                .site
                .error(ErrorKind::Expected("three direction ratios")));
        }
        let site = args.site;
        args.finish()?;
        vector
            .normalize_checked()
            .map_err(|problem| site.error(ErrorKind::Numeric(problem)))
    }
    pub fn placement(&mut self, id: u64, scale: &Real) -> Result<Plane, Error> {
        let mut args = self.simple(id, "AXIS2_PLACEMENT_3D")?;
        let site = args.site;
        self.text(args.next()?, site)?;
        let origin = self.point(args.reference()?)?;
        let origin = Point3::new(&origin.x * scale, &origin.y * scale, &origin.z * scale);
        let z = match args.optional_reference()? {
            Some(id) => self.direction(id)?,
            None => Vector3::z(),
        };
        let reference = match args.optional_reference()? {
            Some(id) => self.direction(id)?,
            None if z[1] == Real::zero() && z[2] == Real::zero() => Vector3::y(),
            None => Vector3::x(),
        };
        args.finish()?;
        let projected = &reference - &(z.clone() * reference.dot(&z));
        let x = projected
            .normalize_checked()
            .map_err(|problem| site.error(ErrorKind::Numeric(problem)))?;
        let y = z
            .cross(&x)
            .normalize_checked()
            .map_err(|problem| site.error(ErrorKind::Numeric(problem)))?;
        let surface = Surface::plane(origin.clone(), x.clone(), y.clone())
            .map_err(|problem| site.error(ErrorKind::Geometry(problem)))?;
        Ok(Plane {
            surface,
            origin,
            x,
            y,
        })
    }
    pub fn plane(&mut self, id: u64, scale: &Real) -> Result<Plane, Error> {
        let mut args = self.simple(id, "PLANE")?;
        self.text(args.next()?, args.site)?;
        let placement = args.reference()?;
        args.finish()?;
        self.placement(placement, scale)
    }
}
