use super::{SvgError, SvgResult};
use crate::{
    CircularArc2, CubicBezier2, Curve2, CurveGeometry2, CurvePath2, LineSeg2, NurbsCurve2, Point2,
    PolynomialSplineCurve2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2, Real,
    SplinePeriodicity2,
};
use std::fmt::Write;

const PREFIX: &str = "1:";
const MAGIC: &[u8; 4] = b"HCP1";

pub(super) fn encode_path(path: &CurvePath2, max_bytes: usize) -> SvgResult<String> {
    let mut writer = ExactWriter::new(max_bytes);
    writer.bytes.extend_from_slice(MAGIC);
    writer.write_len(path.curves().len())?;
    for curve in path.curves() {
        writer.write_curve(curve)?;
    }
    writer.check_size()?;

    let mut encoded = String::with_capacity(PREFIX.len() + writer.bytes.len() * 2);
    encoded.push_str(PREFIX);
    for byte in writer.bytes {
        write!(&mut encoded, "{byte:02x}").expect("writing exact SVG metadata cannot fail");
    }
    Ok(encoded)
}

pub(super) fn decode_path(encoded: &str, max_bytes: usize) -> SvgResult<CurvePath2> {
    let hex = encoded.strip_prefix(PREFIX).ok_or_else(|| {
        SvgError::Unsupported("unknown Hypercurve SVG path extension version".into())
    })?;
    if hex.len() % 2 != 0 {
        return Err(malformed("exact path extension has odd hexadecimal length"));
    }
    if hex.len() / 2 > max_bytes {
        return Err(extension_overflow());
    }

    let mut bytes = Vec::with_capacity(hex.len() / 2);
    for pair in hex.as_bytes().as_chunks::<2>().0 {
        let high = hex_digit(pair[0])?;
        let low = hex_digit(pair[1])?;
        bytes.push((high << 4) | low);
    }

    let mut reader = ExactReader::new(&bytes, max_bytes);
    if reader.take(MAGIC.len())? != MAGIC {
        return Err(malformed("exact path extension has an invalid header"));
    }
    let count = reader.read_len()?;
    reader.check_collection_len(count, 1)?;
    let curves = (0..count)
        .map(|_| reader.read_curve())
        .collect::<SvgResult<Vec<_>>>()?;
    if !reader.is_empty() {
        return Err(malformed("exact path extension has trailing bytes"));
    }
    CurvePath2::try_new(curves).map_err(|error| SvgError::Geometry(error.to_string()))
}

fn hex_digit(byte: u8) -> SvgResult<u8> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err(malformed(
            "exact path extension contains non-hexadecimal data",
        )),
    }
}

fn malformed(detail: impl Into<String>) -> SvgError {
    SvgError::MalformedInput(detail.into())
}

fn extension_overflow() -> SvgError {
    SvgError::SizeOverflow {
        limit: "configured exact-extension byte count",
    }
}

struct ExactWriter {
    bytes: Vec<u8>,
    max_bytes: usize,
}

impl ExactWriter {
    fn new(max_bytes: usize) -> Self {
        Self {
            bytes: Vec::new(),
            max_bytes,
        }
    }

    fn check_size(&self) -> SvgResult<()> {
        if self.bytes.len() > self.max_bytes {
            Err(extension_overflow())
        } else {
            Ok(())
        }
    }

    fn write_u8(&mut self, value: u8) -> SvgResult<()> {
        self.bytes.push(value);
        self.check_size()
    }

    fn write_u32(&mut self, value: u32) -> SvgResult<()> {
        self.bytes.extend_from_slice(&value.to_be_bytes());
        self.check_size()
    }

    fn write_len(&mut self, value: usize) -> SvgResult<()> {
        self.write_u32(value.try_into().map_err(|_| extension_overflow())?)
    }

    fn write_bool(&mut self, value: bool) -> SvgResult<()> {
        self.write_u8(u8::from(value))
    }

    fn write_real(&mut self, value: &Real) -> SvgResult<()> {
        let bytes = value.to_bytes();
        self.write_len(bytes.len())?;
        self.bytes.extend_from_slice(&bytes);
        self.check_size()
    }

    fn write_point(&mut self, point: &Point2) -> SvgResult<()> {
        self.write_real(point.x())?;
        self.write_real(point.y())
    }

    fn write_points(&mut self, points: &[Point2]) -> SvgResult<()> {
        self.write_len(points.len())?;
        for point in points {
            self.write_point(point)?;
        }
        Ok(())
    }

    fn write_reals(&mut self, values: &[Real]) -> SvgResult<()> {
        self.write_len(values.len())?;
        for value in values {
            self.write_real(value)?;
        }
        Ok(())
    }

    fn write_periodicity(&mut self, periodicity: &SplinePeriodicity2) -> SvgResult<()> {
        match periodicity {
            SplinePeriodicity2::NonPeriodic => self.write_bool(false),
            SplinePeriodicity2::Periodic { period } => {
                self.write_bool(true)?;
                self.write_real(period)
            }
        }
    }

    fn write_curve(&mut self, curve: &Curve2) -> SvgResult<()> {
        match curve.geometry() {
            CurveGeometry2::Line(line) => {
                self.write_u8(0)?;
                self.write_point(line.start())?;
                self.write_point(line.end())
            }
            CurveGeometry2::CircularArc(arc) => {
                self.write_u8(1)?;
                self.write_point(arc.start())?;
                self.write_point(arc.end())?;
                self.write_point(arc.center())?;
                self.write_bool(arc.is_clockwise())?;
                self.write_bool(arc.bulge().is_some())?;
                if let Some(bulge) = arc.bulge() {
                    self.write_real(bulge)?;
                }
                Ok(())
            }
            CurveGeometry2::QuadraticBezier(curve) => {
                self.write_u8(2)?;
                self.write_point(curve.start())?;
                self.write_point(curve.control())?;
                self.write_point(curve.end())
            }
            CurveGeometry2::CubicBezier(curve) => {
                self.write_u8(3)?;
                self.write_point(curve.start())?;
                self.write_point(curve.control1())?;
                self.write_point(curve.control2())?;
                self.write_point(curve.end())
            }
            CurveGeometry2::RationalQuadraticBezier(curve) => {
                self.write_u8(4)?;
                for point in curve.control_points() {
                    self.write_point(point)?;
                }
                for weight in curve.weights() {
                    self.write_real(weight)?;
                }
                Ok(())
            }
            CurveGeometry2::RationalBezier(curve) => {
                self.write_u8(5)?;
                self.write_points(curve.control_points())?;
                self.write_reals(curve.weights())
            }
            CurveGeometry2::PolynomialBSpline(curve) => {
                self.write_u8(6)?;
                self.write_len(curve.degree())?;
                self.write_points(curve.control_points())?;
                self.write_reals(curve.knots())?;
                self.write_periodicity(curve.periodicity())
            }
            CurveGeometry2::Nurbs(curve) => {
                self.write_u8(7)?;
                self.write_len(curve.degree())?;
                self.write_points(curve.control_points())?;
                self.write_reals(curve.weights())?;
                self.write_reals(curve.knots())?;
                self.write_periodicity(curve.periodicity())
            }
        }
    }
}

struct ExactReader<'a> {
    bytes: &'a [u8],
    position: usize,
    max_bytes: usize,
}

impl<'a> ExactReader<'a> {
    fn new(bytes: &'a [u8], max_bytes: usize) -> Self {
        Self {
            bytes,
            position: 0,
            max_bytes,
        }
    }

    fn is_empty(&self) -> bool {
        self.position == self.bytes.len()
    }

    fn remaining(&self) -> usize {
        self.bytes.len() - self.position
    }

    fn take(&mut self, count: usize) -> SvgResult<&'a [u8]> {
        let end = self
            .position
            .checked_add(count)
            .filter(|end| *end <= self.bytes.len())
            .ok_or_else(|| malformed("exact path extension is truncated"))?;
        let output = &self.bytes[self.position..end];
        self.position = end;
        Ok(output)
    }

    fn read_u8(&mut self) -> SvgResult<u8> {
        Ok(self.take(1)?[0])
    }

    fn read_u32(&mut self) -> SvgResult<u32> {
        let bytes: [u8; 4] = self
            .take(4)?
            .try_into()
            .expect("four-byte exact metadata slice");
        Ok(u32::from_be_bytes(bytes))
    }

    fn read_len(&mut self) -> SvgResult<usize> {
        Ok(self.read_u32()? as usize)
    }

    fn check_collection_len(&self, count: usize, minimum_bytes: usize) -> SvgResult<()> {
        if count > self.max_bytes || count.saturating_mul(minimum_bytes) > self.remaining() {
            Err(extension_overflow())
        } else {
            Ok(())
        }
    }

    fn read_bool(&mut self) -> SvgResult<bool> {
        match self.read_u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(malformed("exact path extension has an invalid boolean")),
        }
    }

    fn read_real(&mut self) -> SvgResult<Real> {
        let count = self.read_len()?;
        if count > self.max_bytes || count > self.remaining() {
            return Err(extension_overflow());
        }
        Real::from_bytes(self.take(count)?)
            .map_err(|error| malformed(format!("invalid exact coordinate: {error}")))
    }

    fn read_point(&mut self) -> SvgResult<Point2> {
        Ok(Point2::new(self.read_real()?, self.read_real()?))
    }

    fn read_points(&mut self) -> SvgResult<Vec<Point2>> {
        let count = self.read_len()?;
        self.check_collection_len(count, 8)?;
        (0..count).map(|_| self.read_point()).collect()
    }

    fn read_reals(&mut self) -> SvgResult<Vec<Real>> {
        let count = self.read_len()?;
        self.check_collection_len(count, 4)?;
        (0..count).map(|_| self.read_real()).collect()
    }

    fn read_periodicity(&mut self) -> SvgResult<SplinePeriodicity2> {
        Ok(if self.read_bool()? {
            SplinePeriodicity2::Periodic {
                period: self.read_real()?,
            }
        } else {
            SplinePeriodicity2::NonPeriodic
        })
    }

    fn read_curve(&mut self) -> SvgResult<Curve2> {
        let curve = match self.read_u8()? {
            0 => Curve2::from(
                LineSeg2::try_new(self.read_point()?, self.read_point()?)
                    .map_err(geometry_error)?,
            ),
            1 => {
                let start = self.read_point()?;
                let end = self.read_point()?;
                let center = self.read_point()?;
                let clockwise = self.read_bool()?;
                let bulge = self.read_bool()?.then(|| self.read_real()).transpose()?;
                Curve2::from(
                    CircularArc2::try_from_center_with_bulge(start, end, center, clockwise, bulge)
                        .map_err(geometry_error)?,
                )
            }
            2 => Curve2::from(QuadraticBezier2::new(
                self.read_point()?,
                self.read_point()?,
                self.read_point()?,
            )),
            3 => Curve2::from(CubicBezier2::new(
                self.read_point()?,
                self.read_point()?,
                self.read_point()?,
                self.read_point()?,
            )),
            4 => {
                let points = [self.read_point()?, self.read_point()?, self.read_point()?];
                let weights = [self.read_real()?, self.read_real()?, self.read_real()?];
                Curve2::from(
                    RationalQuadraticBezier2::try_new(
                        points[0].clone(),
                        points[1].clone(),
                        points[2].clone(),
                        weights[0].clone(),
                        weights[1].clone(),
                        weights[2].clone(),
                    )
                    .map_err(geometry_error)?,
                )
            }
            5 => Curve2::from(
                RationalBezier2::try_new(self.read_points()?, self.read_reals()?)
                    .map_err(geometry_error)?,
            ),
            6 => {
                let degree = self.read_len()?;
                let points = self.read_points()?;
                let knots = self.read_reals()?;
                let periodicity = self.read_periodicity()?;
                Curve2::from(
                    PolynomialSplineCurve2::try_new_expanded_with_periodicity(
                        degree,
                        points,
                        knots,
                        periodicity,
                    )
                    .map_err(geometry_error)?,
                )
            }
            7 => {
                let degree = self.read_len()?;
                let points = self.read_points()?;
                let weights = self.read_reals()?;
                let knots = self.read_reals()?;
                let periodicity = self.read_periodicity()?;
                Curve2::from(
                    NurbsCurve2::try_new_expanded_with_periodicity(
                        degree,
                        points,
                        weights,
                        knots,
                        periodicity,
                    )
                    .map_err(geometry_error)?,
                )
            }
            tag => {
                return Err(SvgError::Unsupported(format!(
                    "unknown exact Hypercurve curve tag {tag}"
                )));
            }
        };
        Ok(curve)
    }
}

fn geometry_error(error: impl std::fmt::Display) -> SvgError {
    SvgError::Geometry(error.to_string())
}
