#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn operations_work_on_refs() {
        let a = Real::new(Rational::new(2));
        let b = Real::new(Rational::new(3));
        let c = Real::new(Rational::new(6));
        assert_eq!(&a * &b, c.clone());
        assert_eq!(&c / &b, Ok(a.clone()));
        assert_eq!(&c - &a, Real::new(Rational::new(4)));
        assert_eq!(-&c, Real::new(Rational::new(-6)));
        assert_eq!(&a + &b, Real::new(Rational::new(5)));
    }

    #[test]
    fn aggregate_helpers_keep_values_in_real_space() {
        let values = [Real::from(1_i32), Real::from(3_i32), Real::from(5_i32)];

        assert_eq!(Real::sum_refs(values.iter()), Real::from(9_i32));
        assert_eq!(Real::mean(&values), Some(Real::from(3_i32)));
        assert_eq!(
            Real::affine(&Real::from(1_i32), &Real::from(2_i32), &Real::from(3_i32)),
            Real::from(7_i32)
        );

        let stddev = Real::sample_stddev(&values).unwrap();
        assert_eq!(stddev, Real::from(4_i32).sqrt().unwrap());
    }

    #[test]
    fn abs_and_angle_conversions_preserve_exact_real_structure() {
        assert_eq!(Real::from(-7_i32).abs(), Real::from(7_i32));
        assert_eq!((-Real::pi()).abs(), Real::pi());
        assert_eq!(Real::zero().abs(), Real::zero());

        assert_eq!(Real::from(180_i32).to_radians(), Real::pi());
        assert_eq!(Real::pi().to_degrees(), Real::from(180_i32));
        assert_eq!(
            Real::from(45_i32).to_radians().to_degrees(),
            Real::from(45_i32)
        );
    }
}
