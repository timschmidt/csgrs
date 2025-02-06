#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero() {
        assert_eq!(Real::zero(), Real::zero());
    }

    #[test]
    fn parse() {
        let counting: Real = "123456789".parse().unwrap();
        let answer = Real::new(Rational::new(123456789));
        assert_eq!(counting, answer);
    }

    #[test]
    fn parse_large() {
        let input: Real = "378089444731722233953867379643788100".parse().unwrap();
        let root = Rational::new(614889782588491410);
        let answer = Real::new(root.clone() * root);
        assert_eq!(input, answer);
    }

    #[test]
    fn parse_fraction() {
        let input: Real = "98760/123450".parse().unwrap();
        let answer = Real::new(Rational::fraction(9876, 12345));
        assert_eq!(input, answer);
    }

    #[test]
    fn root_divide() {
        let twenty: Real = 20.into();
        let five: Real = 5.into();
        let a = twenty.sqrt().unwrap();
        let b = five.sqrt().unwrap().inverse().unwrap();
        let answer = a * b;
        let two: Real = 2.into();
        assert_eq!(answer, two);
    }

    #[test]
    fn rational() {
        let two: Real = 2.into();
        assert_ne!(two, Real::zero());
        let four: Real = 4.into();
        let answer = four - two;
        let two: Real = 2.into();
        assert_eq!(answer, two);
        let zero = answer - two;
        assert_eq!(zero, Real::zero());
        let six_half: Real = "13/2".parse().unwrap();
        let opposite = six_half.inverse().unwrap();
        let expected: Real = "2/13".parse().unwrap();
        assert_eq!(opposite, expected);
    }

    // https://devblogs.microsoft.com/oldnewthing/?p=93765
    // "Why does the Windows calculator generate tiny errors when calculating the square root of a
    // perfect square?" (fixed in 2018)
    #[test]
    fn perfect_square() {
        let four: Real = 4.into();
        let two: Real = 2.into();
        let calc = four.sqrt().unwrap() - two;
        assert_eq!(calc, Real::zero());
    }

    #[test]
    fn one_over_e() {
        let one: Real = 1.into();
        let e = Real::e();
        let e_inverse = Real::e().inverse().unwrap();
        let answer = e * e_inverse;
        assert_eq!(one, answer);
        let again = answer.sqrt().unwrap();
        assert_eq!(one, again);
    }

    #[test]
    fn unlike_sqrts() {
        let thirty: Real = 30.into();
        let ten: Real = 10.into();
        let answer = thirty.sqrt().unwrap() * ten.sqrt().unwrap();
        let ten: Real = 10.into();
        let three: Real = 3.into();
        let or = ten * three.sqrt().unwrap();
        assert_eq!(answer, or);
    }

    #[test]
    fn zero_pi() {
        let pi = Real::pi();
        let z1 = pi - Real::pi();
        let pi2 = Real::pi() + Real::pi();
        let z2 = pi2 * Real::zero();
        assert!(z1.definitely_zero());
        assert!(z2.definitely_zero());
        let two_pi = Real::pi() + Real::pi();
        let two: Real = 2.into();
        assert_eq!(two_pi, two * Real::pi());
        assert_ne!(two_pi, Rational::new(2));
    }

    #[test]
    fn ln_zero() {
        let zero = Real::zero();
        assert_eq!(zero.ln(), Err(RealProblem::NotANumber));
    }

    #[test]
    fn sqrt_exact() {
        let big: Real = 40_000.into();
        let small: Rational = Rational::new(200);
        let answer = big.sqrt().unwrap();
        assert_eq!(answer, small);
    }

    #[test]
    fn square_sqrt() {
        let two: Real = 2.into();
        let three: Real = 3.into();
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let a = small * two;
        let three: Real = 3.into();
        let small = three.sqrt().expect("Should be able to sqrt(n)");
        let three: Real = 3.into();
        let b = small * three;
        let answer = a * b;
        let eighteen: Rational = Rational::new(18);
        assert_eq!(answer, eighteen);
    }

    #[test]
    fn adding_one_works() {
        let pi = Real::pi();
        let one: Real = 1.into();
        let plus_one = pi + one;
        let float: f64 = plus_one.into();
        assert_eq!(float, 4.141592653589793);
    }

    #[test]
    fn sin_easy() {
        let pi = Real::pi();
        let zero = Real::zero();
        let two: Real = 2.into();
        let two_pi = pi.clone() * two;
        assert_eq!(zero.clone().sin(), zero);
        assert_eq!(pi.clone().sin(), zero);
        assert_eq!(two_pi.clone().sin(), zero);
    }

    #[test]
    fn cos_easy() {
        let pi = Real::pi();
        let zero = Real::zero();
        let one: Real = 1.into();
        let two: Real = 2.into();
        let two_pi = pi.clone() * two;
        let minus_one: Real = (-1).into();
        assert_eq!(zero.clone().cos(), one);
        assert_eq!(pi.clone().cos(), minus_one);
        assert_eq!(two_pi.clone().cos(), one);
    }

    #[test]
    fn sqrt_3045512() {
        let n: Real = 3045512.into();
        let sqrt = n.sqrt().unwrap();
        let root = Rational::new(1234);
        assert_eq!(sqrt.rational, root);
        let two = Rational::new(2);
        assert_eq!(sqrt.class, Sqrt(two));
    }

    #[test]
    fn curves() {
        let eighty = Rational::fraction(80, 100);
        let twenty = Rational::fraction(20, 100);
        assert_eq!(curve(eighty), (false, twenty));
        let forty = Rational::fraction(40, 100);
        let sixty = Rational::fraction(60, 100);
        assert_eq!(curve(sixty), (false, forty));
        let otf = Rational::fraction(124, 100);
        let tf = Rational::fraction(24, 100);
        assert_eq!(curve(otf), (true, tf));
    }

    #[test]
    fn exp_pi() {
        let pi = Real::pi();
        assert_eq!(format!("{pi:.2e}"), "3.14e0");
        assert_eq!(format!("{pi:.4E}"), "3.1416E0");
        assert_eq!(format!("{pi:.8e}"), "3.14159265e0");
        assert_eq!(format!("{pi:.16E}"), "3.1415926535897932E0");
        assert_eq!(format!("{pi:.32e}"), "3.14159265358979323846264338327950e0");
        assert_eq!(format!("{pi:e}"), "3.1415926535897932384626433832795e0");
    }
}
