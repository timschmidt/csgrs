import dk.jonaslindstrom.ruffini.integers.structures.BigIntegers;
import dk.jonaslindstrom.ruffini.finitefields.PrimeField;
import dk.jonaslindstrom.ruffini.polynomials.elements.Polynomial;
import dk.jonaslindstrom.ruffini.polynomials.elements.MultivariatePolynomial;
import dk.jonaslindstrom.ruffini.polynomials.structures.PolynomialRingOverRing;
import dk.jonaslindstrom.ruffini.polynomials.structures.MultivariatePolynomialRing;
import java.math.BigInteger;

/** Stops after a verified finite prefix without changing any donor arithmetic result. */
public final class PolynomialProgress {
    static final class Budget extends RuntimeException {}
    static final class Univariate extends PolynomialRingOverRing<BigInteger> {
        int iterations;
        Univariate(){super(BigIntegers.getInstance());}
        @Override public boolean equals(Polynomial<BigInteger> a,Polynomial<BigInteger> b) {
            if(!BigInteger.ONE.equals(a.getCoefficient(0))||!BigInteger.ZERO.equals(a.getCoefficient(1))||a.degree()!=1||!BigInteger.ZERO.equals(b.getCoefficient(0))||b.degree()!=0)throw new AssertionError("unexpected state");
            iterations++;if(iterations>1024)throw new Budget();return super.equals(a,b);
        }
    }
    static final class Multivariate extends MultivariatePolynomialRing<Integer> {
        int iterations;
        int expected=1;
        Multivariate(){super(new PrimeField(5),2);}
        @Override public MultivariatePolynomial<Integer> add(MultivariatePolynomial<Integer> a,MultivariatePolynomial<Integer> b) {
            if(a.getCoefficient(1,0)!=expected||b.getCoefficient(1,0)!=expected)throw new AssertionError("unexpected leading coefficients");
            iterations++;if(iterations>1024)throw new Budget();
            expected=expected*2%5;
            return super.add(a,b);
        }
    }
    public static void main(String[] args) {
        var u=new Univariate();try{u.divisionWithRemainder(Polynomial.of(BigInteger.ONE,BigInteger.ZERO),u.identity());throw new AssertionError("unexpected completion");}catch(Budget b){System.out.println("PREFIX\tunivariate\t"+u.iterations+"\tunchanged-1-plus-0x");}
        var m=new Multivariate();var x=MultivariatePolynomial.monomial(1,1,0);try{m.divide(x,x);throw new AssertionError("unexpected completion");}catch(Budget b){System.out.println("PREFIX\tmultivariate\t"+m.iterations+"\tcycle-1-2-4-3");}
    }
}
