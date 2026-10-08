import dk.jonaslindstrom.ruffini.common.algorithms.BarrettReduction;
import dk.jonaslindstrom.ruffini.common.algorithms.BinaryGCD;
import dk.jonaslindstrom.ruffini.common.algorithms.BitLength;
import dk.jonaslindstrom.ruffini.common.helpers.PerformanceLoggingField;
import dk.jonaslindstrom.ruffini.common.util.TestUtils;
import dk.jonaslindstrom.ruffini.integers.structures.BigRationals;
import dk.jonaslindstrom.ruffini.integers.structures.BigIntegersModuloN;
import dk.jonaslindstrom.ruffini.reals.elements.ConstructiveReal;
import dk.jonaslindstrom.ruffini.reals.structures.ConstructiveReals;
import java.math.BigInteger;
import java.util.concurrent.atomic.AtomicInteger;

public final class RuffiniShared {
    public static void main(String[] args) {
        if(args[0].equals("binary-gcd-zero")) {
            System.out.println("ENTER\tbinary-gcd-zero");System.out.flush();
            System.out.println(BinaryGCD.apply(BigInteger.ZERO,BigInteger.ONE));return;
        }
        if(args[0].equals("dag")) {
            for(int depth:new int[]{0,1,2,4,8,12,16,20}) {
                AtomicInteger calls=new AtomicInteger();
                ConstructiveReal x=new ConstructiveReal(i->{calls.incrementAndGet();return BigInteger.ONE.shiftLeft(i);},"x");
                for(int i=0;i<depth;i++)x=ConstructiveReal.add(x,x);
                int constructed=calls.get(),length=x.toString().length();
                BigInteger got=x.apply(128);
                if(!got.equals(BigInteger.ONE.shiftLeft(128+depth)))throw new AssertionError();
                int refined=calls.get();
                for(int j=0;j<100;j++) {
                    if(!x.apply(64).equals(BigInteger.ONE.shiftLeft(64+depth)))throw new AssertionError();
                }
                System.out.println("DAG\t"+depth+"\t"+length+"\t"+constructed+"\t"+refined+"\t"+calls.get());
            }
            return;
        }
        ConstructiveReals field=new ConstructiveReals();
        PerformanceLoggingField<ConstructiveReal> logged=new PerformanceLoggingField<>(field);
        ConstructiveReal small=ConstructiveReal.of(Math.scalb(1.0,-17));
        ConstructiveReal one=ConstructiveReal.of(1),two=ConstructiveReal.of(2);
        System.out.println("LOGGING\tadd\t"+field.add(small,one).apply(128)+"\t"+logged.add(small,one).apply(128));
        System.out.println("LOGGING\tmul\t"+field.multiply(small,two).apply(128)+"\t"+logged.multiply(small,two).apply(128));
        for(int modulus:new int[]{5,7,9,17,31}) {
            BarrettReduction reduction=new BarrettReduction(BigInteger.valueOf(modulus));
            for(int n=-200;n<=200;n++)
                System.out.println("BARRETT\t"+modulus+"\t"+n+"\t"+reduction.apply(BigInteger.valueOf(n)));
            BigIntegersModuloN ring=new BigIntegersModuloN(modulus);
            System.out.println("QUOTIENT\t"+modulus+"\t"+ring.equals(BigInteger.valueOf(-modulus),BigInteger.ZERO));
        }
        BigRationals q=BigRationals.getInstance();
        var invalid=q.invert(q.zero());
        System.out.println("FRACTION\tinvert-zero\t"+invalid.numerator()+"\t"+invalid.denominator());
        try {System.out.println("BITLENGTH\t"+new BitLength<>(new TestUtils.TestBigIntegers(),BigInteger::shiftRight).bitLength(BigInteger.ONE,64));}
        catch(StackOverflowError e) {System.out.println("STACK\tbitlength-one");}
    }
}
