import dk.jonaslindstrom.ruffini.polynomials.elements.Polynomial;
import dk.jonaslindstrom.ruffini.polynomials.algorithms.KaratsubaAlgorithm;
import dk.jonaslindstrom.ruffini.common.vector.Vector;
import java.math.BigInteger;
import java.lang.management.ManagementFactory;
import java.util.function.BinaryOperator;

/** Donor-only screen; prepared inputs, every coefficient checked inside the timer. */
public final class PolynomialBench {
    static final com.sun.management.OperatingSystemMXBean OS=(com.sun.management.OperatingSystemMXBean)ManagementFactory.getOperatingSystemMXBean();
    static volatile int sink;
    static long batch(BinaryOperator<Polynomial<BigInteger>> op,Polynomial<BigInteger> a,Polynomial<BigInteger> b,BigInteger[] expected,int loops) {
        long start=OS.getProcessCpuTime();
        for(int i=0;i<loops;i++) {var p=op.apply(a,b);if(!PolynomialContracts.matches(p,expected))throw new AssertionError("incorrect benchmark product");sink^=p.getCoefficient(p.degree()).hashCode();}
        return OS.getProcessCpuTime()-start;
    }
    static BigInteger[] coefficients(int n,int bits,int seed,boolean sparse) {
        BigInteger[] a=PolynomialContracts.values(n,seed);
        for(int i=0;i<n;i++) {
            if(sparse&&i%8!=0&&i!=n-1){a[i]=BigInteger.ZERO;continue;}
            if(a[i].signum()==0)a[i]=BigInteger.ONE;
            a[i]=a[i].shiftLeft(bits-4).add(BigInteger.valueOf(3L*i+seed+1));
        }
        return a;
    }
    public static void main(String[] args) {
        int n=Integer.parseInt(args[0]),bits=Integer.parseInt(args[1]);boolean sparse=args[2].equals("sparse");
        var aa=coefficients(n,bits,3,sparse);var bb=coefficients(n,bits,11,sparse);var expected=PolynomialContracts.product(aa,bb);
        var a=new Polynomial<>(Vector.of(aa),PolynomialContracts.Z);var b=new Polynomial<>(Vector.of(bb),PolynomialContracts.Z);
        String[] names={"ordinary","karatsuba"};
        @SuppressWarnings("unchecked") BinaryOperator<Polynomial<BigInteger>>[] ops=new BinaryOperator[]{(BinaryOperator<Polynomial<BigInteger>>)PolynomialContracts.ZP::multiply,new KaratsubaAlgorithm<>(PolynomialContracts.ZP)};
        int[] loops=new int[2];
        for(int k=0;k<2;k++) {
            long cpu=0;int products=0;while(cpu<500_000_000L){cpu+=batch(ops[k],a,b,expected,4);products+=4;if(products>1_048_576)throw new AssertionError("warmup cap");}
            System.out.println("WARMUP\t"+names[k]+"\t"+products+"\t"+cpu);
        }
        for(int k=0;k<2;k++) {
            loops[k]=2;long cpu;while((cpu=batch(ops[k],a,b,expected,loops[k]))<1_000_000_000L){loops[k]*=2;if(loops[k]>1_048_576)throw new AssertionError("calibration cap");}
            System.out.println("CALIBRATE\t"+names[k]+"\t"+loops[k]+"\t"+cpu);
        }
        for(int round=-3;round<9;round++)for(int position=0;position<2;position++) {
            int k=Math.floorMod(round+position,2);long wallStart=System.nanoTime();long cpu=batch(ops[k],a,b,expected,loops[k]);long wall=System.nanoTime()-wallStart;
            System.out.println("BENCH\t"+round+"\t"+position+"\t"+names[k]+"\t"+loops[k]+"\t"+cpu+"\t"+wall);
            if(round>=0&&cpu<500_000_000L)throw new AssertionError("short measured batch; entire run unqualified");
        }
    }
}
