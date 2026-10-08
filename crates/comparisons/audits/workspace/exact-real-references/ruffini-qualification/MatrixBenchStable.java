import dk.jonaslindstrom.ruffini.common.matrices.algorithms.*;
import dk.jonaslindstrom.ruffini.common.matrices.elements.Matrix;
import java.math.BigInteger;
import java.lang.management.ManagementFactory;
import java.util.function.BinaryOperator;

/** Follow-up to MatrixBench: warm before calibration; reject short measured batches. */
public final class MatrixBenchStable {
    static final com.sun.management.OperatingSystemMXBean OS=(com.sun.management.OperatingSystemMXBean)ManagementFactory.getOperatingSystemMXBean();
    static volatile int sink;
    static long batch(BinaryOperator<Matrix<BigInteger>> op,Matrix<BigInteger> a,Matrix<BigInteger> b,BigInteger[][] expected,int loops) {
        long start=OS.getProcessCpuTime();
        for(int i=0;i<loops;i++) {
            Matrix<BigInteger> got=op.apply(a,b);
            if(!RuffiniMatrix.matches(got,expected))throw new AssertionError("incorrect benchmark result");
            sink^=got.get(0,0).hashCode();
        }
        return OS.getProcessCpuTime()-start;
    }
    public static void main(String[] args) {
        int n=Integer.parseInt(args[0]),bits=Integer.parseInt(args[1]);
        var aa=RuffiniMatrix.values(n,n,7);var bb=RuffiniMatrix.values(n,n,19);
        for(int i=0;i<n;i++)for(int j=0;j<n;j++) {
            aa[i][j]=aa[i][j].shiftLeft(bits-4).add(BigInteger.valueOf(i+j+1));
            bb[i][j]=bb[i][j].shiftLeft(bits-4).subtract(BigInteger.valueOf(i*3+j+1));
        }
        var expected=RuffiniMatrix.multiply(aa,bb);
        var a=RuffiniMatrix.matrix(aa,false);var b=RuffiniMatrix.matrix(bb,false);
        String[] names={"naive","strassen1","strassen8"};
        @SuppressWarnings("unchecked") BinaryOperator<Matrix<BigInteger>>[] ops=new BinaryOperator[]{new MatrixMultiplication<>(RuffiniMatrix.Z),new StrassenMultiplication<>(RuffiniMatrix.Z,1),new StrassenMultiplication<>(RuffiniMatrix.Z,8)};
        for(int k=0;k<3;k++) {
            long cpu=0;int products=0;
            while(cpu<800_000_000L) {cpu+=batch(ops[k],a,b,expected,16);products+=16;if(products>1_048_576)throw new AssertionError("warmup cap");}
            System.out.println("WARMUP\t"+n+"\t"+bits+"\t"+names[k]+"\t"+products+"\t"+cpu);
        }
        int[] loops=new int[3];
        for(int k=0;k<3;k++) {
            loops[k]=4;long cpu;
            while((cpu=batch(ops[k],a,b,expected,loops[k]))<1_000_000_000L) {loops[k]*=2;if(loops[k]>1_048_576)throw new AssertionError("calibration cap");}
            System.out.println("CALIBRATE\t"+n+"\t"+bits+"\t"+names[k]+"\t"+loops[k]+"\t"+cpu);
        }
        for(int round=-3;round<9;round++)for(int position=0;position<3;position++) {
            int k=Math.floorMod(round+position,3);
            long wallStart=System.nanoTime();long cpu=batch(ops[k],a,b,expected,loops[k]);long wall=System.nanoTime()-wallStart;
            System.out.println("BENCH\t"+n+"\t"+bits+"\t"+round+"\t"+position+"\t"+names[k]+"\t"+loops[k]+"\t"+cpu+"\t"+wall);
            if(round>=0&&cpu<500_000_000L)throw new AssertionError("measured CPU duration below 500ms; entire run unqualified");
        }
    }
}
