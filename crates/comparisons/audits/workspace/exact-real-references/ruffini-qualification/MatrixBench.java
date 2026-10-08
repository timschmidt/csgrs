import dk.jonaslindstrom.ruffini.common.matrices.algorithms.*;
import dk.jonaslindstrom.ruffini.common.matrices.elements.Matrix;
import java.math.BigInteger;
import java.lang.management.ManagementFactory;
import java.util.function.BinaryOperator;

/** Exact-result-checked donor comparison; preparation excluded, result verification included. */
public final class MatrixBench {
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
        int[] loops=new int[3];
        for(int k=0;k<3;k++) {
            loops[k]=4;
            while(batch(ops[k],a,b,expected,loops[k])<150_000_000L) {loops[k]*=2;if(loops[k]>131072)throw new AssertionError("calibration cap");}
            System.out.println("CALIBRATE\t"+n+"\t"+bits+"\t"+names[k]+"\t"+loops[k]);
        }
        for(int round=-3;round<9;round++)for(int position=0;position<3;position++) {
            int k=Math.floorMod(round+position,3);
            long wallStart=System.nanoTime();long cpu=batch(ops[k],a,b,expected,loops[k]);long wall=System.nanoTime()-wallStart;
            if(cpu<=0)throw new AssertionError("CPU timer resolution");
            System.out.println("BENCH\t"+n+"\t"+bits+"\t"+round+"\t"+position+"\t"+names[k]+"\t"+loops[k]+"\t"+cpu+"\t"+wall);
        }
    }
}
