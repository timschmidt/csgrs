import dk.jonaslindstrom.ruffini.common.abstractions.Ring;
import dk.jonaslindstrom.ruffini.common.algorithms.*;
import dk.jonaslindstrom.ruffini.common.matrices.algorithms.*;
import dk.jonaslindstrom.ruffini.common.matrices.elements.*;
import dk.jonaslindstrom.ruffini.common.util.*;
import dk.jonaslindstrom.ruffini.common.vector.Vector;
import dk.jonaslindstrom.ruffini.integers.structures.*;
import dk.jonaslindstrom.ruffini.integers.structures.limbs.*;
import dk.jonaslindstrom.ruffini.integers.algorithms.CongruenceSolver;
import dk.jonaslindstrom.ruffini.integers.algorithms.ModularSquareRoot;
import dk.jonaslindstrom.ruffini.reals.elements.*;
import dk.jonaslindstrom.ruffini.reals.structures.*;
import java.math.BigInteger;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

public final class RuffiniMatrix {
    static final BigIntegers Z=BigIntegers.getInstance();
    static int count;
    static void probe(String id,Supplier<Boolean> body) {
        try {System.out.println((body.get()?"PASS":"WRONG")+"\t"+id);}
        catch(Throwable e) {System.out.println("EXCEPTION\t"+id+"\t"+e.getClass().getSimpleName());}
        count++;
    }
    static BigInteger[][] values(int m,int n,int seed) {
        BigInteger[][] a=new BigInteger[m][n];
        for(int i=0;i<m;i++)for(int j=0;j<n;j++)a[i][j]=BigInteger.valueOf(Math.floorMod((i+1)*71+(j+3)*37+seed*29+(i+1)*(j+1)*13,31)-15);
        return a;
    }
    static Matrix<BigInteger> matrix(BigInteger[][] a,boolean lazy) {
        return lazy?Matrix.lazy(a.length,a[0].length,(i,j)->a[i][j]):Matrix.of(a.length,a[0].length,(i,j)->a[i][j],true);
    }
    static BigInteger[][] multiply(BigInteger[][] a,BigInteger[][] b) {
        BigInteger[][] c=new BigInteger[a.length][b[0].length];
        for(int i=0;i<a.length;i++)for(int j=0;j<b[0].length;j++) {
            BigInteger x=BigInteger.ZERO;
            for(int k=0;k<b.length;k++)x=x.add(a[i][k].multiply(b[k][j]));
            c[i][j]=x;
        }
        return c;
    }
    static boolean matches(Matrix<BigInteger> a,BigInteger[][] b) {
        if(a.getHeight()!=b.length||a.getWidth()!=b[0].length)return false;
        for(int i=0;i<b.length;i++)for(int j=0;j<b[0].length;j++)if(!a.get(i,j).equals(b[i][j]))return false;
        return true;
    }
    static BigInteger det(BigInteger[][] a) {
        if(a.length==0)return BigInteger.ONE;
        BigInteger result=BigInteger.ZERO;
        for(int j=0;j<a.length;j++) {
            BigInteger[][] minor=new BigInteger[a.length-1][a.length-1];
            for(int i=1;i<a.length;i++)for(int k=0;k<a.length-1;k++)minor[i-1][k]=a[i][k<j?k:k+1];
            BigInteger term=a[0][j].multiply(det(minor));
            result=(j%2==0)?result.add(term):result.subtract(term);
        }
        return result;
    }
    static void arithmetic() {
        for(int seed=0;seed<3;seed++)for(int m:new int[]{1,2,3,4,5,8})for(int n:new int[]{1,2,3,4,5,8})for(int p:new int[]{1,2,3,4,5,8}) {
            var a=values(m,n,seed);var b=values(n,p,seed+17);var expected=multiply(a,b);
            for(boolean lazy:new boolean[]{false,true})probe("mul-"+seed+"-"+m+"-"+n+"-"+p+"-"+lazy,()->matches(new MatrixMultiplication<>(Z).apply(matrix(a,lazy),matrix(b,lazy)),expected));
        }
        for(int seed=0;seed<3;seed++)for(int n:new int[]{1,2,3,4,5,6,7,8,9,12,16})for(int bound:new int[]{1,2,4,8})for(boolean lazy:new boolean[]{false,true}) {
            var a=values(n,n,seed);var b=values(n,n,seed+17);var expected=multiply(a,b);
            probe("strassen-"+seed+"-"+n+"-"+bound+"-"+lazy,()->matches(new StrassenMultiplication<>(Z,bound).apply(matrix(a,lazy),matrix(b,lazy)),expected));
        }
        for(int m=1;m<=8;m++)for(int n=1;n<=8;n++)for(boolean lazy:new boolean[]{false,true}) {
            var a=values(m,n,13);BigInteger[][] t=new BigInteger[n][m];
            for(int i=0;i<m;i++)for(int j=0;j<n;j++)t[j][i]=a[i][j];
            var expected=multiply(a,t);probe("gram-"+m+"-"+n+"-"+lazy,()->matches(new GramMatrix<>(Z).apply(matrix(a,lazy)),expected));
        }
        for(int n=1;n<=5;n++)for(int seed=0;seed<12;seed++)for(boolean lazy:new boolean[]{false,true}) {
            var a=values(n,n,seed);probe("det-"+n+"-"+seed+"-"+lazy,()->new Determinant<>(Z).apply(matrix(a,lazy)).equals(det(a)));
        }
        probe("det-empty-lazy",()->new Determinant<>(Z).apply(Matrix.lazy(0,0,(i,j)->BigInteger.ZERO)).equals(BigInteger.ONE));
    }
    static void boundaries() {
        for(int m=1;m<=4;m++)for(int n=1;n<=4;n++)for(boolean lazy:new boolean[]{false,true}) {
            var a=values(m,n,31);var x=matrix(a,lazy);var y=matrix(a,lazy);
            probe("matrix-equals-"+m+"-"+n+"-"+lazy,()->x.equals(y,Z::equals));
            BigInteger[] sums=new BigInteger[n];Arrays.fill(sums,BigInteger.ZERO);
            for(int i=0;i<m;i++)for(int j=0;j<n;j++)sums[j]=sums[j].add(a[i][j]);
            probe("collapse-columns-"+m+"-"+n+"-"+lazy,()->x.collapseColumns(BigInteger.ZERO,BigInteger::add).asList().equals(Arrays.asList(sums)));
        }
        probe("complex-identity-inverse",()-> {
            var f=ComplexNumbers.getInstance();var x=new MatrixInversion<>(f).apply(Matrix.eye(1,f));
            return f.equals(x.get(0,0),f.identity());
        });
        probe("real-identity-inverse",()-> {
            var f=RealNumbers.getInstance();var x=new MatrixInversion<>(f).apply(Matrix.eye(1,f));
            return f.equals(x.get(0,0),f.identity());
        });
        probe("sparse-builder-snapshot",()-> {
            var b=new SparseMatrix.Builder<BigInteger>(2,2,BigInteger.ZERO);b.add(0,0,BigInteger.ONE);var x=b.build();b.add(0,0,BigInteger.TWO);return x.get(0,0).equals(BigInteger.ONE);
        });
        probe("mutable-copy-is-independent",()-> {var a=Matrix.of(2,2,BigInteger.ONE);var b=a.mutable();b.set(0,0,BigInteger.TWO);return a.get(0,0).equals(BigInteger.ONE);});
        probe("vector-unequal-length",()->!Vector.of(1).equals(Vector.of(1,2),Integer::equals));
        probe("vector-shorter-right",()->!Vector.of(1,2).equals(Vector.of(1),Integer::equals));
        probe("complex-projection",()-> {
            var v=new ComplexCoordinateSpace(1);var u=Vector.of(new ComplexNumber(1,0));var x=Vector.of(new ComplexNumber(0,1));
            var got=new Projection<>(v).apply(x,u);return v.equals(x,got);
        });
        probe("qr-tall",()-> {
            var f=RealNumbers.getInstance();var v=new RealCoordinateSpace(3);var a=Matrix.of(3,2,(i,j)->i==j?1.0:0.0,true);
            var qr=new QRDecomposition<>(v,x->v.scale(1.0/v.norm(x),x)).apply(a);
            return new MatrixMultiplication<>(f).apply(qr.first,qr.second).equals(a,f::equals);
        });
        probe("ksubsets-list",()->ArrayUtils.ksubsets(2,List.of(0,1,2,3)).count()==6);
        probe("ksubsets-array-control",()->ArrayUtils.ksubsets(2,new int[]{0,1,2,3}).count()==6);
        probe("multidimensional-width-constructor",()-> {var a=MultiDimensionalArray.build(3,i->MultiDimensionalArray.build(List.of(i,10+i)));return a.getDimension()==2&&a.get(2,1)==12;});
        probe("multidimensional-shape-control",()-> {var a=MultiDimensionalArray.build(List.of(3,2),i->i.get(0)*10+i.get(1));return a.getDimension()==2&&a.get(2,1)==21;});
        for(int m:new int[]{5,7,11,17})for(int p=1;p<m;p++) {
            final int value=p,mod=m;
            for(int q=0;q<m;q++) {
                final int rhs=q;
                probe("congruence-"+m+"-"+p+"-"+q,()->new CongruenceSolver(BigInteger.valueOf(mod)).solve(List.of(BigInteger.valueOf(value)),List.of(BigInteger.valueOf(rhs))).multiply(BigInteger.valueOf(value)).mod(BigInteger.valueOf(mod)).intValue()==rhs);
            }
        }
        for(int m:new int[]{3,5,7,11,13,17,19})for(int a=0;a<m;a++) {
            int residue=(a*a)%m;BigInteger p=BigInteger.valueOf(m),x=BigInteger.valueOf(residue);
            probe("modsqrt-"+m+"-"+a,()->new ModularSquareRoot(p).apply(x).pow(2).mod(p).equals(x));
        }
        var limbs=new BigElements<>(Z,BigInteger.TEN);
        probe("limb-carry",()->limbs.equals(limbs.add(new BigElement<>(List.of(BigInteger.valueOf(9))),limbs.identity()),new BigElement<>(List.of(BigInteger.ZERO,BigInteger.ONE))));
        probe("limb-multiply",()->limbs.multiply(limbs.identity(),limbs.identity())!=null);
        probe("int-min-norm",()->Integers.getInstance().norm(Integer.MIN_VALUE).equals(BigInteger.ONE.shiftLeft(31)));
        probe("int-division-overflow",()-> {var qr=Integers.getInstance().divide(Integer.valueOf(Integer.MIN_VALUE),Integer.valueOf(3));return BigInteger.valueOf(qr.first).multiply(BigInteger.valueOf(3)).add(BigInteger.valueOf(qr.second)).equals(BigInteger.valueOf(Integer.MIN_VALUE));});
    }
    static void transforms() {
        for(int modulus:new int[]{7,13,17,31}) {
            TestUtils.TestField f=new TestUtils.TestField(modulus);
            int generator=2;while(true) {int v=1;boolean primitive=true;for(int k=1;k<modulus-1;k++){v=v*generator%modulus;if(v==1){primitive=false;break;}}if(primitive)break;generator++;}
            for(int n=1;n<modulus;n++)if((modulus-1)%n==0) {
                final int size=n,root=BigInteger.valueOf(generator).modPow(BigInteger.valueOf((modulus-1)/n),BigInteger.valueOf(modulus)).intValue();
                for(int seed=0;seed<3;seed++) {
                    int s=seed;Vector<Integer> input=Vector.of(n,i->Math.floorMod(i*7+s*3,modulus));
                    int[] expected=new int[n];for(int k=0;k<n;k++)for(int j=0;j<n;j++)expected[k]=(expected[k]+input.get(j)*BigInteger.valueOf(root).modPow(BigInteger.valueOf(j*k),BigInteger.valueOf(modulus)).intValue())%modulus;
                    probe("dft-"+modulus+"-"+n+"-"+seed,()-> {
                        var got=new DiscreteFourierTransform<>(f,root,size).apply(input);
                        for(int i=0;i<size;i++)if(got.get(i)!=expected[i])return false;
                        var back=new InverseDiscreteFourierTransform<>(f,root,size).apply(got);
                        return back.equals(input,f::equals);
                    });
                }
            }
        }
    }
    public static void main(String[] args) {
        if(args[0].equals("arithmetic"))arithmetic();else if(args[0].equals("boundaries"))boundaries();else if(args[0].equals("transforms"))transforms();else throw new IllegalArgumentException();
        System.out.println("SUMMARY\t"+count);
    }
}
