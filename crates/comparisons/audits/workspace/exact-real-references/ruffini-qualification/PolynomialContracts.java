import dk.jonaslindstrom.ruffini.common.abstractions.*;
import dk.jonaslindstrom.ruffini.common.elements.Fraction;
import dk.jonaslindstrom.ruffini.common.util.Pair;
import dk.jonaslindstrom.ruffini.common.vector.Vector;
import dk.jonaslindstrom.ruffini.integers.IntegerPolynomial;
import dk.jonaslindstrom.ruffini.integers.structures.*;
import dk.jonaslindstrom.ruffini.polynomials.elements.*;
import dk.jonaslindstrom.ruffini.polynomials.structures.*;
import dk.jonaslindstrom.ruffini.polynomials.algorithms.*;
import dk.jonaslindstrom.ruffini.finitefields.*;
import dk.jonaslindstrom.ruffini.finitefields.algorithms.BigTonelliShanks;
import dk.jonaslindstrom.ruffini.permutations.elements.Permutation;
import dk.jonaslindstrom.ruffini.permutations.algorithms.RandomDerangement;
import dk.jonaslindstrom.arithmeticparser.*;
import java.math.BigInteger;
import java.util.*;

/** Unchanged donor inputs; independent mathematical oracles, never donor polynomial equality. */
public final class PolynomialContracts {
    static final BigIntegers Z=BigIntegers.getInstance();
    static final PolynomialRingOverRing<BigInteger> ZP=new PolynomialRingOverRing<>(Z);
    static int count;
    @FunctionalInterface interface Check {boolean run() throws Exception;}
    static void probe(String id,Check c) {
        try {System.out.println((c.run()?"PASS":"WRONG")+"\t"+id);}
        catch(Throwable t) {System.out.println("EXCEPTION\t"+id+"\t"+t.getClass().getSimpleName());}
        count++;
    }
    static BigInteger[] values(int length,int seed) {
        BigInteger[] a=new BigInteger[length];for(int i=0;i<length;i++)a[i]=BigInteger.valueOf(Math.floorMod(i*17+seed*13+i*i*3,11)-5);
        if(a[length-1].signum()==0)a[length-1]=BigInteger.ONE;return a;
    }
    static BigInteger[] product(BigInteger[] a,BigInteger[] b) {
        BigInteger[] c=new BigInteger[a.length+b.length-1];Arrays.fill(c,BigInteger.ZERO);
        for(int i=0;i<a.length;i++)for(int j=0;j<b.length;j++)c[i+j]=c[i+j].add(a[i].multiply(b[j]));return c;
    }
    static BigInteger evaluate(BigInteger[] a,BigInteger x) {BigInteger r=BigInteger.ZERO;for(int i=a.length-1;i>=0;i--)r=r.multiply(x).add(a[i]);return r;}
    static boolean matches(Polynomial<BigInteger> p,BigInteger[] expected) {
        boolean[] valid={true};p.forEach((i,c)-> {if(i<0||i>=expected.length){if(c.signum()!=0)valid[0]=false;}});
        for(int i=0;i<expected.length;i++){BigInteger c=p.getCoefficient(i);if(!(c==null?BigInteger.ZERO:c).equals(expected[i]))return false;}return valid[0];
    }
    static boolean matchesMod(Polynomial<Integer> p,int[] expected,int modulus) {
        boolean[] valid={true};p.forEach((i,c)->{if(i<0||i>=expected.length){if(Math.floorMod(c,modulus)!=0)valid[0]=false;}});
        for(int i=0;i<expected.length;i++){Integer c=p.getCoefficient(i);if(Math.floorMod(c==null?0:c,modulus)!=Math.floorMod(expected[i],modulus))return false;}return valid[0];
    }
    static void arithmetic() {
        for(int n:new int[]{1,2,3,4,5,8,9,16})for(int m:new int[]{1,2,3,4,5,8,9,16})for(int seed=0;seed<5;seed++) {
            var a=values(n,seed);var b=values(m,seed+11);var expected=product(a,b);
            for(boolean sparse:new boolean[]{false,true}) {
                var x=sparse?new Polynomial<>(Vector.of(a),Z):Polynomial.of(a);var y=sparse?new Polynomial<>(Vector.of(b),Z):Polynomial.of(b);
                probe("multiply-"+n+"-"+m+"-"+seed+"-"+sparse,()->matches(ZP.multiply(x,y),expected));
                probe("karatsuba-"+n+"-"+m+"-"+seed+"-"+sparse,()->matches(new KaratsubaAlgorithm<>(ZP).apply(x,y),expected));
            }
        }
        for(int n=1;n<=12;n++)for(int seed=0;seed<5;seed++)for(int x=-3;x<=3;x++) {
            var a=values(n,seed);var p=Polynomial.of(a);var value=BigInteger.valueOf(x);
            probe("evaluate-"+n+"-"+seed+"-"+x,()->p.apply(value,Z).equals(evaluate(a,value)));
        }
        for(int n=1;n<=8;n++)for(int seed=0;seed<5;seed++)for(int length=1;length<=12;length++) {
            var a=values(n,seed);a[0]=BigInteger.ONE;var p=Polynomial.of(a);final int l=length;
            probe("inversion-"+n+"-"+seed+"-"+l,()-> {var g=new Inversion<>(Z).apply(p,l);var fg=ZP.multiply(p,g);for(int i=0;i<l;i++){var c=fg.getCoefficient(i);if(!(c==null?BigInteger.ZERO:c).equals(i==0?BigInteger.ONE:BigInteger.ZERO))return false;}return true;});
        }
        for(int n=1;n<=9;n++)for(int m=1;m<=5;m++)for(int seed=0;seed<5;seed++) {
            var q=values(n,seed);var b=values(m,seed+7);b[m-1]=BigInteger.ONE;var a=product(q,b);var pa=Polynomial.of(a);var pb=Polynomial.of(b);
            probe("division-"+n+"-"+m+"-"+seed,()->{var qr=ZP.divisionWithRemainder(pa,pb);return matches(qr.first,q)&&matches(qr.second,new BigInteger[]{BigInteger.ZERO});});
            probe("fastdivision-"+n+"-"+m+"-"+seed,()->{var qr=new FastDivision<>(ZP).apply(pa,pb);return matches(qr.first,q)&&matches(qr.second,new BigInteger[]{BigInteger.ZERO});});
        }
    }
    static void boundaries() {
        var x=Polynomial.monomial(BigInteger.ONE,1);
        probe("sparse-equality-zero",()->ZP.equals(x,Polynomial.of(BigInteger.ZERO,BigInteger.ONE)));
        probe("sparse-equality-nonzero",()->!ZP.equals(x,Polynomial.of(BigInteger.ONE,BigInteger.ONE)));
        probe("trailing-zero-equality",()->ZP.equals(Polynomial.of(BigInteger.ONE,BigInteger.ZERO),ZP.identity()));
        probe("constant-derivative-zero",()->ZP.isZero(ZP.identity().differentiate(Z)));
        probe("empty-polynomial-zero",()->ZP.isZero(Polynomial.<BigInteger>of()));
        probe("builder-snapshot",()->{var b=new Polynomial.Builder<>(Z);b.set(0,BigInteger.ONE);var p=b.build();b.set(0,BigInteger.TWO);return matches(p,new BigInteger[]{BigInteger.ONE});});
        probe("copy-snapshot",()->{var b=new Polynomial.Builder<>(Z);b.set(0,BigInteger.ONE);var p=new Polynomial<>(b.build());b.set(0,BigInteger.TWO);return matches(p,new BigInteger[]{BigInteger.ONE});});
        probe("fastdivision-monomial",()->{var qr=new FastDivision<>(ZP).apply(Polynomial.monomial(BigInteger.ONE,2),x);return matches(qr.first,new BigInteger[]{BigInteger.ZERO,BigInteger.ONE})&&matches(qr.second,new BigInteger[]{BigInteger.ZERO});});
        var f=new PrimeField(17);var fp=new PolynomialRing<>(f);var kfp=new PolynomialRingKaratsuba<>(f);
        probe("field-division-nonmonic-control",()->{var qr=fp.divide(Polynomial.of(2,4),Polynomial.of(2));return matchesMod(qr.first,new int[]{1,2},17)&&matchesMod(qr.second,new int[]{0},17);});
        probe("karatsuba-field-division-nonmonic",()->{var qr=kfp.divide(Polynomial.of(2,4),Polynomial.of(2));return matchesMod(qr.first,new int[]{1,2},17)&&matchesMod(qr.second,new int[]{0},17);});
        for(int n:new int[]{1,2,3,4,8}) {
            final int size=n;var input=new ArrayList<BigInteger>();for(int i=0;i<n;i++)input.add(BigInteger.valueOf(i));
            probe("batch-zero-"+n,()->new BatchPolynomialEvaluation<>(ZP,input).apply(ZP.zero()).equals(Collections.nCopies(size,BigInteger.ZERO)));
            probe("batch-sparse-"+n,()->new BatchPolynomialEvaluation<>(ZP,input).apply(Polynomial.monomial(BigInteger.ONE,2)).equals(input.stream().map(a->a.multiply(a)).toList()));
        }
        for(int n:new int[]{1,2,4,8}) {
            var xs=new ArrayList<Integer>();var ys=new ArrayList<Integer>();for(int i=0;i<n;i++){xs.add(i);ys.add(i*i%17);}
            probe("tree-interpolation-"+n,()->{var p=new PolynomialInterpolation<>(fp,xs).apply(ys);for(int i=0;i<xs.size();i++)if(!p.apply(xs.get(i),f).equals(ys.get(i)))return false;return true;});
            probe("lagrange-control-"+n,()->{var p=new LagrangePolynomial<>(f).apply(xs,ys);for(int i=0;i<xs.size();i++)if(!p.apply(xs.get(i),f).equals(ys.get(i)))return false;return true;});
        }
        var fft=new PolynomialRingFFT<>(f,3,16);
        probe("fft-cyclic-not-polynomial",()->matchesMod(fft.toPolynomial(fft.multiply(fft.monomial(15),fft.monomial(1))),new int[]{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},17));
        probe("fft-oversize-import",()->matchesMod(fft.toPolynomial(fft.fromPolynomial(Polynomial.monomial(1,16))),new int[]{1},17));
        probe("fft-zero-polynomial",()->fp.isZero(fft.toPolynomial(fft.zero())));
        var mr=new MultivariatePolynomialRingOverRing<>(Z,2);
        probe("multivariate-leading-coefficient",()->{var b=new MultivariatePolynomial.Builder<>(2,Z);var p=b.add(BigInteger.TWO,0,0).add(BigInteger.valueOf(3),1,0).build();return p.leadingCoefficient().equals(BigInteger.valueOf(3));});
        probe("multivariate-snapshot",()->{var b=new MultivariatePolynomial.Builder<>(2,Z);var p=b.add(BigInteger.ONE,0,0).build();b.add(BigInteger.ONE,0,0);return p.apply(Vector.of(BigInteger.ZERO,BigInteger.ZERO),Z).equals(BigInteger.ONE);});
        probe("multivariate-equality-explicit-zero",()->mr.equals(MultivariatePolynomial.monomial(BigInteger.ONE,1,0),MultivariatePolynomial.monomial(BigInteger.ONE,1,0).mapCoefficients(c->c)));
        probe("monomial-array-snapshot",()->{int[] d={1,2};var m=Monomial.of(d);d[0]=7;return m.degree(0)==1;});
        probe("recursive-add-alias",()->{var a=dk.jonaslindstrom.ruffini.polynomials.elements.recursive.Polynomial.<BigInteger>create(1);a.set(BigInteger.ONE,0);var z=dk.jonaslindstrom.ruffini.polynomials.elements.recursive.Polynomial.<BigInteger>create(1);var sum=a.add(z,Z);a.set(BigInteger.TWO,0);return sum.apply(List.of(BigInteger.ZERO),Z).equals(BigInteger.ONE);});
    }
    static void parser() {
        var parser=Parser.getDefault();var funcs=Map.of("f",new MultiOperator<Double>(a->a*a));var eval=Evaluator.getDefault(funcs);
        String[] expressions={"1+2*3","(1+2)*3","8-3+1","8+3-1","12/3*2","12*3/2","2^3^2","f(2)+3","f(2)*3","1*-2","1+(-2)","2(3+4)"};
        double[] expected={7,9,6,10,8,18,512,7,12,-2,-1,14};
        for(int i=0;i<expressions.length;i++){String expression=expressions[i];double wanted=expected[i];probe("parser-"+i,()->{double got=eval.evaluate(parser.parse(expression,List.of(),List.of("f")),Map.of());System.out.println("OBS\tparser-"+expression+"\t"+got);return got==wanted;});}
        for(String malformed:List.of(")","1,2","(1"))probe("parser-malformed-"+malformed,()->{try{eval.evaluate(parser.parse(malformed,List.of(),List.of()),Map.of());return false;}catch(java.text.ParseException|EvaluationException e){return true;}});
        probe("integer-polynomial-repeated-power",()->IntegerPolynomial.parse("x+x").apply(3,Integers.getInstance())==6);
        probe("integer-polynomial-negative-implicit",()->IntegerPolynomial.parse("-x").apply(3,Integers.getInstance())==-3);
    }
    static class Draws extends Random {final int[] draws;int index;Draws(int[] d){draws=d;}@Override public int nextInt(int bound){int v=draws[index++];if(v<0||v>=bound)throw new AssertionError("invalid scripted draw");return v;}}
    static void enumerate(int n,int position,int[] draws,Map<String,Integer> counts) {
        if(position==n-1){var p=Permutation.samplePermutation(n,new Draws(draws));int[] values=new int[n];for(int i=0;i<n;i++)values[i]=p.apply(i);counts.merge(Arrays.toString(values),1,Integer::sum);return;}
        for(int i=0;i<n-position;i++){draws[position]=i;enumerate(n,position+1,draws,counts);}
    }
    static void permutations() {
        for(int n=2;n<=7;n++){final int size=n;probe("uniform-"+n,()->{Map<String,Integer> counts=new TreeMap<>();enumerate(size,0,new int[size-1],counts);int total=counts.values().stream().mapToInt(x->x).sum();System.out.println("OBS\tuniform-"+size+"\t"+total+"\t"+counts.size()+"\t"+Collections.min(counts.values())+"\t"+Collections.max(counts.values()));return counts.size()==total&&counts.values().stream().allMatch(x->x==1);});}
        probe("permutation-array-snapshot",()->{int[] a={0,1};var p=new Permutation(a);a[0]=1;return p.apply(0)==0;});
    }
    static void fields() {
        for(int prime:new int[]{3,5,7,11,17,31}) {
            var f=new PrimeField(prime);var finite=new FiniteField(f,Polynomial.of(0,1));
            for(int a=1;a<prime;a++){final int value=a;probe("finite-inverse-"+prime+"-"+a,()->matchesMod(finite.multiply(Polynomial.constant(value),finite.invert(Polynomial.constant(value))),new int[]{1},prime));}
            for(int a=1;a<prime;a++){final int value=a;probe("base-prime-inverse-"+prime+"-"+a,()->Math.floorMod(value*f.invert(value),prime)==1);}
        }
    }
    static void field(BigInteger prime) {
        var base=new BigPrimeField(prime);var finite=new BigFiniteField(base,Polynomial.of(BigInteger.ZERO,BigInteger.ONE));
        var normalized=new AlgebraicFieldExtension<>(base,"a",Polynomial.of(BigInteger.ZERO,BigInteger.ONE));
        for(int a=1;a<prime.intValueExact();a++){var value=BigInteger.valueOf(a);probe("big-finite-inverse-"+prime+"-"+a,()->matches(finite.multiply(Polynomial.constant(value),finite.invert(Polynomial.constant(value))),new BigInteger[]{BigInteger.ONE}));probe("normalized-extension-inverse-"+prime+"-"+a,()->matches(normalized.multiply(Polynomial.constant(value),normalized.invert(Polynomial.constant(value))),new BigInteger[]{BigInteger.ONE}));}
    }
    static void gaussian(boolean real) {
        AlgebraicFieldExtension<Fraction<Integer>,Rationals> f=real?new QuadraticField(2):new GaussianRationals();
        var unit=new Fraction<Integer>(1,1);var x=Polynomial.monomial(unit,1);
        probe(real?"quadratic-generator-square":"gaussian-generator-square",()->{var p=f.multiply(x,x);var expected=new Fraction<Integer>(real?2:-1,1);var c=p.getCoefficient(0);return c!=null&&c.denominator()!=0&&Rationals.getInstance().equals(c,expected)&&p.degree()==0;});
    }
    public static void main(String[] args) {
        switch(args[0]) {
            case "arithmetic" -> arithmetic();case "boundaries" -> boundaries();case "parser" -> parser();case "permutations" -> permutations();case "fields" -> fields();
            case "bigfields" -> {for(int p:new int[]{5,7,11,17,31})field(BigInteger.valueOf(p));}
            case "gaussian" -> gaussian(false);case "quadratic" -> gaussian(true);
            case "trailing-division" -> probe("trailing-division",()->{ZP.divisionWithRemainder(Polynomial.of(BigInteger.ONE,BigInteger.ZERO),ZP.identity());return true;});
            case "multivariate-division" -> probe("multivariate-division",()->{var f=new PrimeField(5);var r=new MultivariatePolynomialRing<>(f,2);var x=MultivariatePolynomial.monomial(1,1,0);var q=r.divide(x,x);return q.first.apply(Vector.of(0,0),f)==1&&q.second.apply(Vector.of(0,0),f)==0;});
            default -> throw new IllegalArgumentException(args[0]);
        }
        System.out.println("SUMMARY\t"+count);
    }
}
