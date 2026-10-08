import dk.jonaslindstrom.ruffini.finitefields.*;
import dk.jonaslindstrom.ruffini.finitefields.algorithms.*;
import dk.jonaslindstrom.ruffini.polynomials.elements.Polynomial;
import dk.jonaslindstrom.ruffini.common.exceptions.NotASquareException;
import java.math.BigInteger;
import java.util.*;

/** Local arithmetic qualification, not cryptographic validation or a benchmark. */
public final class FiniteBoundaries {
    static int count;
    static final int[][] BINARY={
        {0,1},{1,1,1},{1,0,1,1},{1,1,0,0,1},{1,0,1,0,0,1},
        {1,1,0,0,0,0,1},{1,1,0,0,0,0,0,1},{1,1,0,1,1,0,0,0,1}
    };
    static void require(boolean b) {if(!b)throw new AssertionError("independent oracle invariant");}
    static String csv(int[] a) {return Arrays.toString(a).replace(" ","");}
    static void start(String id) {System.out.println("START\t"+id);}
    static void result(String id,String status,String actual,String expected) {
        System.out.println("RESULT\t"+id+"\t"+status+"\t"+actual+"\t"+expected);count++;
    }
    static final class Oracle {
        final int p,n,q; final int[] modulus,inverses,roots;
        Oracle(int p,int[] modulus) {
            this.p=p;this.modulus=modulus.clone();n=modulus.length-1;
            require(modulus[n]==1);int order=1;for(int i=0;i<n;i++)order*=p;q=order;
            inverses=new int[q];roots=new int[q];Arrays.fill(inverses,-1);Arrays.fill(roots,-1);
            for(int a=0;a<q;a++)roots[multiply(a,a)]=a;
            for(int a=1;a<q;a++){for(int b=1;b<q;b++)if(multiply(a,b)==1){inverses[a]=b;break;}require(inverses[a]>=0);}
            require(roots[0]==0);
        }
        int[] digits(int a) {int[] out=new int[n];for(int i=0;i<n;i++){out[i]=a%p;a/=p;}require(a==0);return out;}
        int reduce(int[] input) {
            int[] a=Arrays.copyOf(input,Math.max(input.length,n));for(int i=0;i<a.length;i++)a[i]=Math.floorMod(a[i],p);
            for(int i=a.length-1;i>=n;i--)for(int j=0,c=a[i];j<=n;j++)a[i-n+j]=Math.floorMod(a[i-n+j]-c*modulus[j],p);
            int out=0;for(int i=n-1;i>=0;i--)out=out*p+a[i];return out;
        }
        int multiply(int a,int b) {
            int[] x=digits(a),y=digits(b),r=new int[2*n-1];
            for(int i=0;i<n;i++)for(int j=0;j<n;j++)r[i+j]+=x[i]*y[j];return reduce(r);
        }
    }
    static Polynomial<Integer> ints(int[] a) {return Polynomial.of(Arrays.stream(a).boxed().toArray(Integer[]::new));}
    static Polynomial<BigInteger> bigs(int[] a) {return Polynomial.of(Arrays.stream(a).mapToObj(BigInteger::valueOf).toArray(BigInteger[]::new));}
    static <T> String raw(Polynomial<T> p) {
        TreeMap<Integer,String> terms=new TreeMap<>();p.forEach((i,c)->{require(i>=0&&i<=128&&c!=null);terms.put(i,c.toString());});
        return terms.isEmpty()?"empty":String.join(",",terms.entrySet().stream().map(e->e.getKey()+":"+e.getValue()).toList());
    }
    static int exported(String raw,Oracle o) {
        if(raw.equals("empty"))return 0;int[] a=new int[129];
        for(String term:raw.split(",")){String[] pair=term.split(":");a[Integer.parseInt(pair[0])]=new BigInteger(pair[1]).mod(BigInteger.valueOf(o.p)).intValueExact();}
        return o.reduce(a);
    }
    static void extensions() {
        for(int[] spec:new int[][]{{2,1,1,1},{2,1,0,1,1},{3,2,2,1},{5,2,0,1}}){
            int p=spec[0];int[] mod=Arrays.copyOfRange(spec,1,spec.length);Oracle o=new Oracle(p,mod);
            for(int scale=1;scale<p;scale++){
                int factor=scale;int[] scaled=Arrays.stream(mod).map(c->c*factor%p).toArray();
                var f=new FiniteField(new PrimeField(p),ints(scaled));var base=new BigPrimeField(BigInteger.valueOf(p));
                var b=new BigFiniteField(base,bigs(scaled));var normalized=new AlgebraicFieldExtension<>(base,"a",bigs(scaled));
                for(int a=0;a<o.q;a++)for(String algorithm:List.of("int","big","normalized")){
                    String id="inverse/"+p+"/"+csv(mod)+"/"+scale+"/"+a+"/"+algorithm;start(id);
                    String expected=Integer.toString(o.inverses[a]);
                    try {
                        String out=switch(algorithm){case "int"->raw(f.invert(ints(o.digits(a))));case "big"->raw(b.invert(bigs(o.digits(a))));default->raw(normalized.invert(bigs(o.digits(a))));};
                        result(id,exported(out,o)==o.inverses[a]?"PASS":"WRONG",out,expected);
                    } catch(StackOverflowError e){result(id,"EXCEPTION",e.getClass().getSimpleName(),expected);}
                    catch(RuntimeException|AssertionError e){result(id,a==0?"DOMAIN_REJECT":"EXCEPTION",e.getClass().getSimpleName(),expected);}
                }
            }
        }
    }
    static void binary() {
        for(int n=1;n<=8;n++){
            Oracle o=new Oracle(2,BINARY[n-1]);require(Arrays.stream(o.roots).allMatch(x->x>=0));
            var f=new FiniteField(2,n);var sqrt=new TonelliShanks(f);
            for(int a=0;a<o.q;a++){
                String id="binary/"+n+"/"+a;start(id);String expected=Integer.toString(o.roots[a]);
                try{String out=raw(sqrt.apply(ints(o.digits(a))));result(id,exported(out,o)==o.roots[a]?"PASS":"WRONG",out,expected);}
                catch(Throwable e){result(id,"EXCEPTION",e.getClass().getSimpleName(),expected);}
            }
        }
    }
    static void bigSmall() {
        for(int p:new int[]{3,5,7,11,13,17,29,41,97}){
            require(prime(p));var sqrt=new BigTonelliShanks(new BigPrimeField(BigInteger.valueOf(p)));boolean[] squares=new boolean[p];
            for(int i=0;i<p;i++)squares[i*i%p]=true;
            for(int a=0;a<p;a++)for(int shift:new int[]{-1,0,1}){
                BigInteger input=BigInteger.valueOf(a+shift*p);String id="big-small/"+p+"/"+input;start(id);
                String expected=squares[a]?"SQUARE":"NONSQUARE";
                try{BigInteger out=sqrt.apply(input);result(id,out.multiply(out).mod(BigInteger.valueOf(p)).intValueExact()==a?"PASS":"WRONG",out.toString(),expected);}
                catch(NotASquareException e){result(id,squares[a]?"REJECTED_SQUARE":"PASS","NotASquareException",expected);}
                catch(Throwable e){result(id,"EXCEPTION",e.getClass().getSimpleName(),expected);}
            }
        }
    }
    static void odd(int p) throws Exception {
        require(prime(p));var member=TonelliShanks.class.getDeclaredField("random");member.setAccessible(true);
        var field=new FiniteField(new PrimeField(p),Polynomial.of(0,1));boolean[] squares=new boolean[p];for(int i=0;i<p;i++)squares[i*i%p]=true;
        for(int seed:new int[]{17,42,149})for(int a=0;a<p;a++){
            String id="odd/"+p+"/"+seed+"/"+a;start(id);String expected=squares[a]?"SQUARE":"NONSQUARE";
            var sqrt=new TonelliShanks(field);((Random)member.get(sqrt)).setSeed(seed);
            try{String out=raw(sqrt.apply(Polynomial.constant(a)));int value=exported(out,new Oracle(p,new int[]{0,1}));result(id,value*value%p==a?"PASS":"WRONG",out,expected);}
            catch(IllegalArgumentException e){result(id,squares[a]?"REJECTED_SQUARE":"PASS",e.getClass().getSimpleName(),expected);}
            catch(Throwable e){result(id,"EXCEPTION",e.getClass().getSimpleName(),expected);}
        }
    }
    static final class DrawBudget extends RuntimeException {}
    static final class BoundedRandom extends Random {
        int draws;BoundedRandom(int seed){super(seed);}
        @Override public int nextInt(int bound){if(++draws>256)throw new DrawBudget();return super.nextInt(bound);}
    }
    static void berlekamp(int p) {
        require(prime(p));for(int seed:new int[]{17,42,149})for(int b=0;b<p;b++)for(int c=0;c<p;c++){
            String id="berlekamp/"+p+"/"+seed+"/"+b+"/"+c;start(id);List<Integer> roots=new ArrayList<>();
            for(int x=0;x<p;x++)if((x*x+b*x+c)%p==0)roots.add(x);
            String expected=roots.toString().replace(" ","");
            try{int out=new BerlekampRabinAlgorithm(p,8,new BoundedRandom(seed)).apply(Polynomial.of(c,b,1));result(id,roots.contains(Math.floorMod(out,p))?"PASS":"WRONG",Integer.toString(out),expected);}
            catch(DrawBudget e){result(id,"HARNESS_BUDGET","DrawBudget",expected);}
            catch(IllegalArgumentException e){result(id,"UNRESOLVED","IterationLimit",expected);}
            catch(Throwable e){result(id,"EXCEPTION",e.getClass().getSimpleName(),expected);}
        }
    }
    static boolean prime(long p) {
        if(p<2)return false;if(p%2==0)return p==2;
        for(long d=3;d<=p/d;d+=2)if(p%d==0)return false;return true;
    }
    static void high(int s) {
        long p=0;int q=0;for(int candidate=1;candidate<100;candidate+=2){long n=((long)candidate<<s)+1;if(prime(n)){p=n;q=candidate;break;}}
        require(p>0);System.out.println("PRIME\t"+s+"\t"+p+"\t"+q+"\texhaustive-trial-division");
        BigInteger modulus=BigInteger.valueOf(p);var sqrt=new BigTonelliShanks(new BigPrimeField(modulus));
        for(int root:new int[]{2,3,5,17,31,65537}){
            BigInteger input=BigInteger.valueOf(root).pow(2).mod(modulus);String id="high/"+s+"/"+p+"/"+root;start(id);
            try{BigInteger out=sqrt.apply(input);result(id,out.multiply(out).mod(modulus).equals(input)?"PASS":"WRONG",out.toString(),input.toString());}
            catch(NotASquareException e){result(id,"REJECTED_SQUARE",e.getClass().getSimpleName(),input.toString());}
            catch(Throwable e){result(id,"EXCEPTION",e.getClass().getSimpleName(),input.toString());}
        }
    }
    public static void main(String[] args) throws Exception {
        switch(args[0]){
            case "extensions"->extensions();case "binary"->binary();case "big-small"->bigSmall();
            case "odd"->odd(Integer.parseInt(args[1]));case "berlekamp"->berlekamp(Integer.parseInt(args[1]));case "high"->high(Integer.parseInt(args[1]));
            default->throw new IllegalArgumentException(args[0]);
        }
        System.out.println("SUMMARY\t"+count);
    }
}
