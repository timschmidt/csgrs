import dk.jonaslindstrom.ruffini.finitefields.*;
import dk.jonaslindstrom.ruffini.finitefields.algorithms.BigTonelliShanks;
import dk.jonaslindstrom.ruffini.common.exceptions.NotASquareException;
import java.math.BigInteger;
import java.util.*;

/** V2 changes only fixture admission: BigPrimeField requires modulus > 3. */
public final class FiniteBoundariesV2 {
    static void constructors() {
        for(int p:new int[]{2,3}){
            String id="unsupported-constructor/"+p;FiniteBoundaries.start(id);
            try{new BigPrimeField(BigInteger.valueOf(p));FiniteBoundaries.result(id,"WRONG","accepted","modulus-greater-than-3");}
            catch(IllegalArgumentException e){FiniteBoundaries.result(id,"CONSTRUCTOR_UNSUPPORTED",e.getClass().getSimpleName(),"modulus-greater-than-3");}
        }
    }
    static void extensions() {
        for(int[] spec:new int[][]{{2,1,1,1},{2,1,0,1,1},{3,2,2,1},{5,2,0,1},{7,1,0,1}}){
            int p=spec[0];int[] mod=Arrays.copyOfRange(spec,1,spec.length);var o=new FiniteBoundaries.Oracle(p,mod);
            for(int scale=1;scale<p;scale++){
                int factor=scale;int[] scaled=Arrays.stream(mod).map(c->c*factor%p).toArray();
                var f=new FiniteField(new PrimeField(p),FiniteBoundaries.ints(scaled));
                BigPrimeField base=p>3?new BigPrimeField(BigInteger.valueOf(p)):null;
                BigFiniteField b=base==null?null:new BigFiniteField(base,FiniteBoundaries.bigs(scaled));
                var normalized=base==null?null:new AlgebraicFieldExtension<>(base,"a",FiniteBoundaries.bigs(scaled));
                for(int a=0;a<o.q;a++)for(String algorithm:base==null?List.of("int"):List.of("int","big","normalized")){
                    String id="inverse/"+p+"/"+FiniteBoundaries.csv(mod)+"/"+scale+"/"+a+"/"+algorithm;FiniteBoundaries.start(id);
                    String expected=Integer.toString(o.inverses[a]);
                    try {
                        String out=switch(algorithm){case "int"->FiniteBoundaries.raw(f.invert(FiniteBoundaries.ints(o.digits(a))));case "big"->FiniteBoundaries.raw(b.invert(FiniteBoundaries.bigs(o.digits(a))));default->FiniteBoundaries.raw(normalized.invert(FiniteBoundaries.bigs(o.digits(a))));};
                        FiniteBoundaries.result(id,FiniteBoundaries.exported(out,o)==o.inverses[a]?"PASS":"WRONG",out,expected);
                    } catch(StackOverflowError e){FiniteBoundaries.result(id,"EXCEPTION",e.getClass().getSimpleName(),expected);}
                    catch(RuntimeException|AssertionError e){FiniteBoundaries.result(id,a==0?"DOMAIN_REJECT":"EXCEPTION",e.getClass().getSimpleName(),expected);}
                }
            }
        }
    }
    static void bigSmall() {
        constructors();
        for(int p:new int[]{5,7,11,13,17,29,41,97}){
            FiniteBoundaries.require(FiniteBoundaries.prime(p));var sqrt=new BigTonelliShanks(new BigPrimeField(BigInteger.valueOf(p)));boolean[] squares=new boolean[p];
            for(int i=0;i<p;i++)squares[i*i%p]=true;
            for(int a=0;a<p;a++)for(int shift:new int[]{-1,0,1}){
                BigInteger input=BigInteger.valueOf(a+shift*p);String id="big-small/"+p+"/"+input;FiniteBoundaries.start(id);
                String expected=squares[a]?"SQUARE":"NONSQUARE";
                try{BigInteger out=sqrt.apply(input);FiniteBoundaries.result(id,out.multiply(out).mod(BigInteger.valueOf(p)).intValueExact()==a?"PASS":"WRONG",out.toString(),expected);}
                catch(NotASquareException e){FiniteBoundaries.result(id,squares[a]?"REJECTED_SQUARE":"PASS","NotASquareException",expected);}
                catch(Throwable e){FiniteBoundaries.result(id,"EXCEPTION",e.getClass().getSimpleName(),expected);}
            }
        }
    }
    public static void main(String[] args) throws Exception {
        switch(args[0]){case "extensions"->extensions();case "big-small"->bigSmall();default->{FiniteBoundaries.main(args);return;}}
        System.out.println("SUMMARY\t"+FiniteBoundaries.count);
    }
}
